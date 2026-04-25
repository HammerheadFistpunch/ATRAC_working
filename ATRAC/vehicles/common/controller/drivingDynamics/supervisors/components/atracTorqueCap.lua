-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- ATRAC Torque Cap Controller
-- Runs as the final component in the tractionControl chain (order 40),
-- AFTER brakeControl (order 30) has already written desiredBrakingTorque
-- via its PID. We overwrite that value with our own bang-bang command.
--
-- SLIP SOURCE: wheelGroup.wheels[j].slip is populated by virtualSpeedSlip
-- before actAsTractionControl is called. We read it directly; we do not
-- re-compute slip ourselves.
--
-- CONTROL MODEL: per-wheel state machine.
--   IDLE     : slip below threshold. Do not touch desiredBrakingTorque.
--   ATTACK   : slip at or above threshold. Immediately write
--              wd.brakeTorque * attackFactor to desiredBrakingTorque.
--              Hard, no ramp. Stays here while slip >= slipThreshold.
--   RELEASE1 : slip just dropped below threshold. Fast drop to
--              wd.brakeTorque * releaseHoldFactor for releaseHoldTime
--              seconds. Prevents immediate re-spin.
--   RELEASE2 : linear bleed from releaseHoldFactor down to 0 over
--              releaseBleedTime seconds. Smooth handoff back to idle.
--
-- TORQUE CAP (updateFixedStep):
--   Still acts as a stall-protection ceiling on desiredBrakingTorque.
--   Attack: instant -- smoothedTorque = rawTorque, no lerp lag.
--   Decay:  linear at torqueDecayRate Nm/s when not intervening.
--           Frozen while interventionHoldTimer > 0.
--   safetyMarginCoef is 1.4 so the cap sits well above normal drivetrain
--   output and is rarely the limiting factor. It only steps in if the
--   bang-bang command would genuinely stall the engine.
--
-- TORQUE CAP UNITS:
--   torqueSourceDevice.outputTorque is post-transmission, post-low-range,
--   pre-differential (transfer case output).
--   perWheelTorqueCap = smoothedTorque * ASSUMED_FINAL_DRIVE
--                       / drivenWheelCount * safetyMarginCoef
--   This gives a per-wheel ceiling in the same Nm units as
--   wd.desiredBrakingTorque.
--
-- LOW-RANGE GATE: actAsTractionControl is a no-op unless the transfer
--   case is in low range.
--
-- AIRBORNE DETECTION: wheels with contactDepth <= 0.001 are skipped
--   (brakeControl retains authority) but still arm the intervention hold.
--
-- UI TOGGLE: The esc controller calls applyConfig on ALL controllers
--   when the user cycles the button. We check the config name to toggle
--   isEnabled.

local M = {}
M.type = "auxiliary"
M.defaultOrder = 75
M.componentOrderTractionControl = 40
M.isActive = false

local max  = math.max
local min  = math.min

-- Assumed final drive ratio (transfer case output to wheel).
-- Update if axle gearing changes. rearDiff read is kept for debug only.
local ASSUMED_FINAL_DRIVE = 4.0

-- Sound: looping clip played while ATRAC is actively intervening.
-- Place the file at:  vehicles/<your_vehicle>/sounds/atrac_active.ogg
local ATRAC_SOUND_PATH = "art/sound/vehicles/roamer/sounds/atrac_active.ogg"
local soundSource      = nil
local soundPlaying     = false

local CMUref         = nil
local isDebugEnabled = false

local torqueSourceDevice = nil
local torqueSourceName   = "none"
local transferCaseDevice = nil
local finalDriveRatio    = 1.0   -- debug log only; math uses ASSUMED_FINAL_DRIVE
local drivenWheelCount   = 0     -- used for torque cap math

-- Per-wheel bang-bang state. Keyed by wd.name. Populated in initSecondStage.
-- Fields: phase ("idle"|"attack"|"release1"|"release2"), releaseTimer (s)
local wheelStates = {}

local smoothedTorque        = 0
local perWheelTorqueCap     = 0
local isIntervening         = false
local interventionHoldTimer = 0
local isEnabled             = true

local escFlashTimer    = 0
local ESC_FLASH_PERIOD = 0.15
local escFlashState    = false

local controlParameters = {
  -- Slip sense
  slipThreshold     = 0.08,   -- slip ratio that triggers attack (0-0.5 scale from virtualSpeedSlip)

  -- Bang-bang brake factors (fraction of wd.brakeTorque)
  attackFactor      = 0.85,   -- applied immediately on slip onset; hard step
  releaseHoldFactor = 0.25,   -- reduced pressure during release phase 1

  -- Release timing (stored in seconds; jbeam supplies milliseconds)
  releaseHoldTime   = 0.12,   -- seconds at releaseHoldFactor before bleeding
  releaseBleedTime  = 0.25,   -- seconds to linearly ramp from holdFactor to 0

  -- Torque cap (stall protection backstop)
  torqueFloor       = 5500,   -- minimum cap in transfer-case-output Nm
  safetyMarginCoef  = 1.4,    -- cap = smoothedTorque * this; 1.4 keeps cap above normal output
  torqueDecayRate   = 8000,   -- linear decay of cap in Nm/s when not intervening

  -- Hold timer: freeze cap decay this long after last intervention
  interventionHoldTime = 0.4,
}
local initialControlParameters

local debugPacket  = {sourceType = "atracTorqueCap"}
local configPacket = {sourceType = "atracTorqueCap", packetType = "config", config = controlParameters}

-- ─── Low-Range Detection ──────────────────────────────────────────────────────

local function isInLowRange()
  if not transferCaseDevice then return true end

  local mode = transferCaseDevice.mode
  if type(mode) == "string" then
    return mode:lower():find("low") ~= nil
  end

  local gi = transferCaseDevice.gearIndex
  if type(gi) == "number" then return gi == 1 end

  local gr = transferCaseDevice.gearRatio
  if type(gr) == "number" then return gr > 1.5 end

  return true
end

-- ─── Enable / Disable ─────────────────────────────────────────────────────────

local function setEnabled(val)
  isEnabled = val
  local tc = CMUref and CMUref.getSupervisor("tractionControl")
  if tc then tc.setParameters({isEnabled = isEnabled}) end
end

local function applyConfig(configName, configData)
  if type(configName) == "string" then
    local name = configName:lower()
    if name:find("off") then
      setEnabled(false)
    else
      setEnabled(true)
    end
  end
end

-- ─── Torque Cap Fixed Step ────────────────────────────────────────────────────
-- Maintains perWheelTorqueCap as a stall-protection ceiling.
-- Runs each physics tick before actAsTractionControl.

local function updateFixedStep(dt)
  -- Arm / count down the hold timer from the PREVIOUS frame's state,
  -- then clear isIntervening for this frame (actAsTractionControl re-arms it).
  if isIntervening then
    interventionHoldTimer = controlParameters.interventionHoldTime
  elseif interventionHoldTimer > 0 then
    interventionHoldTimer = max(0, interventionHoldTimer - dt)
  end
  isIntervening = false

  if not torqueSourceDevice then return end

  local rawTorque = max(torqueSourceDevice.outputTorque or 0, 0)

  -- Attack: instant. No lerp. Cap tracks rising torque with zero lag.
  -- Decay:  linear at torqueDecayRate Nm/s, only when hold timer is clear.
  -- Hold:   frozen while interventionHoldTimer > 0 and torque is falling.
  if rawTorque >= smoothedTorque then
    smoothedTorque = rawTorque
  elseif interventionHoldTimer <= 0 then
    smoothedTorque = max(
      smoothedTorque - controlParameters.torqueDecayRate * dt,
      controlParameters.torqueFloor
    )
  end
  -- else: hold, no change

  local effectiveTorque = max(smoothedTorque, controlParameters.torqueFloor)
  perWheelTorqueCap = (effectiveTorque * ASSUMED_FINAL_DRIVE / drivenWheelCount)
                      * controlParameters.safetyMarginCoef
end

-- ─── TC Component Interface ───────────────────────────────────────────────────
-- Called by tractionControl supervisor after virtualSpeedSlip has populated
-- wheelGroup.wheels[j].slip for all wheels in this group.
-- We overwrite desiredBrakingTorque directly; we do not clip the upstream PID.

local function actAsTractionControl(wheelGroup, dt)
  if not isInLowRange() then return false end

  local didAct = false

  for i = 1, wheelGroup.wheelCount do
    local wheelData = wheelGroup.wheels[i]
    local wd        = wheelData.wd
    local slip      = wheelData.slip
    local state     = wheelStates[wd.name]

    -- Lazily create state for any wheel not seen at init.
    if not state then
      state = {phase = "idle", releaseTimer = 0}
      wheelStates[wd.name] = state
    end

    -- Airborne: no ground contact. Clear any active brake state, arm hold timer.
    local isAirborne = false
    if wd.contactDepth ~= nil then
      isAirborne = wd.contactDepth <= 0.001
    elseif wd.downForce ~= nil then
      isAirborne = wd.downForce <= 0
    end

    if isAirborne then
      state.phase        = "idle"
      state.releaseTimer = 0
      isIntervening      = true
      -- do not touch desiredBrakingTorque; brakeControl retains authority

    elseif slip >= controlParameters.slipThreshold then
      -- ── ATTACK ──────────────────────────────────────────────────────────────
      -- Wheel is slipping. Hard brake step, no ramp.
      -- Re-enters attack from any release phase: if the wheel re-spins during
      -- bleed we go immediately back to full brake.
      state.phase        = "attack"
      state.releaseTimer = 0
      wd.desiredBrakingTorque = min(
        wd.brakeTorque * controlParameters.attackFactor,
        perWheelTorqueCap
      )
      isIntervening = true
      didAct        = true

    elseif state.phase == "attack" then
      -- ── RELEASE PHASE 1 (fast drop) ─────────────────────────────────────────
      -- Slip just cleared. Drop to holdFactor immediately; hold for
      -- releaseHoldTime to prevent instant re-spin.
      state.phase        = "release1"
      state.releaseTimer = controlParameters.releaseHoldTime
      wd.desiredBrakingTorque = min(
        wd.brakeTorque * controlParameters.releaseHoldFactor,
        perWheelTorqueCap
      )
      isIntervening = true
      didAct        = true

    elseif state.phase == "release1" then
      -- ── RELEASE PHASE 1 (hold) ───────────────────────────────────────────────
      state.releaseTimer = state.releaseTimer - dt
      if state.releaseTimer <= 0 then
        state.phase        = "release2"
        state.releaseTimer = controlParameters.releaseBleedTime
      end
      wd.desiredBrakingTorque = min(
        wd.brakeTorque * controlParameters.releaseHoldFactor,
        perWheelTorqueCap
      )
      isIntervening = true
      didAct        = true

    elseif state.phase == "release2" then
      -- ── RELEASE PHASE 2 (linear bleed) ──────────────────────────────────────
      -- Ramp from releaseHoldFactor linearly to 0 over releaseBleedTime.
      state.releaseTimer = state.releaseTimer - dt
      if state.releaseTimer <= 0 then
        state.phase        = "idle"
        state.releaseTimer = 0
        -- Back to idle: stop writing desiredBrakingTorque.
      else
        local bleedFraction = state.releaseTimer / controlParameters.releaseBleedTime
        wd.desiredBrakingTorque = min(
          wd.brakeTorque * controlParameters.releaseHoldFactor * bleedFraction,
          perWheelTorqueCap
        )
        isIntervening = true
        didAct        = true
      end

    end
    -- phase == "idle": do nothing. brakeControl's PID value stands.
  end

  return didAct
end

-- ─── GFX Update ───────────────────────────────────────────────────────────────
-- Overwrites electrics.values.esc each frame. Runs after esc controller.
--   SOLID 1 = system disabled (warning)
--   FLASH   = active intervention
--   OFF 0   = enabled, idle

local function updateGFX(dt)
  if not isEnabled then
    electrics.values.esc       = 1
    electrics.values.escActive = false
    escFlashTimer = 0
    escFlashState = false
    if soundPlaying and soundSource then
      soundSource:stop()
      soundPlaying = false
    end
    return
  end

  if isIntervening then
    if not soundPlaying and soundSource then
      soundSource:play()
      soundPlaying = true
    end
    escFlashTimer = escFlashTimer + dt
    if escFlashTimer >= ESC_FLASH_PERIOD then
      escFlashTimer = escFlashTimer - ESC_FLASH_PERIOD
      escFlashState = not escFlashState
    end
    electrics.values.esc       = escFlashState and 1 or 0
    electrics.values.escActive = true
  else
    if soundPlaying and interventionHoldTimer <= 0 and soundSource then
      soundSource:stop()
      soundPlaying = false
    end
    escFlashTimer = 0
    escFlashState = false
    electrics.values.esc       = 0
    electrics.values.escActive = false
  end
end

local function updateGFXDebug(dt)
  updateGFX(dt)
  debugPacket.torqueSourceName      = torqueSourceName
  debugPacket.assumedFinalDrive     = ASSUMED_FINAL_DRIVE
  debugPacket.readFinalDriveRatio   = finalDriveRatio
  debugPacket.smoothedTorque        = smoothedTorque
  debugPacket.perWheelTorqueCap     = perWheelTorqueCap
  debugPacket.drivenWheelCount      = drivenWheelCount
  debugPacket.torqueFloor           = controlParameters.torqueFloor
  debugPacket.safetyMarginCoef      = controlParameters.safetyMarginCoef
  debugPacket.torqueDecayRate       = controlParameters.torqueDecayRate
  debugPacket.interventionHoldTimer = interventionHoldTimer
  debugPacket.isEnabled             = isEnabled
  debugPacket.isIntervening         = isIntervening
  debugPacket.inLowRange            = isInLowRange()
  debugPacket.rawTorque             = torqueSourceDevice and max(torqueSourceDevice.outputTorque, 0) or 0
  debugPacket.wheelStates           = {}
  for name, state in pairs(wheelStates) do
    debugPacket.wheelStates[name] = {phase = state.phase, releaseTimer = state.releaseTimer}
  end
  CMUref.sendDebugPacket(debugPacket)
end

-- ─── Lifecycle ────────────────────────────────────────────────────────────────

local function resetWheelStates()
  for _, state in pairs(wheelStates) do
    state.phase        = "idle"
    state.releaseTimer = 0
  end
end

local function reset()
  smoothedTorque        = 0
  perWheelTorqueCap     = controlParameters.torqueFloor * ASSUMED_FINAL_DRIVE
                          / math.max(drivenWheelCount, 1)
  isEnabled             = true
  isIntervening         = false
  interventionHoldTimer = 0
  escFlashTimer         = 0
  escFlashState         = false
  resetWheelStates()
  if soundSource and soundPlaying then
    soundSource:stop()
  end
  soundPlaying = false
end

local function init(jbeamData)
  -- Slip sense
  if jbeamData.slipThreshold      ~= nil then controlParameters.slipThreshold      = jbeamData.slipThreshold      end

  -- Bang-bang factors
  if jbeamData.attackFactor       ~= nil then controlParameters.attackFactor       = jbeamData.attackFactor       end
  if jbeamData.releaseHoldFactor  ~= nil then controlParameters.releaseHoldFactor  = jbeamData.releaseHoldFactor  end

  -- Release timing (jbeam in ms)
  if jbeamData.releaseHoldTime    ~= nil then controlParameters.releaseHoldTime    = jbeamData.releaseHoldTime    / 1000 end
  if jbeamData.releaseBleedTime   ~= nil then controlParameters.releaseBleedTime   = jbeamData.releaseBleedTime   / 1000 end

  -- Torque cap
  if jbeamData.torqueFloor        ~= nil then controlParameters.torqueFloor        = jbeamData.torqueFloor        end
  if jbeamData.safetyMarginCoef   ~= nil then controlParameters.safetyMarginCoef   = jbeamData.safetyMarginCoef   end
  if jbeamData.torqueDecayRate    ~= nil then controlParameters.torqueDecayRate     = jbeamData.torqueDecayRate    end
  if jbeamData.interventionHoldTime ~= nil then controlParameters.interventionHoldTime = jbeamData.interventionHoldTime / 1000 end

  smoothedTorque        = 0
  perWheelTorqueCap     = controlParameters.torqueFloor
  isEnabled             = true
  isIntervening         = false
  interventionHoldTimer = 0
  escFlashTimer         = 0
  escFlashState         = false

  M.isActive = true
end

local function initSecondStage(jbeamData)
  -- Count driven wheels and pre-populate wheel states.
  local drivenCount = 0
  wheelStates = {}
  for i = 0, wheels.wheelCount - 1 do
    local wd = wheels.wheels[i]
    if wd and wd.isPropulsed then
      drivenCount = drivenCount + 1
      wheelStates[wd.name] = {phase = "idle", releaseTimer = 0}
    end
  end
  drivenWheelCount = math.max(drivenCount, 1)

  torqueSourceDevice = powertrain.getDevice("transferCase")
  if torqueSourceDevice then
    torqueSourceName = "transferCase"
  else
    torqueSourceDevice = powertrain.getDevice("gearbox")
    torqueSourceName   = torqueSourceDevice and "gearbox" or "none"
  end

  transferCaseDevice = powertrain.getDevice("transferCase")

  -- Read finalDriveRatio for debug logging only.
  local rearDiff = powertrain.getDevice("rearDiff")
  if rearDiff then
    finalDriveRatio = rearDiff.finalDriveRatio or rearDiff.gearRatio or 1.0
  else
    local allDiffs = powertrain.getDevicesByType("differential")
    if allDiffs and #allDiffs > 0 then
      finalDriveRatio = allDiffs[1].finalDriveRatio or allDiffs[1].gearRatio or 1.0
    else
      finalDriveRatio = 1.0
    end
  end

  local tractionControl = CMUref and CMUref.getSupervisor("tractionControl")
  if tractionControl then
    tractionControl.registerComponent(M)
  end

  controller.setParameter = controller.setParameter or function() end
  initialControlParameters = deepcopy(controlParameters)

  soundSource  = obj:createSFXSource(ATRAC_SOUND_PATH, "AudioDefaultLoop3D")
  soundPlaying = false
end

local function initLastStage(jbeamData)
end

local function shutdown()
  M.isActive        = false
  M.updateGFX       = nil
  M.updateFixedStep = nil
  if soundSource then
    soundSource:stop()
    soundSource = nil
  end
  soundPlaying = false
end

-- ─── Debug & Config Interface ─────────────────────────────────────────────────

local function setDebugMode(debugEnabled)
  isDebugEnabled = debugEnabled
  M.updateGFX = isDebugEnabled and updateGFXDebug or updateGFX
end

local function registerCMU(cmu)
  CMUref = cmu
end

local function setParameters(parameters)
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "slipThreshold")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "attackFactor")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "releaseHoldFactor")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "releaseHoldTime")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "releaseBleedTime")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "torqueFloor")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "safetyMarginCoef")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "torqueDecayRate")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "interventionHoldTime")
end

local function setConfig(configTable)  controlParameters = configTable end
local function getConfig()             return deepcopy(controlParameters) end
local function sendConfigData()
  configPacket.config = controlParameters
  CMUref.sendDebugPacket(configPacket)
end

-- ─── Exports ──────────────────────────────────────────────────────────────────

M.init                 = init
M.initSecondStage      = initSecondStage
M.initLastStage        = initLastStage
M.reset                = reset
M.updateGFX            = updateGFX
M.updateFixedStep      = updateFixedStep
M.registerCMU          = registerCMU
M.setDebugMode         = setDebugMode
M.shutdown             = shutdown
M.setParameters        = setParameters
M.setConfig            = setConfig
M.getConfig            = getConfig
M.sendConfigData       = sendConfigData
M.actAsTractionControl = actAsTractionControl
M.applyConfig          = applyConfig

return M
