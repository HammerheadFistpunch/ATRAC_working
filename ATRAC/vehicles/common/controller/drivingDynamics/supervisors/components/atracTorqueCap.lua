-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- ATRAC Torque Cap Controller
-- Runs as the final component in the tractionControl chain.
--
-- Torque read point: transferCase.outputTorque (falls back to gearbox)
--   - Post-gearbox ratio        ✓
--   - Post-low-range ratio      ✓
--   - Pre-differential          ✓  (no feedback loop from brakeControl)
--
-- Final drive ratio: assumed 4.0 (ASSUMED_FINAL_DRIVE constant).
-- The rearDiff device is unreliable at init; hardcoding avoids silent
-- mis-reads. Update the constant if axle gearing changes. The read
-- from rearDiff is kept for debug logging only.
--
-- SMOOTHING: Asymmetric -- fast attack, intervention-gated decay.
--   Attack  : always runs, tracks rising drivetrain torque immediately.
--   Decay   : blocked while isIntervening == true OR within
--             interventionHoldTime seconds of the last intervention.
--             This prevents the cap from bleeding down while the wheel
--             is still slipping, eliminating the grab-release oscillation.
--
-- LOW-RANGE GATE: actAsTractionControl is a no-op unless the transfer
-- case is in low range.
--
-- AIRBORNE DETECTION: wheels with contactDepth <= 0 have no ground
-- contact. The torque cap is bypassed for those wheels, allowing
-- brakeControl full authority. They still count as intervening so the
-- hold timer stays armed.
--
-- UI TOGGLE: The esc controller calls applyConfig on ALL controllers
-- when the user cycles the button. We check the config name to toggle
-- isEnabled.

local M = {}
M.type = "auxiliary"
M.defaultOrder = 75
M.componentOrderTractionControl = 40
M.isActive = false

local max = math.max
local min = math.min

-- Assumed final drive ratio (ring gear to wheel).
-- transferCase.outputTorque is already post-transmission and post-low-range,
-- so this is the only remaining multiplication to reach wheel torque.
-- For 4 driven wheels: ASSUMED_FINAL_DRIVE / drivenWheelCount = 4/4 = 1,
-- so perWheelTorqueCap ≈ smoothedTorque × safetyMarginCoef at that config.
-- The constant is kept explicit so the math stays correct if wheel count
-- ever differs from 4.
local ASSUMED_FINAL_DRIVE = 4.0

local CMUref = nil
local isDebugEnabled = false

local torqueSourceDevice = nil
local torqueSourceName   = "none"
local transferCaseDevice = nil
local finalDriveRatio    = 1.0   -- read from rearDiff for debug logging only
local drivenWheels       = {}
local drivenWheelCount   = 0

local smoothedTorque        = 0
local perWheelTorqueCap     = 0
local isIntervening         = false
local interventionHoldTimer = 0
local isEnabled             = true

local escFlashTimer    = 0
local ESC_FLASH_PERIOD = 0.15
local escFlashState    = false

local controlParameters = {
  torqueCapSmoothingRateAttack = 25,
  torqueCapSmoothingRateDecay  = 1.5,
  torqueFloor                  = 5500,
  safetyMarginCoef             = 1.05,
  interventionHoldTime         = 0.4,
}
local initialControlParameters

local debugPacket  = {sourceType = "atracTorqueCap"}
local configPacket = {sourceType = "atracTorqueCap", packetType = "config", config = controlParameters}

-- Low-Range Detection

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

-- Internal enable/disable

local function setEnabled(val)
  isEnabled = val
  local tc = CMUref and CMUref.getSupervisor("tractionControl")
  if tc then tc.setParameters({isEnabled = isEnabled}) end
end

-- applyConfig: called by the esc controller on button cycle.
-- configName is the string key from the configurations block
-- ("ATRAC On" or "ATRAC Off").
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

-- Fixed Step

local function updateFixedStep(dt)
  -- Update the hold timer from the PREVIOUS frame's intervention state,
  -- then reset the flag for this frame. actAsTractionControl will re-arm
  -- it if it clamps any wheel this frame.
  if isIntervening then
    interventionHoldTimer = controlParameters.interventionHoldTime
  elseif interventionHoldTimer > 0 then
    interventionHoldTimer = max(0, interventionHoldTimer - dt)
  end
  isIntervening = false

  if not torqueSourceDevice then return end

  local rawTorque = max(torqueSourceDevice.outputTorque or 0, 0)

  -- Attack: always runs at full rate, tracks rising torque immediately.
  -- Decay: only runs when the hold timer has fully expired.
  --        rate = 0 while timer is active → cap stays frozen.
  local rate
  if rawTorque >= smoothedTorque then
    rate = controlParameters.torqueCapSmoothingRateAttack
  elseif interventionHoldTimer <= 0 then
    rate = controlParameters.torqueCapSmoothingRateDecay
  else
    rate = 0
  end

  smoothedTorque = smoothedTorque + (rawTorque - smoothedTorque) * min(rate * dt, 1)

  -- torqueFloor is in transfer-case-output Nm (same units as outputTorque).
  -- Multiply by ASSUMED_FINAL_DRIVE and divide by wheel count to reach
  -- per-wheel brake authority.
  local effectiveTorque = max(smoothedTorque, controlParameters.torqueFloor)
  perWheelTorqueCap = (effectiveTorque * ASSUMED_FINAL_DRIVE / drivenWheelCount)
                      * controlParameters.safetyMarginCoef
end

-- TC Component Interface

local function actAsTractionControl(wheelGroup, dt)
  if not isInLowRange() then return false end

  for i = 1, drivenWheelCount do
    local wd = drivenWheels[i]

    -- Airborne detection.
    -- contactDepth > 0 means the tyre is deformed against a surface.
    -- If the field is absent fall back to downForce, and if that is also
    -- absent treat the wheel as grounded (fail safe).
    local isAirborne = false
    if wd.contactDepth ~= nil then
      isAirborne = wd.contactDepth <= 0.001
    elseif wd.downForce ~= nil then
      isAirborne = wd.downForce <= 0
    end

    if isAirborne then
      -- No ground contact: no traction possible regardless of torque.
      -- Skip the torque cap entirely so brakeControl can apply full authority.
      -- Still count as intervening to keep the hold timer armed.
      isIntervening = true
    elseif wd.desiredBrakingTorque and wd.desiredBrakingTorque > perWheelTorqueCap then
      wd.desiredBrakingTorque = perWheelTorqueCap
      isIntervening = true
    end
  end
  return false
end

-- GFX Update
-- Overwrites electrics.values.esc each frame with our indicator logic.
-- We run after the esc controller so our value wins.
--   OFF     = enabled, idle
--   FLASH   = enabled, actively intervening
--   SOLID   = disabled (system off warning)

local function updateGFX(dt)
  if not isEnabled then
    electrics.values.esc       = 1
    electrics.values.escActive = false
    escFlashTimer = 0
    escFlashState = false
    return
  end

  if isIntervening then
    escFlashTimer = escFlashTimer + dt
    if escFlashTimer >= ESC_FLASH_PERIOD then
      escFlashTimer = escFlashTimer - ESC_FLASH_PERIOD
      escFlashState = not escFlashState
    end
    electrics.values.esc       = escFlashState and 1 or 0
    electrics.values.escActive = true
  else
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
  debugPacket.readFinalDriveRatio   = finalDriveRatio        -- logged, not used in math
  debugPacket.smoothedTorque        = smoothedTorque
  debugPacket.perWheelTorqueCap     = perWheelTorqueCap
  debugPacket.drivenWheelCount      = drivenWheelCount
  debugPacket.torqueFloor           = controlParameters.torqueFloor
  debugPacket.safetyMarginCoef      = controlParameters.safetyMarginCoef
  debugPacket.interventionHoldTimer = interventionHoldTimer
  debugPacket.isEnabled             = isEnabled
  debugPacket.isIntervening         = isIntervening
  debugPacket.inLowRange            = isInLowRange()
  debugPacket.rawTorque             = torqueSourceDevice and max(torqueSourceDevice.outputTorque, 0) or 0
  CMUref.sendDebugPacket(debugPacket)
end

-- Lifecycle

local function reset()
  smoothedTorque        = 0
  perWheelTorqueCap     = controlParameters.torqueFloor * ASSUMED_FINAL_DRIVE / math.max(drivenWheelCount, 1)
  isEnabled             = true
  isIntervening         = false
  interventionHoldTimer = 0
  escFlashTimer         = 0
  escFlashState         = false
end

local function init(jbeamData)
  -- Support separate attack / decay keys only.
  -- The legacy torqueCapSmoothingRate key is intentionally NOT supported:
  -- it collapsed the asymmetric rates into one value and was the root cause
  -- of the grab-release oscillation.
  if jbeamData.torqueCapSmoothingRateAttack ~= nil then
    controlParameters.torqueCapSmoothingRateAttack = jbeamData.torqueCapSmoothingRateAttack
  end
  if jbeamData.torqueCapSmoothingRateDecay  ~= nil then
    controlParameters.torqueCapSmoothingRateDecay  = jbeamData.torqueCapSmoothingRateDecay
  end
  if jbeamData.torqueFloor          ~= nil then controlParameters.torqueFloor          = jbeamData.torqueFloor          end
  if jbeamData.safetyMarginCoef     ~= nil then controlParameters.safetyMarginCoef     = jbeamData.safetyMarginCoef     end
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
  drivenWheels = {}
  for i = 0, wheels.wheelCount - 1 do
    local wd = wheels.wheels[i]
    if wd and wd.isPropulsed then
      table.insert(drivenWheels, wd)
    end
  end
  drivenWheelCount = math.max(#drivenWheels, 1)

  torqueSourceDevice = powertrain.getDevice("transferCase")
  if torqueSourceDevice then
    torqueSourceName = "transferCase"
  else
    torqueSourceDevice = powertrain.getDevice("gearbox")
    torqueSourceName   = torqueSourceDevice and "gearbox" or "none"
  end

  transferCaseDevice = powertrain.getDevice("transferCase")

  -- Read finalDriveRatio for debug logging only.
  -- All torque cap math uses ASSUMED_FINAL_DRIVE instead.
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
end

local function initLastStage(jbeamData)
end

local function shutdown()
  M.isActive        = false
  M.updateGFX       = nil
  M.updateFixedStep = nil
end

-- Debug & Config Interface

local function setDebugMode(debugEnabled)
  isDebugEnabled = debugEnabled
  M.updateGFX = isDebugEnabled and updateGFXDebug or updateGFX
end

local function registerCMU(cmu)
  CMUref = cmu
end

local function setParameters(parameters)
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "torqueCapSmoothingRateAttack")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "torqueCapSmoothingRateDecay")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "torqueFloor")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "safetyMarginCoef")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "interventionHoldTime")
end

local function setConfig(configTable)  controlParameters = configTable end
local function getConfig()             return deepcopy(controlParameters) end
local function sendConfigData()
  configPacket.config = controlParameters
  CMUref.sendDebugPacket(configPacket)
end

-- Exports

M.init            = init
M.initSecondStage = initSecondStage
M.initLastStage   = initLastStage
M.reset           = reset
M.updateGFX       = updateGFX
M.updateFixedStep = updateFixedStep
M.registerCMU     = registerCMU
M.setDebugMode    = setDebugMode
M.shutdown        = shutdown
M.setParameters   = setParameters
M.setConfig       = setConfig
M.getConfig       = getConfig
M.sendConfigData  = sendConfigData
M.actAsTractionControl = actAsTractionControl
M.applyConfig     = applyConfig

return M
