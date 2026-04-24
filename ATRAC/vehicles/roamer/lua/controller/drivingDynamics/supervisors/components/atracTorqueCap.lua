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
-- Final drive ratio: read ONCE at init from rearDiff as a static constant.
--
-- SMOOTHING: Asymmetric -- fast attack, slow decay.
-- Fixes "needs a stop to reset" behaviour.
--
-- LOW-RANGE GATE: actAsTractionControl is a no-op unless transfer case
-- is in low range.
--
-- UI TOGGLE: The esc controller (loaded via roamer_ATRAC_esc_ui sub-part)
-- calls controller.applyConfig(configName, configData) on ALL controllers
-- when the user cycles the button. We expose applyConfig here and check
-- the config name to toggle isEnabled. This is the native callback --
-- no electrics field polling needed.

local M = {}
M.type = "auxiliary"
M.defaultOrder = 75
M.componentOrderTractionControl = 40
M.isActive = false

local max = math.max
local min = math.min

local CMUref = nil
local isDebugEnabled = false

local torqueSourceDevice = nil
local torqueSourceName   = "none"
local transferCaseDevice = nil
local finalDriveRatio    = 1.0
local drivenWheels       = {}
local drivenWheelCount   = 0

local smoothedTorque    = 0
local perWheelTorqueCap = 0
local isIntervening     = false
local isEnabled         = true

local escFlashTimer    = 0
local ESC_FLASH_PERIOD = 0.15
local escFlashState    = false

local controlParameters = {
  torqueCapSmoothingRateAttack = 25,
  torqueCapSmoothingRateDecay  = 3,
  torqueFloor                  = 400,
  safetyMarginCoef             = 1.05,
}
local initialControlParameters

local debugPacket = {sourceType = "atracTorqueCap"}
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

-- applyConfig: called by the esc controller whenever the user cycles
-- the button. configName is the string key from the configurations block
-- ("ATRAC On" or "ATRAC Off"). We use it to drive our enabled state.
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
  isIntervening = false
  if not torqueSourceDevice then return end

  local rawTorque = max(torqueSourceDevice.outputTorque or 0, 0)
  local rate = (rawTorque >= smoothedTorque)
               and controlParameters.torqueCapSmoothingRateAttack
               or  controlParameters.torqueCapSmoothingRateDecay

  smoothedTorque = smoothedTorque + (rawTorque - smoothedTorque) * min(rate * dt, 1)

  local effectiveTorque = max(smoothedTorque, controlParameters.torqueFloor)
  perWheelTorqueCap = (effectiveTorque * finalDriveRatio / drivenWheelCount)
                      * controlParameters.safetyMarginCoef
end

-- TC Component Interface

local function actAsTractionControl(wheelGroup, dt)
  if not isInLowRange() then return false end

  for i = 1, drivenWheelCount do
    local wd = drivenWheels[i]
    if wd.desiredBrakingTorque and wd.desiredBrakingTorque > perWheelTorqueCap then
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
  debugPacket.torqueSourceName  = torqueSourceName
  debugPacket.finalDriveRatio   = finalDriveRatio
  debugPacket.smoothedTorque    = smoothedTorque
  debugPacket.perWheelTorqueCap = perWheelTorqueCap
  debugPacket.drivenWheelCount  = drivenWheelCount
  debugPacket.torqueFloor       = controlParameters.torqueFloor
  debugPacket.safetyMarginCoef  = controlParameters.safetyMarginCoef
  debugPacket.isEnabled         = isEnabled
  debugPacket.isIntervening     = isIntervening
  debugPacket.inLowRange        = isInLowRange()
  debugPacket.rawTorque         = torqueSourceDevice and max(torqueSourceDevice.outputTorque, 0) or 0
  CMUref.sendDebugPacket(debugPacket)
end

-- Lifecycle

local function reset()
  smoothedTorque    = 0
  perWheelTorqueCap = controlParameters.torqueFloor * finalDriveRatio / math.max(drivenWheelCount, 1)
  isEnabled         = true
  isIntervening     = false
  escFlashTimer     = 0
  escFlashState     = false
end

local function init(jbeamData)
  if jbeamData.torqueCapSmoothingRateAttack ~= nil then
    controlParameters.torqueCapSmoothingRateAttack = jbeamData.torqueCapSmoothingRateAttack
  elseif jbeamData.torqueCapSmoothingRate ~= nil then
    controlParameters.torqueCapSmoothingRateAttack = jbeamData.torqueCapSmoothingRate
    controlParameters.torqueCapSmoothingRateDecay  = jbeamData.torqueCapSmoothingRate * 0.2
  end
  if jbeamData.torqueCapSmoothingRateDecay ~= nil then
    controlParameters.torqueCapSmoothingRateDecay = jbeamData.torqueCapSmoothingRateDecay
  end
  if jbeamData.torqueFloor      ~= nil then controlParameters.torqueFloor      = jbeamData.torqueFloor      end
  if jbeamData.safetyMarginCoef ~= nil then controlParameters.safetyMarginCoef = jbeamData.safetyMarginCoef end

  smoothedTorque    = 0
  perWheelTorqueCap = controlParameters.torqueFloor
  isEnabled         = true
  isIntervening     = false
  escFlashTimer     = 0
  escFlashState     = false

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

-- Native callback from the esc controller on button cycle.
M.applyConfig = applyConfig

return M
