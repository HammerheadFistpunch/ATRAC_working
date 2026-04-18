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
--   - Converts transferCase output to true wheel torque              ✓
--   - Static property, never changes at runtime, no feedback loop    ✓
--
-- Per-wheel cap = (smoothedTorque * finalDriveRatio / drivenWheelCount)
--                * safetyMarginCoef
--
-- brakeControl.desiredBrakingTorque is clamped to this value each fixed
-- step, ensuring net wheel torque never goes negative and the engine
-- cannot stall due to ATRAC brake intervention.
--
-- UI toggle: registers with the standard tcsToggle input action so the
-- TCS button appears in the vehicle controls panel.

local M = {}
M.type = "auxiliary"
M.defaultOrder = 75

-- Run after brakeControl (componentOrderTractionControl = 30).
M.componentOrderTractionControl = 40

M.isActive = false

local max = math.max
local min = math.min
local floor = math.floor

local CMUref = nil
local isDebugEnabled = false

-- Torque source device -- transferCase preferred, gearbox fallback.
local torqueSourceDevice = nil
local torqueSourceName = "none"

-- Final drive ratio read once at init from the diff device.
-- Static geometric property -- never changes at runtime.
local finalDriveRatio = 1.0

-- Driven wheels discovered at init via isPropulsed.
local drivenWheels = {}
local drivenWheelCount = 0

-- Smoothed torque at the torque source device output shaft.
local smoothedTorque = 0

-- Per-wheel commanded torque cap in Nm, updated each fixed step.
local perWheelTorqueCap = 0

-- Toggle state -- mirrors tractionControl.isEnabled for UI feedback
local isEnabled = true
local isActiveSmoothed = 0
local isActiveSmoother = newTemporalSmoothing(10, 5)

-- Parameters settable from jbeam.
local controlParameters = {
  torqueCapSmoothingRate = 15,
  torqueFloor = 400,
  safetyMarginCoef = 1.05,
}
local initialControlParameters

local debugPacket = {sourceType = "atracTorqueCap"}
local configPacket = {sourceType = "atracTorqueCap", packetType = "config", config = controlParameters}

-- ─── UI Toggle ────────────────────────────────────────────────────────────────

-- Called by the tcsToggle input action (vehicle controls panel button).
-- Toggles ATRAC on/off by enabling/disabling the tractionControl supervisor.
-- Updates electrics so the TCS dash indicator and UI button reflect state.
local function toggleATRAC(val)
  -- BeamNG input actions call with val=1 on press, val=0 on release.
  -- We only act on the down press.
  if val ~= 1 then return end

  local tc = CMUref and CMUref.getSupervisor("tractionControl")
  if not tc then return end

  isEnabled = not isEnabled
  tc.setParameters({isEnabled = isEnabled})

  -- When disabled, hold tcs indicator on solid to show system is off.
  -- When enabled, let tractionControl manage the pulsing itself.
  if not isEnabled then
    electrics.values.tcs = 1
  end

  -- Notify the UI so the button highlight updates immediately.
  guihooks.message(
    {
      txt = isEnabled and "vehicle.atrac.enabled" or "vehicle.atrac.disabled",
      context = {}
    },
    2,
    "vehicle.atrac.toggle"
  )
end

-- ─── Fixed Step ───────────────────────────────────────────────────────────────

local function updateFixedStep(dt)
  if not torqueSourceDevice then
    return
  end

  local rawTorque = max(torqueSourceDevice.outputTorque, 0)
  smoothedTorque = smoothedTorque + (rawTorque - smoothedTorque) * min(controlParameters.torqueCapSmoothingRate * dt, 1)
  local effectiveTorque = max(smoothedTorque, controlParameters.torqueFloor)
  perWheelTorqueCap = (effectiveTorque * finalDriveRatio / drivenWheelCount) * controlParameters.safetyMarginCoef
end

-- ─── TC Component Interface ───────────────────────────────────────────────────

local function actAsTractionControl(wheelGroup, dt)
  for i = 1, drivenWheelCount do
    local wd = drivenWheels[i]
    if wd.desiredBrakingTorque and wd.desiredBrakingTorque > perWheelTorqueCap then
      wd.desiredBrakingTorque = perWheelTorqueCap
    end
  end
  return false
end

-- ─── GFX Update ───────────────────────────────────────────────────────────────

local function updateGFX(dt)
  -- Keep electrics in sync with enabled state so dash indicator
  -- and UI button highlight correctly reflect ATRAC status.
  electrics.values.hasTCS = true
  if not isEnabled then
    electrics.values.tcs = 1
    electrics.values.tcsActive = false
  end
end

local function updateGFXDebug(dt)
  updateGFX(dt)
  debugPacket.torqueSourceName = torqueSourceName
  debugPacket.finalDriveRatio = finalDriveRatio
  debugPacket.smoothedTorque = smoothedTorque
  debugPacket.perWheelTorqueCap = perWheelTorqueCap
  debugPacket.drivenWheelCount = drivenWheelCount
  debugPacket.torqueFloor = controlParameters.torqueFloor
  debugPacket.safetyMarginCoef = controlParameters.safetyMarginCoef
  debugPacket.isEnabled = isEnabled
  CMUref.sendDebugPacket(debugPacket)
end

-- ─── Lifecycle ────────────────────────────────────────────────────────────────

local function reset()
  smoothedTorque = 0
  perWheelTorqueCap = controlParameters.torqueFloor * finalDriveRatio / drivenWheelCount
  isEnabled = true
end

local function init(jbeamData)
  if jbeamData.torqueCapSmoothingRate ~= nil then
    controlParameters.torqueCapSmoothingRate = jbeamData.torqueCapSmoothingRate
  end
  if jbeamData.torqueFloor ~= nil then
    controlParameters.torqueFloor = jbeamData.torqueFloor
  end
  if jbeamData.safetyMarginCoef ~= nil then
    controlParameters.safetyMarginCoef = jbeamData.safetyMarginCoef
  end

  smoothedTorque = 0
  perWheelTorqueCap = controlParameters.torqueFloor
  isEnabled = true

  M.isActive = true
end

local function initSecondStage(jbeamData)
  -- Discover driven wheels via isPropulsed flag.
  drivenWheels = {}
  for i = 0, wheels.wheelCount - 1 do
    local wd = wheels.wheels[i]
    if wd and wd.isPropulsed then
      table.insert(drivenWheels, wd)
    end
  end
  drivenWheelCount = #drivenWheels
  if drivenWheelCount == 0 then
    drivenWheelCount = 1
  end

  -- Find torque source device.
  -- Priority: transferCase → gearbox fallback.
  torqueSourceDevice = powertrain.getDevice("transferCase")
  if torqueSourceDevice then
    torqueSourceName = "transferCase"
  else
    torqueSourceDevice = powertrain.getDevice("gearbox")
    if torqueSourceDevice then
      torqueSourceName = "gearbox"
    end
  end

  -- Read final drive ratio once as a static constant.
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

  -- Register as TC component.
  local tractionControl = CMUref and CMUref.getSupervisor("tractionControl")
  if tractionControl then
    tractionControl.registerComponent(M)
  end

  -- Register tcsToggle input action so the vehicle controls panel
  -- shows a TCS button that calls our toggleATRAC function.
  -- hasTCS tells the UI the button should exist.
  electrics.values.hasTCS = true
  controller.setParameter = controller.setParameter or function() end

  initialControlParameters = deepcopy(controlParameters)
end

local function initLastStage(jbeamData)
  -- Bind the tcsToggle input action to our toggle function.
  -- This is what wires the vehicle controls panel button to ATRAC.
  input.bindAction("tcsToggle", toggleATRAC)
end

local function shutdown()
  M.isActive = false
  M.updateGFX = nil
  M.updateFixedStep = nil
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
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "torqueCapSmoothingRate")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "torqueFloor")
  CMUref.applyParameter(controlParameters, initialControlParameters, parameters, "safetyMarginCoef")
end

local function setConfig(configTable)
  controlParameters = configTable
end

local function getConfig()
  return deepcopy(controlParameters)
end

local function sendConfigData()
  configPacket.config = controlParameters
  CMUref.sendDebugPacket(configPacket)
end

-- ─── Exports ──────────────────────────────────────────────────────────────────

M.init = init
M.initSecondStage = initSecondStage
M.initLastStage = initLastStage

M.reset = reset

M.updateGFX = updateGFX
M.updateFixedStep = updateFixedStep

M.registerCMU = registerCMU
M.setDebugMode = setDebugMode
M.shutdown = shutdown
M.setParameters = setParameters
M.setConfig = setConfig
M.getConfig = getConfig
M.sendConfigData = sendConfigData

M.actAsTractionControl = actAsTractionControl
M.toggleATRAC = toggleATRAC

return M
