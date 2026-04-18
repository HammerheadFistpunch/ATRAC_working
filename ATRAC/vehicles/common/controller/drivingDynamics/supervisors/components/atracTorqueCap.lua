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

local M = {}
M.type = "auxiliary"
M.defaultOrder = 75

-- Run after brakeControl (componentOrderTractionControl = 30).
M.componentOrderTractionControl = 40

M.isActive = false

local max = math.max
local min = math.min

local CMUref = nil
local isDebugEnabled = false

-- Torque source device -- transferCase preferred, gearbox fallback.
-- Both are pre-diff so unaffected by brakeControl downstream.
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

-- Parameters settable from jbeam.
local controlParameters = {
  -- Smoothing rate for torque source reading (1/s).
  -- High enough to track throttle and range changes quickly,
  -- low enough to reject driveline lash and gear change spikes.
  torqueCapSmoothingRate = 15,

  -- Minimum cap (Nm) when torque source reports zero or near-zero.
  -- Represents commanded wheel torque at idle in low range.
  -- Prevents ATRAC going fully passive at light throttle crawling.
  -- Default 400 Nm is appropriate for the roamer V8 at idle in low range
  -- after final drive multiplication. Tune down for lighter engines.
  torqueFloor = 400,

  -- Safety margin multiplier on the per-wheel torque cap.
  -- 1.0 = tightest stall protection (brake never exceeds wheel torque)
  -- 1.05 = 5% buffer for smoothing lag, negligible stall risk
  -- 1.25 = more TC authority, increased stall risk at very low RPM
  safetyMarginCoef = 1.05,
}
local initialControlParameters

local debugPacket = {sourceType = "atracTorqueCap"}
local configPacket = {sourceType = "atracTorqueCap", packetType = "config", config = controlParameters}

-- ─── Fixed Step ───────────────────────────────────────────────────────────────

local function updateFixedStep(dt)
  if not torqueSourceDevice then
    return
  end

  -- Read commanded torque at the output shaft of the torque source.
  -- Clamp to zero on overrun -- negative torque means engine braking,
  -- in which case the cap falls back to torqueFloor, not below zero.
  local rawTorque = max(torqueSourceDevice.outputTorque, 0)

  -- Smooth to reject driveline lash noise while tracking throttle quickly.
  smoothedTorque = smoothedTorque + (rawTorque - smoothedTorque) * min(controlParameters.torqueCapSmoothingRate * dt, 1)

  -- Apply torque floor so cap never collapses to zero at idle/light throttle.
  local effectiveTorque = max(smoothedTorque, controlParameters.torqueFloor)

  -- Multiply by static final drive ratio to get true commanded wheel torque.
  -- finalDriveRatio is read once at init -- static, no feedback loop.
  -- Divide equally across driven wheels (open diff assumption, conservative).
  perWheelTorqueCap = (effectiveTorque * finalDriveRatio / drivenWheelCount) * controlParameters.safetyMarginCoef
end

-- ─── TC Component Interface ───────────────────────────────────────────────────

-- Called by tractionControl supervisor after all other components have acted.
-- Walks every driven wheel and clamps desiredBrakingTorque to perWheelTorqueCap.
local function actAsTractionControl(wheelGroup, dt)
  for i = 1, drivenWheelCount do
    local wd = drivenWheels[i]
    if wd.desiredBrakingTorque and wd.desiredBrakingTorque > perWheelTorqueCap then
      wd.desiredBrakingTorque = perWheelTorqueCap
    end
  end
  -- Return false: we are a cap not an actuator. Returning true would
  -- incorrectly illuminate the TCS indicator during clamping only.
  return false
end

-- ─── GFX Update ───────────────────────────────────────────────────────────────

local function updateGFX(dt)
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
  CMUref.sendDebugPacket(debugPacket)
end

-- ─── Lifecycle ────────────────────────────────────────────────────────────────

local function reset()
  smoothedTorque = 0
  perWheelTorqueCap = controlParameters.torqueFloor * finalDriveRatio / drivenWheelCount
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

  M.isActive = true
end

local function initSecondStage(jbeamData)
  -- Discover driven wheels via isPropulsed flag.
  -- Captures rear wheels on RWD, all four on AWD/4WD, automatically.
  drivenWheels = {}
  for i = 0, wheels.wheelCount - 1 do
    local wd = wheels.wheels[i]
    if wd and wd.isPropulsed then
      table.insert(drivenWheels, wd)
    end
  end
  drivenWheelCount = #drivenWheels
  if drivenWheelCount == 0 then
    drivenWheelCount = 1 -- guard against division by zero
  end

  -- Find torque source device.
  -- Priority: transferCase (includes low range ratio) → gearbox (fallback).
  -- Both are pre-differential so completely unaffected by brakeControl.
  torqueSourceDevice = powertrain.getDevice("transferCase")
  if torqueSourceDevice then
    torqueSourceName = "transferCase"
  else
    torqueSourceDevice = powertrain.getDevice("gearbox")
    if torqueSourceDevice then
      torqueSourceName = "gearbox"
    end
  end

  -- Read final drive ratio from the rear differential as a static constant.
  -- This is a fixed geometric property of the diff -- it never changes at
  -- runtime so reading it here and storing it carries zero feedback risk.
  -- Try finalDriveRatio first, fall back to gearRatio (BeamNG uses both
  -- field names depending on the diff device implementation).
  local rearDiff = powertrain.getDevice("rearDiff")
  if rearDiff then
    finalDriveRatio = rearDiff.finalDriveRatio or rearDiff.gearRatio or 1.0
  else
    -- If no named rearDiff, try to find any differential device.
    local allDiffs = powertrain.getDevicesByType("differential")
    if allDiffs and #allDiffs > 0 then
      finalDriveRatio = allDiffs[1].finalDriveRatio or allDiffs[1].gearRatio or 1.0
    else
      finalDriveRatio = 1.0
    end
  end

  -- Register as a TC component so tractionControl supervisor calls
  -- actAsTractionControl each fixed step after brakeControl has acted.
  local tractionControl = CMUref and CMUref.getSupervisor("tractionControl")
  if tractionControl then
    tractionControl.registerComponent(M)
  end

  initialControlParameters = deepcopy(controlParameters)
end

local function initLastStage(jbeamData)
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

return M
