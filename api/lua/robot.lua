local robot = {}



--- Enables or disables specific hardware modules on the robot.
--
-- Configures the active state of various sensors, LEDs, and actuators 
-- by passing a configuration table mapping module names to boolean values.
--
-- @tparam table options Key-value table specifying the desired state for each module (`true` to enable, `false` to disable).
-- @tparam[opt] boolean enabled_modules.mode_selector Active state of the mode selector.
-- @tparam[opt] boolean enabled_modules.ir_receiver Active state of the IR receiver.
-- @tparam[opt] boolean enabled_modules.battery Active state of the battery monitor/sensor.
-- @tparam[opt] boolean enabled_modules.proximity Active state of the proximity sensors.
-- @tparam[opt] boolean enabled_modules.ring_led_1 Active state of ring LED 1.
-- @tparam[opt] boolean enabled_modules.ring_led_3 Active state of ring LED 3.
-- @tparam[opt] boolean enabled_modules.ring_led_5 Active state of ring LED 5.
-- @tparam[opt] boolean enabled_modules.ring_led_7 Active state of ring LED 7.
-- @tparam[opt] boolean enabled_modules.body_led Active state of the body LED.
-- @tparam[opt] boolean enabled_modules.front_led Active state of the front LED.
-- @tparam[opt] boolean enabled_modules.tof Active state of the Time-of-Flight (ToF) distance sensor.
-- @tparam[opt] boolean enabled_modules.imu Active state of the Inertial Measurement Unit (IMU).
-- @tparam[opt] boolean enabled_modules.camera Active state of the camera.
-- @tparam[opt] boolean enabled_modules.motors Active state of the drive motors.
-- @tparam[opt] boolean enabled_modules.ground Active state of the ground sensors.
--
-- @usage
-- robot.set_modules({
--     mode_selector = false,
--     ir_receiver   = false,
--     battery       = false,
--     proximity     = false,
--     ring_led_1    = false,
--     ring_led_3    = false,
--     ring_led_5    = false,
--     ring_led_7    = false,
--     body_led      = false,
--     front_led     = false,
--     tof           = true,
--     imu           = false,
--     camera        = false,
--     motors        = false,
--     ground        = true,
-- })
function robot.set_modules(options) end

--- Registers a function to run `func` whenever `trigger` triggers
--
-- A trigger never runs at the same time as the update function runs since Lua is synchronous. 
-- All triggers that were triggered are executed before the update loop.
--
-- @tparam string trigger 
-- @tparam function func
-- 
-- @usage
-- robot.on("telemetry:tof", function(distance) 
--     if distance < 50 then print("Caution !!") end
-- end)

function robot.on(trigger, func) end

--- Set target linear wheel velocities directly.
-- @tparam number left Linear velocity of left wheel (m/s).
-- @tparam number right Linear velocity of right wheel (m/s).
function robot.wheel_speed(left, right) end

--- Execute a planned motion profile (distance, arc, acceleration).
-- @tparam number distance Target travel distance along the arc (meters).
-- @tparam table options Table of trajectory parameters:
-- @tparam number options.speed: Max linear speed (m/s) [default: max_speed]
-- @tparam number options.radius: Turn radius in meters (positive = left turn, negative = right turn, 0 = spin in place, inf = straight line) [default: inf]
-- @tparam number options.accel: Acceleration rate (m/s^2) [default: max_accel]
-- @tparam number options.decel: Deceleration rate (m/s^2) [default: accel]
-- @tparam number options.notify: Weather to notify the user when the motion is finished [default: false]
function robot.move_trajectory(distance, options) end