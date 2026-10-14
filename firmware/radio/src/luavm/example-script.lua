local tostr_mt = {
    __tostring = function(self)
        local str = "{\n"
        for k, v in pairs(self) do
            str = str .. "\t" .. tostring(k) .. " = " .. tostring(v) .. "\n"
        end
        return str .. "}"
    end
}

local enabled_modules = setmetatable({
    mode_selector = false,
    ir_receiver = false,
    battery = false,
    proximity = false,
    ring_led_1 = false,
    ring_led_3 = false,
    ring_led_5 = false,
    ring_led_7 = false,
    body_led = true,
    front_led = false,
    tof = true, -- ENABLED
    imu = false,
    camera = false,
    motors = true, -- ENABLED
    ground = true -- ENABLED
}, tostr_mt)

-- Obstacle Avoidance Settings
local SAFE_DISTANCE = 100 -- Distance in mm to trigger backing up
local BACKWARD_MAX = -0.15 -- Max backward speed (m/s)
local KP_TOF = 0.003 -- Proportional gain for backing up

-- Line Follower Settings
local FORWARD_SPEED = 0.08 -- Base cruising speed (m/s)
local KP_GROUND = 0.0005 -- Steering sensitivity (tune this if it wobbles too much!)
local GROUND_DEADZONE = 20 -- Ignore tiny variances in the floor surface

-- State variable to share data between sensors
local current_tof_distance = 400

function init()
    print("Starting Line Follower with Obstacle Repulsion...")
    robot.set_modules(enabled_modules)

    -- 1. ToF Sensor: Just updates the state variable
    robot.on("telemetry:tof", function(distance)
        current_tof_distance = distance
    end)

    -- 2. Ground Sensor: The main control loop
    robot.on("telemetry:ground", function(ground)

        -- PRIORITY 1: Obstacle Repulsion
        if current_tof_distance < SAFE_DISTANCE then
            -- Turn on the LED to indicate we are blocked
            enabled_modules.body_led = true
            robot.set_modules(enabled_modules)

            -- Calculate how fast to back up based on how close the obstacle is
            local error_mm = current_tof_distance - SAFE_DISTANCE
            local speed = error_mm * KP_TOF

            -- Clamp to maximum backward speed
            if speed < BACKWARD_MAX then
                speed = BACKWARD_MAX
            end

            -- Drive straight backward (override line following)
            robot.set_wheels(speed, speed)
            return
        end

        -- PRIORITY 2: Line Following
        -- Turn off the LED because the path is clear
        enabled_modules.body_led = false
        robot.set_modules(enabled_modules)

        local difference = ground[1] - ground[3]
        local steering = 0

        -- Apply deadzone
        if math.abs(difference) > GROUND_DEADZONE then
            -- difference < 0 means line is on the left -> steering becomes negative
            -- difference > 0 means line is on the right -> steering becomes positive
            steering = difference * KP_GROUND
        end

        -- Calculate individual wheel speeds
        -- A negative steering value slows down the left wheel and speeds up the right, turning left.
        local left_speed = FORWARD_SPEED + steering
        local right_speed = FORWARD_SPEED - steering

        -- Apply the differential drive
        robot.set_wheels(left_speed, right_speed)
    end)
end

local counter
function update(dt)
    counter = counter + dt
    if counter > 5 then
        print(counter .. "s passed...")
        counter = 0
    end
end
