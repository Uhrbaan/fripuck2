local tostr_mt = {
    __tostring = function(self)
        local str = "{\n"
        for k, v in pairs(self) do
            str = str .. "\t" .. tostring(k) .. " = " .. tostring(v) .. "\n"
        end
        return str .. "}"
    end
}

-- Make sure motors and ToF are enabled!
local enabled_modules = setmetatable({
    mode_selector = false,
    ir_receiver = false,
    battery = false,
    proximity = false,
    ring_led_1 = false,
    ring_led_3 = false,
    ring_led_5 = false,
    ring_led_7 = false,
    body_led = true, -- Let's use this for visual feedback
    front_led = false,
    tof = true,
    imu = false,
    camera = false,
    motors = true, -- Enabled to allow driving!
    ground = false
}, tostr_mt)

-- Controller Settings
local TARGET_DISTANCE = 100 -- The distance (in mm) the robot wants to maintain
local MAX_SPEED = 0.3 -- Maximum speed in m/s
local KP = 0.004 -- Proportional gain (how aggressively it corrects the error)
local DEADZONE = 15 -- Ignore errors smaller than this to prevent nervous jittering

function init()
    print("Starting Jedi-Force Follow-Me mode (with Trajectory Profiling)...")
    robot.set_modules(enabled_modules)

    robot.on("telemetry:tof", function(distance)

        -- 1. If nothing is in front of the robot, anchor it with a standard stop
        if distance > 400 then
            robot.drive(0)
            enabled_modules.body_led = false
            robot.set_modules(enabled_modules)
            return
        end

        enabled_modules.body_led = true
        robot.set_modules(enabled_modules)

        -- 2. Calculate the error in mm, and convert it to meters for the API
        local error_mm = distance - TARGET_DISTANCE
        local error_m = math.abs(error_mm) / 1000.0 -- Absolute distance in meters

        -- 3. Deadzone check
        if math.abs(error_mm) < DEADZONE then
            -- By the time it reaches the deadzone, the FSM's deceleration 
            -- profile should have already brought the velocity down to ~0.
            robot.drive(0)
            return
        end

        -- 4. Calculate speed based on the error
        local speed = error_mm * KP

        -- 5. Clamp the speed
        if speed > MAX_SPEED then
            speed = MAX_SPEED
        elseif speed < -MAX_SPEED then
            speed = -MAX_SPEED
        end

        -- 6. Drive with the remaining distance!
        -- The C-level FSM will automatically trigger MOTION_DECEL when
        -- this provided distance approaches the dynamic `d_stop` braking threshold.
        robot.drive(speed)
    end)
end

local counter = 0.0
function update(dt)
    counter = counter + dt
    if counter > 5 then
        print(counter .. "s passed...")
        counter = 0
    end
end
