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
local MAX_SPEED = 0.5 -- Maximum speed in m/s
local KP = 0.004 -- Proportional gain (how aggressively it corrects the error)
local DEADZONE = 20 -- Ignore errors smaller than this to prevent nervous jittering

local measurements_since_last = 0

function init()
    print("Starting Jedi-Force Follow-Me mode (with Trajectory Profiling)...")
    robot.set_modules(enabled_modules)

    robot.on("telemetry:tof", function(distance)
        measurements_since_last = measurements_since_last + 1

        -- 1. If nothing is in front of the robot, anchor it with a standard stop
        if distance > 400 then
            -- robot.drive(0)
            enabled_modules.body_led = false
            robot.set_modules({
                body_led = false
            })
            return
        end

        robot.set_modules({
            body_led = true
        })

    end)
    robot.on("telemetry:tof", function(value)
        -- print("selector: " .. value)
    end)
end

local counter = 0.0
function update(dt)
    -- if robot.selector > 3 then
    --     robot.set_modules({
    --         body_led = true
    --     })
    -- else
    --     robot.set_modules({
    --         body_led = false
    --     })
    -- end
end
