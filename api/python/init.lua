function init()
    print("Hello, world ! This file prints the current selector value when it changes.")
    robot.on("telemetry:selector", function(value)
        print("Selector: " .. value)
    end)
end

local counter = 0.0
function update(dt)
    counter = counter + dt
    if counter > 1 then
        print("Alive...")
        counter = 0
    end
end
