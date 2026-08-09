local speed = 0

function init()
    print("Robot initialized!")
    speed = 100

    robot.on("telemetry:tof", function(distance)
        print("Nearest object: " .. distance .. "mm")
    end)

end

local counter = 0
local total_dt = 0

function update(dt)
    counter = counter + 1
    total_dt = total_dt + dt
    if counter > 100  then 
        local average = total_dt / counter 
        print("Over 100 iterations, the average dt is: " .. dt .. "s.")
        counter = 0
        total_dt = 0
    end
end