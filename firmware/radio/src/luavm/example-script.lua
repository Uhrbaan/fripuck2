local speed = 0
local last_side = "center"
local side = "center"

function init()
    print("Robot initialized!")
    speed = 100

    robot.on("telemetry:ground", function(ground)
        -- lua arrays start at 1
        local difference = ground[1] - ground[3]
        
        if math.abs(difference) > 20 then 
            if difference < 0 then side = "left"
            else side = "right" end 
        else side = "center" end

        if side ~= last_side then 
            print("Black line on the " .. side .. " side.") 
        end

        last_side = side
    end)

    robot.on("telemetry:tof", function(distance) 
        if distance < 50 then print("Caution !!") end
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