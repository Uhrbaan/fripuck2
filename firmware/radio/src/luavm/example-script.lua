function init()
    print("Hello world, from Lua ! 🌛")
end

local counter = 0.0
local played = false
function update(dt)
    counter = counter + dt

    if counter > 10 and played == false then
        print("Playing Megalovania")
        robot.play("T120 O4 L16 Q5 I2 " .. -- settings
        "d d > d8 < a8. r16 g#8 g8 f8 d16 f16 g16 " .. -- intro 1
        "c c > d8 < a8. r16 g#8 g8 f8 d16 f16 g16 " .. -- intro 2
        "< b b > d8 < a8. r16 g#8 g8 f8 d16 f16 g16 " .. -- intro 3
        "< a# a# > d8 < a8. r16 g#8 g8 f8 d16 f16 g16")
        counter = 0
        played = true
    end
end
