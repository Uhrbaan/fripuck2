local timer = 0
local current_led = 1

function init()
    print("Hello world, from Lua ! 🌛")

    robot.set_led("all", false)
    robot.set_led(current_led, {255, 0, 0})
end

function update(dt)
    timer = timer + dt

    if timer >= 0.5 then
        timer = timer - 0.5

        local next_led = (current_led % 8) + 1

        robot.set_leds({
            [current_led] = false,
            [next_led] = {255, 0, 0}
        })

        current_led = next_led
    end
end
