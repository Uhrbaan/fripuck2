The goal of the Lua VM is: 
1. Easy configuration for the robot (does not need to recompile the robot to save changes)
2. Run small scripts on-device or alongside remote instructions for lower latency or not needing wifi
3. Be able to use a REPL to test robot values in real-time.

Ideally, I would also have a copy (or _very_ similar) api to run on the laptop. However, I don't know how useful that would be (except for comparing the latency).

## Scripting
For scripting, I imagine a Löve-2D/Arduino style approach, combined with hooks for more real time applications: 
```lua
--- Runs during initialization phase of the robot.
function init()
    robot.camera.set_resolution(robot.camera.VGA)

    -- Specify hooks to get new data in real-time
    robot.on("telemetry:battery", function(battery)
        if battery < 10 then 
            robot.camera.disable()
        end
    end)
end

--- Runs periodically.
---@param dt number Time passed since last update
function update(dt)
    -- Internal values are update whenever the update function gets called
    if robot.battery < 10 then
        robot.camera.disable()
    end
end
```

## Available hooks
Most FB schema types should be available, where the `telemetry:` prefix is for the the data types in the `sensors.fbs` and `command:` for the data types in the `commands.fbs` schema.

On top of that, the user can extend the communication protocol with the VM using the `Custom` data type. 
This data type features a `.topic` field. A Lua script can hook into `custom:topic` where topic is the custom topic defined in the `Custom.topic` field. This gives access to either a custom object with `.payload` and `.message` fields, or one argument for each if two arguments are provided in the function.
The `custom:` target is only available through hooks.

Similarily, the user can also hook into the repl. In that case, the default repl functionality is diabled, and instead the user direclty communicates with the scripts function until it "un-hooks" from the `repl` target.

For this, a json module is also provided (lua-cjson).

## REPL
If students have access to a REPL over the wifi on their laptop, it can simplify the exploration of the capabilities of the robot.
However, it poses some problems when managing the order of events.
The idea would be to have a queue of instructions, and before running the next `update()` loop, these instructions are executed.

## Limitations
Since lua won't have great performance (except for latency), I don't think it makes much sense to make the entire sensor history that gets sent over SPI to the laptop. 
