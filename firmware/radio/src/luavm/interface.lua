-- Simple file to keep track of what we need to initialize the VM with.


----------------- Global functions -----------------

--- Runs one at startup. Use to configure robot and set hooks.
function init()
end

--- Runs every [] ms and executes the code block. Can be used for simple control loops.
---@param dt number The number of milliseconds elapsed since the last call.
function update(dt)
end

--------------------- Robot specifics ---------------------

---@type table
local robot = {

    --- Function to register a hook on the robot. 
    ---@type fun(hook: string, fn: function)
    ---@param hook string Name of the data piece the function can be called on
    ---@param fn fun(value) | nil Register a hook. `fn` will be called when data is read with the parameter `value`, which is the value of the data the hook is linked to. If nil, it unregisters.
    ---@return boolean True if worked, False if it failed.
    on = function(hook, fn)
    end

    ---@type number Value of the time of flight sensor
    tof = 100
}