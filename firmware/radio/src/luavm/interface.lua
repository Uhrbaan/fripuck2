--- @module robot
--- Function to register a hook on the robot. 
---@type fun(hook: string, fn: function)
---@param hook string Name of the data piece the function can be called on
---@param fn fun(value) | nil Register a hook. `fn` will be called when data is read with the parameter `value`, which is the value of the data the hook is linked to. If nil, it unregisters.
---@return boolean True if worked, False if it failed.
on = function(hook, fn)
end

---@type number Value of the time of flight sensor
tof = 100

--- Configures the global default acceleration and deceleration for all movements.
-- @tparam number accl Default linear acceleration rate in m/s^2.
-- @tparam number decel Default linear deceleration rate in m/s^2.
movement_settings = function(accl, decel)
end

--- Starts continuous movement at a specific speed.
-- @tparam number speed Target cruising velocity in m/s.
-- @tparam[opt] table options Configuration table.
-- @tparam[opt=INFINITY] number options.distance Distance to travel before stopping.
-- @tparam[opt=INFINITY] number options.radius Turn radius in meters.
-- @tparam[opt] number options.accel Override default acceleration.
-- @tparam[opt] number options.decel Override default deceleration.
-- @tparam[opt=false] boolean options.synchronous Block execution until motion completes.
-- @tparam[opt=false] boolean options.notify Trigger a notification when finished.
drive = function(speed, options)
end

--- Moves the robot in a straight line for a specified distance.
-- @tparam number distance Distance to travel in meters.
-- @tparam number speed Target cruising velocity in m/s.
-- @tparam[opt] table options Configuration table (accel, decel, synchronous, notify).
move = function(distance, speed, options)
end

--- Turns the robot in-place by a specific angle.
-- @tparam number radians Angle to turn (positive for left, negative for right).
-- @tparam number speed Turn speed in m/s (linear speed of the wheels).
-- @tparam[opt] table options Configuration table (accel, decel, synchronous, notify).
turn = function(radians, speed, options)
end

--- Moves the robot along an arc.
-- @tparam number distance Length of the arc to travel in meters.
-- @tparam number radius Turn radius in meters (positive for left, negative for right).
-- @tparam[opt] table options Configuration table.
-- @tparam[opt=0.1] number options.speed Movement speed in m/s (required if not set).
-- @tparam[opt] number options.accel Override default acceleration.
-- @tparam[opt] number options.decel Override default deceleration.
-- @tparam[opt=false] boolean options.synchronous Block execution until motion completes.
-- @tparam[opt=false] boolean options.notify Trigger a notification when finished.
arc = function(distance, radius, options)
end

--- Drives the robot by setting individual wheel velocities directly.
-- Translates wheel speeds into a continuous trajectory command.
-- @tparam number left Target velocity of the left wheel in m/s.
-- @tparam number right Target velocity of the right wheel in m/s.
-- @tparam[opt] table options Configuration table (distance, accel, decel, synchronous, notify).
set_wheels = function(left, right, options)
end
