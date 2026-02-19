# Python API for the fripuck2 firmware
> Note: not implemented yet...

This api enables **remote controlling** of the e-puck2 with the fripuck firmware.
It offers two different APIs to collect data from the robot.
The first API is **state based**. One can synchronously query for the latest bit of information collected by the python program.
It is the simplest method.

Here is what it should look like: 
```py 
from fripuck2 import epuck

# initialization...

while True:
    distance = epuck.get_distance(0) # returns the sampled distance from distance sensor n°0.
    print(f'distance: {distance}')
    # ...
```

The second api is an asynchronous **stream based** api. 
It uses python's asynchronous capabilities to collect the data asynchronously.

Here is what it should look lik: 
```py 
import asyncio
from fripuck2 import epuck

async def cool_func():
    # react to every single packet being sent
    async for distance in epuck.distance_stream():
        print(f'distance: {distance}')
        
asyncio.run(cool_func())
```

The first API is technically simpler to use for students, but the problem is, that once the student's code gets complex and slow (if they use a ML model for example), data is lost and some calculations can loose accuracy, meaning we need a more robust approach, hence the need for the second API. 
It will however be a lot more complex to use, so maybe not the best for the students.
