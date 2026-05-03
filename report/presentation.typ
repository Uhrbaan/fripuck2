#import "@preview/typslides:1.3.3": *

#show: typslides.with(
  font: "Noto Sans",
  link-style: "color",
  show-progress: true,
)

#show figure.caption: it => v(1em) + text(size: .8em)[_#it _]
#show raw.where(block: true): set text(size: .8em)
#set table(columns: 2, align: left, inset: 5pt)

#front-slide(
  title: "Fripuck",
  subtitle: [Improving the e-puck2 robot as an educational tool.],
  authors: [Léonard Clément],
  info: [#link("https://github.com/Uhrbaan/fripuck2")],
)

#table-of-contents()

#title-slide[
  _What_ is the e-puck2 ?
]

#slide[
  #cols(columns: (1fr, 1fr))[
    // _Suggestion: "Mobile educational robot designed by EPFL and developed by GCtronic"_
    - Mobile educational robot designed by #link("https://www.epfl.ch/labs/mobots/robots-technologies/e-puck2/")[EPFL] and developed by #link("https://www.gctronic.com/")[GCtronic]
    - Used by students in the robotics course.
  ][
    #figure(image("assets/epuck-pic.png"), caption: [The e-puck2 robot.])
  ]
]


#slide[#cols[
  #figure(image("assets/image-2.png"), caption: [The e-puck2 and its numerous sensors.])
][
  - The e-puck2 provides many sensors for students to explore:
    - IMU
    - ToF
    - Proximity (8x)
    - Camera
    - etc...
]]

#title-slide[
  _Why_ the need to update ?
]

#slide[
  #columns(2)[
    = Real-time only
    - Only latest data sample is known.
    - Data resolution severely limited.
    - Unable to query historical data.
    - Makes data analysis difficult.

    = Networking
    - Does not manage connection loss.
    - Unable to handle multiple robots.
    - Makes swarm behavior complex.

    = Blocking API
    - Sending/receiving data blocks execution.
    - Heavy computation (like image recognition) blocks execution.
    - Reduces responsiveness.

    = Tele-robotics
    - Cannot modify robot without re-compiling firmware.
    - Limits applications requiring low latency.
  ]
]

#slide(title: [Real-time], cols[
  - Neither the API nor the protocol allows sending or receiving multiple data points at once.
  - Limits the data resolution to the speed of the connection.
  - Currently, the only solution is to slow down the robot's movement.
][
  #figure(
    ```py
    # only receive latest data sample
    robot.get_tof()
    ```,
    caption: [Currently, both the API and the protocol only allow querying for the latest data sample.],
  )
])

#slide(title: [Networking], cols[
  - If the robot disconnects, the Python script simply fails.
  - The API only allows for one robot per script.
  - Projects requiring multiple robots need external solutions.
])

#slide(title: [Blocking API], cols[
  - The receive/send operation (```py robot.go_on()```) block execution
  - Similarly, resource-intensive operations like image detection do not operate in the background.
  - This greatly impacts the responsiveness of the robots, and often requires that the students make their robots run slower.
])

#slide(title: [Tele-robotics], cols[
  - It is currently impossible to modify the behavior of the robot directly without recompiling the firmware
  - This makes time-critical tasks impossible because of network latency
    - E.g. Determining which direction a sound came from
  - Also means that currently, robot-to-robot (P2P) communication is not possible.
])

#title-slide[_How_ can it be improved ?]

#slide(title: [Enable Data analysis], cols[
  - Modify protocol and API to allow querying for historical data
    - Store unqueried data in a buffer in the background
    - Allow both querying latest data or historical data
    - Data resolution is no longer bottlenecked by code execution speed
][
  #figure(
    [
      _status quo_:
      ```py
      # returns only latest reading
      robot.get_tof()
      ```
      _suggested improvement:_
      ```py
      # returns latest reading
      robot.tof.get()
      # returns multiple readings stored asynchronously in the background
      robot.tof.get_all()
      ```],
    caption: [Illustrating how scripting would differ if allowing querying of multiple data points.],
  )
])

#slide(title: [Networking and Blocking API], cols[
  #figure(
    [
      _status quo_:
      ```py
      # register a single robot
      robot = epuckapi.wrapper.get_robot(ip_addr)
      # block execution to send/receive data
      robot.go_on()
      ```
      _suggested improvement:_
      ```py
      cm = ConnectionManager()
      robot = Robot("192.168.2.214", history_size=100)
      # register one or multiple robots
      cm.register_robot(robot)
      # data is gathered asynchronously in another thread
      ```
    ],
    caption: [Illustrating how scripting would differ when implementing an asynchronous connection manager.],
  )
][
  - Splitting connection management into a manager class.
    - Enables multiple robots to connect to a single manager.
    - Can automatically reconnect the robot if the connection is lost.
    - Can collect and send data asynchronously in the background.
])

#slide(title: [Beyond tele-robotics], cols(columns: (2fr, 1fr))[
  - Introducing a lightweight Lua engine running on-board.
  - Allow uploading lua scripts through the API.
  - Custom code can then run directly on-chip:
    - Custom protocol
    - Low latency code
    - Exploring limitations of hardware...
][
  #figure(image("assets/powered-by-lua.gif"), caption: [Logo of the Lua programming language])
])

#slide(title: [And more...])[
  - Improving build system (chaotic CMake vs PlatformIO)
  - Modernizing codebase (standard FreeRTOS vs ChibiOS)
  - Increasing data throughput
  - Enabling more precise control over the robot's movement
  - Personal interest in exploring embedded systems development, a topic not fully covered in the university curriculum.
]

#slide(title: [Proposed Architecture])[
  #figure(image("assets/Fripuck Architecture full.svg"), caption: [Full architecture of the Fripuck project])
]

#slide(title: [Proposed Architecture: API])[
  #figure(
    image("assets/Fripuck Architechture: API.svg"),
    caption: [Focus on the API architecture of the Fripuck project],
  )
]

#slide(title: [Proposed Architecture: Firmware])[
  #figure(
    image("assets/Fripuck Architecture: Frimware.svg"),
    caption: [Focus on the Firmware architecture of the Fripuck project],
  )
]

#title-slide[_When_ are the features implemented ?]

#slide(cols(columns: (1fr, 2fr))[
  - The bachelor thesis is split over 4 semesters.
  - Last semester was spent testing out if the project was feasible.
][
  #set text(size: .8em)
  #figure(table(
    table.header(
      [*_When_*],
      [*_What_*],
    ),
    [SA 2025],
    [
      - Exploring legacy code
      - Testing feasibility of on-board Lua engine
    ],

    [SP 2026],
    [
      - Moving the project to a modern framework
      - Updating API and protocol
    ],

    [Summer 2026],
    [
      - Finishing transition to the modern framework
      - Implementing the Lua engine
      - Publishing beta version of API
    ],

    [SA 2026],
    [
      - Testing and polishing of API
      - Testing and polishing of firmware
        - Inquire whether GCtronic are interested in maintaining the project
    ],

    [SP 2026],
    [
      - (Maybe) test and fix the new technology stack with students
      - Publish the Bachelor thesis
    ],
  ))
])

#slide(title: [Tracking progress], cols[
  #figure(image("assets/image-3.png"))
][
  Progress can be tracked at https://github.com/Uhrbaan/fripuck2.
])

#focus-slide[Questions ?]
#focus-slide[
  Thank you for your attention !
]
