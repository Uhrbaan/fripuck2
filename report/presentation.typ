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
  subtitle: [Improving the e-puck2 robot as an educational tool:\ _What_, _Why_, _How_ and _When_.],
  authors: [Léonard Clément],
  info: [#link("https://github.com/Uhrbaan/fripuck2")],
)

// #table-of-contents()

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
  There are _*2*_ reasons why I feel the need to improve the e-puck2 robot:

  + During the robotics cours, a _*project rendered impossible*_ due to the low data resolution.

  + Personal interest in *_exploring embedded systems_* development, a topic not fully covered in the university curriculum.

  Improvements on 4 aspects: enabeling easy _data analysis_, improved _networking_, _asynchronous API_ and enabeling _on-board robotics_.
]

// #slide[
//   #columns(2)[
//     = Real-time only
//     - Only latest data sample is known.
//     - Data resolution severely limited.
//     - Unable to query historical data.
//     - Makes data analysis difficult.

//     = Networking
//     - Does not manage connection loss.
//     - Unable to handle multiple robots.
//     - Makes swarm behavior complex.

//     = Blocking API
//     - Sending/receiving data blocks execution.
//     - Heavy computation (like image recognition) blocks execution.
//     - Reduces responsiveness.

//     = Tele-robotics
//     - Cannot modify robot without re-compiling firmware.
//     - Limits applications requiring low latency.
//   ]
// ]

#title-slide[_How_ can it be improved ?]

#slide(title: [Enabeling easy data analysis], cols[
  Currently, the e-puck2 API and firmware focus on *_real-time robotics_*, making projects using heavy data analysis hard to achieve.

  - Storing historical data in the background#footnote[Already implemented and operational.]<ftn-implemented-operational>
  - Allows querying multiple data points @ftn-implemented-operational
  - Data resolution not bottlenecked by code execution speed but hardware.#footnote[Which has been measured of being capable of up to 1.18Mb/s]
  - This allows for new projects that require data analysis.
    - E.g. Determining the position of the robot based on the IMU sensor.
])
// ][
//   #figure(
//     [
//       _status quo_:
//       ```py
//       # returns only latest reading
//       robot.get_tof()
//       ```
//       _suggested improvement:_
//       ```py
//       # returns latest reading
//       robot.tof.get()
//       # returns multiple readings stored asynchronously in the background
//       robot.tof.get_all()
//       ```],
//     caption: [Illustrating how scripting would differ if allowing querying of multiple data points.],
//   )
// ]

#slide(title: [Improving Networking], cols[
  Right now, a major painpoint for robotics students is an *_unstable connection_* to the robots.

  - Adding automatic reconnection on connection loss.
  - Enabeling multiple robots to be connected at once to simplify swarm robotics.
  - Managing sending and receiving data in the background.

  #footnote[Already implemented and operational]
])

#slide(title: [Asynchronous API], cols[
  Currently, the API _*blocks code execution*_ (synchronous) on steps like sending/receiving data and heavy operations like image recognition. This directly impacts data resolution.

  - Removing the ```py robot.go_on()```#footnote[A simple script spends 98% of execution time waiting on this single function call.] call and managing communication in a background thread.#footnote[Already implemented and operational.]
  - Similarly, make resource-intensive operations like image detection run in the background.
  - Students no longer would have to make their robots run slower to mitigate the latency.
])

#slide(title: [Beyond Tele-robotics], cols[
  Robots can _*only be remote-controlled*_. This makes low-latency tasks difficult and does not allow students to explore the limitations of on-board robotics.

  - Suggesting a lightweight on-board VM#footnote[Based on Lua, has been tested to be lightweight enough.] with scripts uploaded through the API, which mitigates the difficulty of modifying C source code for students.
  - Allows students to explore the differences between remote-controlled or on-board robotics.
  - This allows new projects like...
    - Determining where a sound comes from,
    - Creating your own custom protocol,
    - And more !
])

#slide(title: [Other improvements])[
  Right now, the *_technology stack is outdated_* and hard to maintain.
  - Modernizing the firmware enables future work by students.
  - Simplifying the build process and documentation makes the barrier of entry lower for small modifications, possibly allowing students to modify the firmware if needed.
  - Overall, a lot of work is also going into modernizing the codebase for possible future modifications, beyond what I currently have planned.

]

// #slide(title: [Enable Data analysis], cols[
//   - Modify protocol and API to allow querying for historical data
//     - Store unqueried data in a buffer in the background
//     - Allow both querying latest data or historical data
//     - Data resolution is no longer bottlenecked by code execution speed
// ])

// #slide(title: [Networking and Blocking API], cols[
//   #figure(
//     [
//       _status quo_:
//       ```py
//       # register a single robot
//       robot = epuckapi.wrapper.get_robot(ip_addr)
//       # block execution to send/receive data
//       robot.go_on()
//       ```
//       _suggested improvement:_
//       ```py
//       cm = ConnectionManager()
//       robot = Robot("192.168.2.214", history_size=100)
//       # register one or multiple robots
//       cm.register_robot(robot)
//       # data is gathered asynchronously in another thread
//       ```
//     ],
//     caption: [Illustrating how scripting would differ when implementing an asynchronous connection manager.],
//   )
// ][
//   - Splitting connection management into a manager class.
//     - Enables multiple robots to connect to a single manager.
//     - Can automatically reconnect the robot if the connection is lost.
//     - Can collect and send data asynchronously in the background.
// ])

// #slide(title: [Beyond tele-robotics], cols(columns: (2fr, 1fr))[
//   - Introducing a lightweight Lua engine running on-board.
//   - Allow uploading lua scripts through the API.
//   - Custom code can then run directly on-chip:
//     - Custom protocol
//     - Low latency code
//     - Exploring limitations of hardware...
// ][
//   #figure(image("assets/powered-by-lua.gif"), caption: [Logo of the Lua programming language])
// ])

// #slide(title: [And more...])[
//   - Improving build system (chaotic CMake vs PlatformIO)
//   - Modernizing codebase (standard FreeRTOS vs ChibiOS)
//   - Increasing data throughput
//   - Enabling more precise control over the robot's movement
//   -

//   C'est un projet conséquent. Montrer les résultats.
// ]

#slide(title: [Proposed Architecture])[


  #figure(
    image("assets/Fripuck Architecture full.svg"),
    caption: [Full architecture of the Fripuck project. It illustrates the complexity of the project.],
  )
]

// #slide(title: [Proposed Architecture: API])[
//   #figure(
//     image("assets/Fripuck Architechture: API.svg"),
//     caption: [Focus on the API architecture of the Fripuck project],
//   )
// ]

// #slide(title: [Proposed Architecture: Firmware])[
//   #figure(
//     image("assets/Fripuck Architecture: Frimware.svg"),
//     caption: [Focus on the Firmware architecture of the Fripuck project],
//   )
// ]

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

    [Summer\ 2026],
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
  The first priority is reaching the featureset of the _status quo_. Beyond qualitative improvements, we can measure
  - Data throughput
  - Student's ease-of-use

  Progress can be tracked at https://github.com/Uhrbaan/fripuck2.
])

#focus-slide[Questions ?]
#focus-slide[
  Thank you for your attention !
]

Recommendations
