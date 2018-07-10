# useful-robot-links
List of links I've found useful while exploring robotics and ROS

- [ROS](#ros)
  - [Embedded Systems](#embedded-systems)
  - [ROS1 Communications](#ros1-communications)
  - [Actions](#actions)
  - [Autonomous Vehicles](#autonomous-vehicles)
  - [Enhancements](#enhancements)
- [ROS2](#ros2)
  - [Starting Out](#starting-out)
  - [Raspberry Pi](#raspberry-pi)
  - [Communications](#communications)
- [Simulation](#simulation)
  - [Simulation Engines](#simulation-engines)
  - [Physics Engines](#physics-engines)
- [Hardware](#hardware)
  - [Controllers](#controllers)
- [General Robotics Topics](#general-robotics-topics)
  - [SLAM](#slam)
  - [Control Systems](#control-systems)
- [Real Time Systems](#real-time-systems)
  - [Communications and Networking](#communications-and-networking)
- [Reinforcement Learning](#reinforcement-learning)
- [Stores](#stores)
- [Maker](#maker)

# ROS

## Embedded Systems
* [OpenEmbedded Layer for ROS Applications](https://github.com/bmwcarit/meta-ros) - Layer for Indigo Igloo to run under OpenEmbedded

## ROS1 Communications
* [MQTT Bridge](http://wiki.ros.org/mqtt_bridge) - Bridge between ROS1 and MQTT protocol
* [RosBridge](http://wiki.ros.org/rosbridge_suite) - JSON interface to ROS to allow interactions with the ROS graph from non-ROS applications via WebSockets and TCP.

## Actions
* [ActionLib](http://wiki.ros.org/actionlib) - Standardized interface for creating preemptable tasks

## Autonomous Vehicles
* [MOOS-IvP](http://oceanai.mit.edu/moos-ivp/pmwiki/pmwiki.php?n=Main.HomePage) - Set of open source C++ modules for providing autonomy on robotic platforms, in particular autonomous marine vehicles
* [PX4 Autopilot User Guide](https://docs.px4.io/en/) - PX4 is the Professional Autopilot, an autopiloting system for all kinds of vehicles from racing to cargo drones through ground vehicles to submersible vehicles.
* [Light Autonomous Underwater Vehicle (LAUV) model for Gazebo](https://github.com/uuvsimulator/lauv_gazebo) - Robot description and necessary launch files to simulate the Light Autonomous Underwater Vehicle (LAUV), developed by the Laboratório de Sistemas e Tecnologia Subaquática (LSTS) from Porto University and OceanScan-MST.

## Enhancements
* [ROS Enhancement Proposals (REPs)](http://www.ros.org/reps/rep-0000.html) - All public REPs for ROS since 2010

# ROS2

## Starting Out
* [ROS2 Onboarding Guide](https://github.com/ros2/ros2/wiki/ROS-2-On-boarding-Guide) - On-boarding guide for contributing to ROS 2.0
* [ROS2 Project Board](https://waffle.io/ros2/ros2) - Issues board for ROS 2.0 development
* [ROS2 Package List](https://github.com/ros2/ros2/blob/master/ros2.repos) - Current list of all packages included in ROS 2.0
* [Core Stack Developer Overview](http://docs.ros2.org/ardent/developer_overview.html) - Overview of the main internal core componets of ROS 2.0

## Raspberry Pi
* [ros2_raspbian_tools](https://github.com/esteve/ros2_raspbian_tools) - Tools for crosscompiling ROS2 for the Raspberry Pi

## Communications
* [ZeroMQ/ROS](https://design.ros2.org/articles/ros_with_zeromq.html) - Design article on an attempt to integrate ZeroMQ into ROS 2.0

# Simulation

## Simulation Engines
* [Gazebo](http://gazebosim.org/) - Robotics simulation environment for ROS

## Physics Engines
* [MuJoCo](http://www.mujoco.org/) - Multi-Joint dynamics with Contact, commercial license physics engine, built specifically to facilitate research and development in robotics.
* [Open Dynamics Engine (ODE)](http://www.ode.org/) - An open source, high performance library for simulation rigid body dynamics
* [Dynamic Animation and Robotics Toolkit (DART)](https://dartsim.github.io/) - A collaborative, cross-platform, open source library providing data structures and algorithms for kinematic and dynamic applications in robotics and computer animation
* [Bullet Physics Engine](https://github.com/bulletphysics/bullet3) - Real-time collision detection and multi-physics simulation

# Hardware

## Controllers
* [OpenCR](http://support.robotis.com/en/product/controller/opencr.htm) - Open Controller for ROS
* [ST](https://www.st.com/content/st_com/en.html) - Maker of semiconductors, lots of products related to many industries
  * [Power Management](https://www.st.com/en/power-management.html)
  * [Motor Drivers](https://www.st.com/en/motor-drivers.html)
  * [Microcontrollers](https://www.st.com/en/microcontrollers.html)

# General Robotics Topics

## Libraries
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) - C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms
* [Orocos BFL](http://www.orocos.org/bfl) - Bayesian Filtering Libaray from Orocos

## Kinematics
* [Orocos KDL](http://www.orocos.org/kdl) - Kinematics and dynamics library from Orocos

## SLAM
* [Simultaneous Location and Mapping (SLAM)](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) - Wikipedia entry on SLAM, an important problem in robotics

## Control Systems
* [PID Controller](https://en.wikipedia.org/wiki/PID_controller) - Wikipedia entry on Proportional-Integral-Derivative controller, a control loop feedback mechanism used in mechanical and electrical control systems


# Real Time Systems

## Communications and Networking
* [Non-Return-to-Zero](https://en.wikipedia.org/wiki/Non-return-to-zero) - A binary encoding technique used in telecommunications to transmit data
* [Controller Area Network (CAN bus)](https://en.wikipedia.org/wiki/CAN_bus) - Message-based protocol forming a bus to enable devices to communicate without a host computer
  * [CANOpen](https://www.can-cia.org/canopen/) - CAN-based communication system
    * [SDO/Service Data Object](https://www.can-cia.org/can-knowledge/canopen/sdo-protocol/)
    * [CAN-FD/Flexible Data Rate](https://www.can-cia.org/can-knowledge/can/can-fd/)
    * [PMS/Physical Media-dependent Sublayers](https://en.wikipedia.org/wiki/Physical_Medium_Dependent)
  * [UAVCAN](http://uavcan.org/) - Lightweight protocol for reliable communication in aerospace and robotic applications
* [Orocos/RTT](http://www.orocos.org/rtt) - Real Time Toolkit, a library for RTOS
* [Pragmatic General Multicast (PGM)](https://en.wikipedia.org/wiki/Pragmatic_General_Multicast)
* [OPC Unified Architecture](https://en.wikipedia.org/wiki/OPC_Unified_Architecture) - Wikipedia entry on machine-to-machine communication protocol for industrial automation

# Reinforcement Learning
* [Arcade Learning Environment](https://github.com/mgbellemare/Arcade-Learning-Environment) - Atari 2600 game emulator, together with a framework for developing AI agents
* [OpenAI Gym](https://github.com/openai/gym) - Toolkit for developing and comparing reinforcement learning algorithms
  * [Ingredients for Robotics Research](https://blog.openai.com/ingredients-for-robotics-research/) - Gym environments specific to robotics

# Stores
* [Karlsson Robotics](https://www.kr4.us/)
* [Online Metals](http://www.onlinemetals.com/) - Pre-cut and custom cut metals of many different types, shapes, and sizes
* [DFRobot](https://www.dfrobot.com)
* [Arrow](https://www.arrow.com)

# Maker

## Electonics
* [Maker Pro](https://maker.pro/) - Lots of articles on building things using Arduino and Raspberry Pi
* [All About Circuits](https://www.allaboutcircuits.com/) - All about electrical circuits
* [Fundamentals of Electronics Tutorial](http://developer.wildernesslabs.co/Hardware/Tutorials/Electronics/) - Practical, hands-on tutorial on electronic circuits that focuses on modern circuit development for connected things
