# useful-robot-links
List of links I've found useful while exploring robotics and ROS

- [ROS](#ros)
  - [Embedded Systems](#embedded-systems)
  - [ROS1 Communications](#ros1-communications)
  - [Actions](#actions)
  - [Autonomous Vehicles](#autonomous-vehicles)
  - [Enhancements](#enhancements)
  - [Robot Examples](#robot-examples)
- [ROS2](#ros2)
  - [Starting Out](#starting-out)
  - [Raspberry Pi](#raspberry-pi)
  - [Communications](#communications)
- [Simulation](#simulation)
  - [Simulation Engines](#simulation-engines)
  - [Physics Engines](#physics-engines)
- [Hardware](#hardware)
  - [Controllers](#controllers)
  - [Embedded Systems Software](#embedded-systems)
- [General Robotics Topics](#general-robotics-topics)
  - [Robotics Software](#robotics-software)
  - [Middleware](#middleware)
  - [Kinematics](#kinematics)
  - [Computer Vision](#computer-vision)
  - [SLAM](#slam)
  - [Control Systems](#control-systems)
- [Real Time Systems](#real-time-systems)
  - [Communications and Networking](#communications-and-networking)
  - [Operating Systems](#operating-systems)
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
* [Detect and AvoID Alerting Logic for Unmanned Systems](https://github.com/nasa/wellclear) - DAIDALUS is a reference implementation of a detect and avoid concept intended to support the integration of Unmanned Aircraft Systems into civil airspace
* [LibrePilot](https://www.librepilot.org/site/index.html) - A software suite to control multicopter and other RC-models

## Enhancements
* [ROS Enhancement Proposals (REPs)](http://www.ros.org/reps/rep-0000.html) - All public REPs for ROS since 2010

## Robot Examples
* [Astrobee](https://software.nasa.gov/software/ARC-17994-1) - A free-flying robot that is designed to operate as a payload inside the International Space Station (ISS), contains embedded (on-board) software and simulator

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
* [Software Bus Network - Core Flight System](https://software.nasa.gov/software/GSC-16917-1) - NASA Core Flight System plugin to provide a publish/subscribe mechanism for peer-to-peer communication

# Simulation

## Simulation Engines
* [Gazebo](http://gazebosim.org/) - Robotics simulation environment for ROS
* [Mission Simulation Toolkit](https://software.nasa.gov/software/ARC-14932-1) - A simulation framework to support research in autonomy for remote exploration
* [RoboDK](https://robodk.com/) - Offline simulation environment for industrial robotic arms

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
* [C2000 Piccolo 32-bit Microcontrollers - TI](http://www.ti.com/microcontrollers/c2000-real-time-control-mcus/overview.html) - Real-time 32-bit microcontrollers from Texas Instruments

## Embedded Systems Software
* [Yocto](https://www.yoctoproject.org/) - Open source collaboration project that helps developers create custom Linux-based systems for embedded products, regardless of the hardware architecture
* [OpenEmbedded](https://www.openembedded.org/wiki/Main_Page) - Build framework for embedded Linux.

# General Robotics Topics

## Middleware
* [Robot Application Programming Interface Delegate - RAPID](https://software.nasa.gov/software/ARC-16368-1A) - RAPID is a software reference implementation framework for remote operations, which promotes interoperability between robot software modules
* [MOOS](https://sites.google.com/site/moossoftware/) - Light, Fast, Cross Platform Middleware for Robots
* [RoboComp](https://robocomp.github.io/web/) - Open-source Robotics framework providing the tools to create and modify software components that communicate through public interfaces
* [OpenJAUS](http://openjaus.com/) - OpenJAUS is the leading source for JAUS-compliant middleware for unmanned systems
* [JAUS Toolset](http://jaustoolset.org/) - Open-source implementation of the Joint Architecture for Unmanned Systems (JAUS)
* [EEROS](http://eeros.org/wordpress/) - Easy, Elegant, Reliable, Open and Safe Real-Time Robotics Software Framework

## Robotics Software
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) - C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms
* [Orocos BFL](http://www.orocos.org/bfl) - Bayesian Filtering Library from Orocos
* [PolyCARP](https://software.nasa.gov/software/LAR-18798-1) - A package of algorithms, including both their formal models and software implementations, for computing containment, collision, resolution, and recovery information for polygons

## Computer Vision
* [OpenCV](https://opencv.org/) - Open Source Computer Vision Library
* [NASA Vision Workbench](https://github.com/nasa/visionworkbench) - The NASA Vision Workbench is a general purpose image processing and computer vision library
* [NASA Ames Stero Pipeline](https://github.com/nasa/StereoPipeline) - The NASA Ames Stereo Pipeline is a suite of automated geodesy & stereogrammetry tools designed for processing planetary imagery captured from orbiting and landed robotic explorers on other planets

## Kinematics
* [Orocos KDL](http://www.orocos.org/kdl) - Kinematics and dynamics library from Orocos
* [The Robotics Library](https://www.roboticslibrary.org/) - A self-contained C++ library for robot kinematics, motion planning and control. It covers mathematics, kinematics and dynamics, hardware abstraction, motion planning, collision detection, and visualization

## SLAM
* [Simultaneous Location and Mapping (SLAM)](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) - Wikipedia entry on SLAM, an important problem in robotics
* [Mobile Robot Programming Toolkit](https://www.mrpt.org/) - Portable and well-tested applications and libraries covering data structures and algorithms employed in common robotics research areas

## Control Systems
* [PID Controller](https://en.wikipedia.org/wiki/PID_controller) - Wikipedia entry on Proportional-Integral-Derivative controller, a control loop feedback mechanism used in mechanical and electrical control systems
* [Livingstone 2](https://software.nasa.gov/software/ARC-14725-1) - System for Automated Diagnosis and Discrete Control of Complex Systems

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
* [IEC 61158 / Fieldbus](https://en.wikipedia.org/wiki/Fieldbus) - Wikipedia entry on Fieldbus, a family of industrial computer network protocols used for real-time distributed control, standardized as IEC 61158.
* [EtherCAT](https://www.ethercat.org/en/) - A real-time Industrial Ethernet technology originally developed by Beckhoff Automation. The EtherCAT protocol which is disclosed in the IEC standard IEC61158 is suitable for hard and soft real-time requirements in automation technology, in test and measurement and many other applications.
  * [Open EtherCAT Society](https://openethercatsociety.github.io/)
  * [EtherCAT Introduction](https://www.ethercat.org/pdf/english/EtherCAT_Introduction_0905.pdf)
* [Modbus](https://en.wikipedia.org/wiki/Modbus) - A serial communications protocol which is a commonly available means of connecting industrial electronic devices.

## Operating Systems
* [QNX Neutrino](https://community.qnx.com/sf/sfmain/do/viewProject/projects.core_os) - QNX Neutrino is a realtime microkernel operating system. It's scalable, embeddable, networked, SMP-capable, memory protected, and has a POSIX interface. The OS supports several processor families, including x86, ARM, XScale, PPC, MIPS, and SH-4.
  * [Beaglebone Support](http://community.qnx.com/sf/wiki/do/viewPage/projects.bsp/wiki/TiAm335Beaglebone) - Board Support Package for AM335x Beaglebone
* [Amazon FreeRTOS](https://aws.amazon.com/freertos/) - Amazon FreeRTOS is an operating system for microcontrollers that makes small, low-power edge devices easy to program, deploy, secure, connect, and manage.
* [Linux PREEMPT_RT](https://wiki.linuxfoundation.org/realtime/start) - Patch to the linux kernel to add preemptive, realtime scheduling.



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

## Electronics
* [Maker Pro](https://maker.pro/) - Lots of articles on building things using Arduino and Raspberry Pi
* [All About Circuits](https://www.allaboutcircuits.com/) - All about electrical circuits
* [Fundamentals of Electronics Tutorial](http://developer.wildernesslabs.co/Hardware/Tutorials/Electronics/) - Practical, hands-on tutorial on electronic circuits that focuses on modern circuit development for connected things
