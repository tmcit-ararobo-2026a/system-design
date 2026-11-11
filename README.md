# Robot System Design Specification

## Electrical & Communication Architecture

### Integrated Wiring Diagram
Comprehensive power routing, multi-bus CAN architecture, Ethernet networking, and emergency stop interlocks.

![Integrated Wiring Diagram](wireing/wireing%20diagram.png)

### Power Distribution Diagram
Power rails for 5S actuator drive, uninterruptible power supply (LPMB), DC-DC converters, and emergency power cutoff.

![Power Distribution Diagram](wireing/power%20wireing%20diagram.png)

### Signal & Communication Topology Diagram
Communication bus topology showing isolated CAN channels (CAN1/CAN2/CAN3), Ethernet switch connections, and digital emergency stop lines.

![Signal & Communication Topology Diagram](wireing/logic%20wireing%20diagram.png)

### Control Routing Architecture
Network routing architecture separating high-reliability manual control (Linux kernel IP forwarding) from autonomous computing (ROS 2 user space).

![Control Routing Architecture](wireing/control%20routing%20architecture.png)

## Software & Automation Architecture

### ROS 2 Node Architecture
Inter-process communication between sensor drivers, point cloud localization, navigation stack, trajectory analysis, and MCU UDP interface.

![ROS 2 Node Architecture](automation/ros2%20node%20diagram.png)

### System Operation State Machine
Operating mode transitions across manual teleoperation, autonomous navigation, moving target tracking, and safety fault states.

![System Operation State Machine](automation/state%20machine%20diagram.png)

### Navigation System Architecture & Toolchain
Data flow between gn10_navigation nodes, configuration maps, interactive VSCode extension tuning tool, and master MCU.

![Navigation System Architecture & Toolchain](automation/navigation%20node%20diagram.png)

### Navigation State Machine
State machine inside the navigation stack handling idle standby, dual-stage path pursuit, precision stopping, and target tracking.

![Navigation State Machine](automation/navigation%20state%20diagram.png)
