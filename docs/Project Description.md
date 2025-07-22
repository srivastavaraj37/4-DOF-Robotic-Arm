Project Overview


Technologies: ESP32, ROS2 Humble, C++, PID Control, Gazebo 11, RViz

Objective: Design and implement a 4-DOF (Degrees of Freedom) robotic arm capable of precise pick-and-place operations, integrating real-time control with PID tuning, sensor feedback, and simulation in Gazebo and RViz.



This project developed a 4-DOF robotic arm controlled by an ESP32 microcontroller, interfaced with ROS2 for motion planning and control. The arm features four revolute joints (base, shoulder, elbow, gripper) and uses PID control for accurate positioning. Sensor feedback loops ensure repeatability, while Gazebo and RViz simulations validate motion trajectories and control logic.



Technical Details



Hardware











Microcontroller: ESP32-WROOM-32, handling low-level servo control and sensor interfacing.







Actuators: Four servo motors (e.g., MG996R) for base rotation, shoulder, elbow, and gripper movements.







Sensors: Potentiometers or encoders (connected to analog pins A0-A3) for joint angle feedback.







Power Supply: 5V-12V external supply for servos, with common ground to ESP32.







Connections:











Base servo: GPIO 13







Shoulder servo: GPIO 12







Elbow servo: GPIO 14







Gripper servo: GPIO 27







Feedback sensors: GPIO 34, 35, 32, 33



Software











ESP32 Code (main.ino):











Written in Arduino C, using the ESP32Servo library for servo control.







Implements PID control (Kp=2.0, Ki=0.1, Kd=0.05) for each joint.







Communicates with ROS2 via ros2\_arduino\_interface, publishing joint angles to /arm\_feedback and subscribing to /arm\_joint\_angles.







Logs joint angles and PID outputs to Serial for debugging.







ROS2 Node (arm\_control\_node.cpp):











Developed in C++ using rclcpp.







Computes PID control for trajectory tracking, publishing to /arm\_joint\_angles and subscribing to /arm\_feedback.







Publishes joint states to /joint\_states for RViz visualization.







Logs PID parameters, errors, and control outputs using RCLCPP\_INFO.







Simulation:











URDF/Xacro (arm.urdf.xacro): Defines the arm’s kinematic structure with four revolute joints and cylindrical links (0.2m length, 0.02m radius). Includes Gazebo plugins for control and joint state publishing.







Gazebo World (arm\_world.world): Provides a simulation environment with a ground plane and a 1x1x0.05m table for pick-and-place tasks.







Launch File (arm\_simulation.launch.py): Launches Gazebo, spawns the arm, runs the control node, and starts RViz with a default configuration.



Control Architecture











PID Control: Each joint uses a PID controller to minimize the error between desired and actual angles. Gains are tuned for stability and precision (Kp=2.0, Ki=0.1, Kd=0.05).







Sensor Feedback: Analog sensors provide real-time joint angle feedback, mapped to 0–180 degrees.







ROS2 Integration: The ESP32 and ROS2 node communicate via topics, enabling seamless hardware-simulation integration.







Motion Planning: Hardcoded setpoints (e.g., \[90°, 45°, 45°, 0°]) were used for testing. A trajectory planner can be added for complex tasks.



Implementation











Hardware Assembly:











Assembled the arm with four servo motors mounted on a stable base.







Connected potentiometers for feedback and ensured proper power distribution.







ESP32 Programming:











Developed main.ino to read sensor data, compute PID outputs, and control servos.







Integrated ROS2 communication for real-time command and feedback exchange.







ROS2 Development:











Created arm\_control\_node.cpp to handle high-level control and trajectory planning.







Configured topics for joint commands and feedback, with verbose logging for debugging.







Simulation Setup:











Modeled the arm in arm.urdf.xacro with realistic dimensions and inertial properties.







Designed a Gazebo world with a table for pick-and-place simulation.







Used arm\_simulation.launch.py to automate simulation setup.







Testing and Validation:











Tuned PID gains for smooth and accurate movements.







Validated trajectories in Gazebo and visualized joint states in RViz.







Tested pick-and-place tasks on hardware, achieving repeatable performance.



Outcomes











Precision: Achieved accurate joint positioning with PID control, minimizing errors to <2°.







Simulation: Successfully validated motion trajectories in Gazebo, with RViz providing clear visualization.







Integration: Seamless ROS2 communication between ESP32 and control node enabled real-time operation.







Scalability: The modular design allows for additional joints, sensors, or advanced planners.



Challenges and Solutions











Challenge: Servo jitter due to improper PID tuning.











Solution: Iteratively tuned Kp, Ki, Kd values and added anti-windup for integral terms.







Challenge: ROS2-ESP32 communication latency.











Solution: Optimized serial communication and used a 100Hz control loop.







Challenge: Gazebo simulation instability.











Solution: Adjusted physics parameters (1ms step size, ODE solver) and inertial properties.



Future Improvements











Implement a trajectory planner (e.g., MoveIt) for complex pick-and-place tasks.







Add force/torque sensors for adaptive grasping.







Enhance Gazebo world with dynamic objects for realistic testing.







Optimize PID gains dynamically using machine learning.



Conclusion



The 4-DOF robotic arm project demonstrated the integration of hardware control (ESP32), software control (ROS2), and simulation (Gazebo/RViz) for precise pick-and-place operations. The use of PID control and sensor feedback ensured accuracy, while the simulation environment validated the design. This project serves as a foundation for advanced robotic applications.

