# rover_autonomy
This is a repository for a (work-in-progress) ROS2 based software stack for an Ackermann Steered, RC car based, land vehicle capable of autonomously traversing GPS Waypoints. The repo root is the root of a ROS2 colcon workspace.


 ## High Level System Architecture for ROS2-based Autonomous Rover
 High level overview of ROS2-based modular architecture for autonomous rover. Includes ROS2 Lifecycle nodes for localization, waypoint following, real-time control, and telemetry streaming over long-range radio to an off-board GUI. Detailed documentation describing the ROS2 interfaces (messages, services & actions) and data flow and logic in each of the packages is in the works.  

![image](https://github.com/user-attachments/assets/86d441dc-08d0-451a-988b-73b4f20a3d36)  

- `serial_interface`: ROS2 package run as a lifecycle node to connect to and read from a serial connection and publish read data over a ROS2 topic [Initial Working version: Done]  
- `um982_ros_driver` & `unicore_msgs`: ROS2 packages with the driver run as lifecycle node to parse a limited set of *Binary* Unicore Logs from the Unicore UM982 GNSS receiver, connected to over a serial connection and publish parsed data over ROS2 topics. [Initial Working version: Done]  
- `olixsense_imu_node`: The Olixsense IMU runs a ROS2 node on its own internal microprocessor and publishes IMU data over a ROS2 topic [OEM software: tested to be working: Done]  
- `localization`: ROS2 package to fuse data from the IMU and GNSS [Development in Progress]  

More packages such as `route_manager`, `route_tracker` (waypoint following), `state_manager` (supervisory), `rc_car_interface` etc  will be added to the repo as they are finished



