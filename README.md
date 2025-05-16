# (UE22EC342BC2) - MOHITHA BAI S_Shivani Nittor - Color Sorting Robot Simulation

## Project Overview

This project implements a ROS 2 simulation of a wheeled mobile robot with a 2-DOF arm designed to sort colored objects (represented as spheres) into corresponding colored boxes. The simulation is visualized using RViz. Due to constraints, a simplified kinematic approach is used where the robot base and arm perform pre-scripted "sweep" motions, and object interaction (pick/place) is simulated by changing the object's TF frame.

## Team Members

- MOHITHA BAI S 
- Shivani Nittor

## Software Requirements

- Ubuntu 22.04 (or compatible Linux distribution)
- ROS 2 Humble Hawksbill
- Standard ROS 2 packages (rclpy, std_msgs, sensor_msgs, visualization_msgs, geometry_msgs, tf2_ros, robot_state_publisher, joint_state_publisher_gui, rviz2)
- colcon (for building)

## Package Structure

The `simple_sorter_sim` package contains:
- `urdf/simple_arm.urdf`: Describes the wheeled robot model with a 2-DOF arm.
- `launch/sorter_simulation.launch.py`: Launches the simulation nodes, including RViz, robot_state_publisher, and the main logic node.
- `scripts/sorter_logic.py`: The main Python node that controls the robot base movement, arm sweeps, object states, and publishes markers for visualization.
- `rviz/sorter_sim.rviz`: Default RViz configuration for viewing the simulation.
- `package.xml`: ROS 2 package manifest.
- `setup.py`: Python package setup script.


## Simulation Logic

- The robot base is programmed to "drive" (interpolate its TF transform) to a predefined area near the objects to be sorted.
- The 2-DOF arm then performs a "sweep" motion towards the target object based on its color.
- Object "picking" is simulated by changing the object marker's TF frame to be a child of the robot's `gripper_link`.
- The robot base then "drives" to a predefined area near the corresponding colored box.
- The arm performs another "sweep" motion for placing.
- Object "placing" is simulated by changing the object marker's TF frame back to the `world` frame and setting its position within the target box.
- The sequence repeats for all defined objects.

## Known Issues / Current State

- **Visual Lag:** There can be a visual lag where the arm appears momentarily detached from the robot base when the base is moving. The arm snaps back to its correct position once the base stops. This is an artifact of the simplified simulation approach where base and arm TFs are updated rapidly but processed by RViz with its own rendering cycle.
- **Ball Carrying Visual:** The visual attachment of the ball to the gripper might sometimes be inconsistent due to TF update timing in RViz. The logic for changing the ball's `frame_id` is in place, but the visual update in RViz can sometimes lag or miss an intermediate state during rapid movements.
- **Arm Reach and Precision:** The arm uses pre-defined "sweep" angles. It does not perform precise inverse kinematics to reach exact points. The success of picking/placing relies on the base positioning itself appropriately and the general sweep of the arm being in the correct direction. The ball "teleports" to/from the gripper when the logic dictates a pick/place.

## Video of Output

 A video demonstrating the simulation output can be found at `videos/MOHITHA_BAI_S_Shivani_Nittor_Video.webm`)



---