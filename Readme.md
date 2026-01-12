Task 1: Single Arm Manipulation (MoveIt 2)

This repository contains the solution for Task 1 of the technical screening. It demonstrates a full MoveIt 2 integration for the Fanuc CRX-10iA cobot using a custom, standalone URDF.

📦 Repository Structure

dexsent_task1: Contains the robot description (URDF) utilizing standalone geometric primitives for optimized collision checking.

dexsent_task1_moveit_config: MoveIt 2 configuration, kinematics solvers, and launch pipelines.

🚀 Getting Started

Prerequisites

ROS 2 Humble

MoveIt 2 (sudo apt install ros-humble-moveit)

Installation

Clone this repo into your workspace src folder.

Build and source:

colcon build --packages-select dexsent_task1 dexsent_task1_moveit_config
source install/setup.bash


🛠️ Execution

Launch the MoveIt demo:

ros2 launch dexsent_task1_moveit_config task1_manual.launch.py


Instructions

In the RViz MotionPlanning panel, ensure the planning group is set to crx_arm.

Interact with the blue ball marker to set a target pose.

Click Plan & Execute.

The white ghost (Plan) will visualize the trajectory path computed by the solver.

💡 Technical Implementation

Collision Strategy: Used primitive shapes (cylinders/boxes) instead of meshes to ensure zero-dependency portability and high-speed collision checking.

Planning Pipeline: Integrated the CHOMP optimizer for smooth trajectory generation.
