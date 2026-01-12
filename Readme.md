# Task 1 – Single Arm Manipulation (MoveIt 2)

This repository contains the solution for Task 1 of the technical screening. It demonstrates a complete MoveIt 2 integration for the Fanuc CRX-10iA collaborative robot using a custom, standalone URDF and an optimized collision model.

## 📦 Repository Structure

```
.
├── dexsent_task1
│   └── description/
│       └── urdf/
│           └── crx10ia.urdf.xacro
│
├── dexsent_task1_moveit_config
│   ├── config/
│   │   ├── kinematics.yaml
│   │   ├── joint_limits.yaml
│   │   ├── ompl_planning.yaml
│   │   └── chomp_planning.yaml
│   ├── launch/
│   │   └── task1_manual.launch.py
│   └── rviz/
│       └── moveit.rviz
│
└── README.md
```

- **`dexsent_task1`**: Contains the robot description (URDF/Xacro) using standalone geometric primitives for fast and portable collision checking.
- **`dexsent_task1_moveit_config`**: Full MoveIt 2 configuration including planning pipelines, kinematics solvers, and launch files.

## 🚀 Getting Started

### Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **MoveIt 2**

#### Install MoveIt 2:

```bash
sudo apt update
sudo apt install ros-humble-moveit
```

### Installation

1. **Create and initialize a ROS 2 workspace** (if not already done):

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. **Clone this repository** into the `src` folder:

```bash
cd src
git clone <repository-url>
```

3. **Build the packages**:

```bash
cd ~/ros2_ws
colcon build --packages-select dexsent_task1 dexsent_task1_moveit_config
```

4. **Source the workspace**:

```bash
source install/setup.bash
```

## 🛠️ Execution

Launch the MoveIt 2 demo:

```bash
ros2 launch dexsent_task1_moveit_config task1_manual.launch.py
```

This will start:
- RViz with the Motion Planning plugin
- MoveIt 2 planning scene
- Fanuc CRX-10iA robot model

## 🎮 Usage Instructions

1. In RViz, open the **MotionPlanning** panel.
2. Set the **Planning Group** to:
   ```
   crx_arm
   ```
3. Use the interactive blue ball marker to specify a target end-effector pose.
4. Click **Plan & Execute**.

### Visualization

- The **white ghost robot** shows the planned trajectory.
- The **actual robot model** follows the executed path.

## 💡 Technical Implementation Details

### Collision Strategy

- The robot collision model is built entirely from **primitive shapes** (boxes and cylinders).
- Avoids mesh dependencies for:
  - Faster collision checking
  - Improved portability
  - Reduced computational overhead

### Planning Pipeline

- Integrated **CHOMP** as the trajectory optimization pipeline.
- Provides:
  - Smooth joint-space trajectories
  - Collision-aware optimization
  - Continuous path refinement

### Kinematics

- Configured with MoveIt 2 kinematics plugins.
- Supports interactive pose targeting via RViz.

## ✅ Key Features

- ✨ Custom URDF (no vendor meshes required)
- ⚡ Optimized collision geometry
- 🎯 CHOMP-based trajectory optimization
- 🖱️ Interactive RViz planning and execution
- 🤖 ROS 2 Humble & MoveIt 2 compliant

## 📌 Notes

- This setup is intended for **simulation and planning demonstration** purposes.
- No hardware interfaces are enabled.
- The configuration can be extended to real hardware by adding appropriate controllers and drivers.

## 📄 License

This project is provided for technical evaluation purposes. All rights reserved by the author.



