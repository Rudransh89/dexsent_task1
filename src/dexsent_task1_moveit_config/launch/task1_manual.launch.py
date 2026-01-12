import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths
    pkg_task1 = get_package_share_directory('dexsent_task1')
    pkg_config = get_package_share_directory('dexsent_task1_moveit_config')
    
    urdf_path = os.path.join(pkg_task1, 'urdf', 'crx10ia_robotiq.urdf')
    srdf_path = os.path.join(pkg_config, 'config', 'crx10ia_robotiq.srdf')
    rviz_config_path = os.path.join(pkg_config, 'config', 'moveit.rviz')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    with open(srdf_path, 'r') as f:
        robot_desc_semantic = f.read()

    # 2. Kinematics (The Solver)
    kinematics_yaml = {
        "crx_arm": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.005
        }
    }

    # 3. Controller Manager Config (The "Brain" of Execution)
    # We explicitly map the controllers to the joints here.
    controllers_yaml = {
        "moveit_controller_manager": "moveit_fake_controller_manager/MoveItFakeControllerManager",
        "moveit_manage_controllers": True,
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 1.2,
            "allowed_goal_duration_margin": 0.5,
            "allowed_start_tolerance": 0.01
        },
        "controller_names": ["crx_arm_controller", "gripper_controller"],
        "crx_arm_controller": {
            "action_ns": "follow_joint_trajectory",
            "type": "FollowJointTrajectory",
            "default": True,
            "joints": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        },
        "gripper_controller": {
            "action_ns": "follow_joint_trajectory",
            "type": "FollowJointTrajectory",
            "default": True,
            "joints": ["finger_joint"]
        }
    }

    # 4. Combine Parameters
    params = {
        "robot_description": robot_desc,
        "robot_description_semantic": robot_desc_semantic,
        "robot_description_kinematics": kinematics_yaml,
        "use_sim_time": False,
        "publish_robot_description_semantic": True
    }
    params.update(controllers_yaml)

    # 5. Nodes
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[params]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[params]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'source_list': ['move_group/fake_controller_joint_states']}],
        ),
    ])
