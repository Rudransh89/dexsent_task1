from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load the MoveIt Configuration we just generated
    moveit_config = MoveItConfigsBuilder("dexsent_task1", package_name="dexsent_task1_moveit_config").to_moveit_configs()

    # 2. Define the Nodes
    return LaunchDescription([
        
        # A. Robot State Publisher (Publishes the URDF to TF)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
        ),

        # B. Move Group (The Motion Planning "Brain")
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": False},
                {"publish_robot_description_semantic": True}
            ],
        ),

        # C. RViz (The Visualizer)
        Node(
            package="moveit_ros_visualization",
            executable="moveit_rviz",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
            ],
        ),

        # D. Fake Joint Driver (Simulates the robot moving so you can see execution)
        # This publishes "fake" joint states based on where MoveIt thinks the robot is.
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            parameters=[{
                "source_list": ["move_group/fake_controller_joint_states"],
            }],
        ),
    ])
