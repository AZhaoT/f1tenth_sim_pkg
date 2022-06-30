from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    gap_finder_node = Node(
        package="f1tenth_sim",
        executable="gap_finder",
    )

    drive_to_gap_node = Node(
        package="f1tenth_sim",
        executable="drive_to_gap"
    )

    ld.add_action(gap_finder_node)
    ld.add_action(drive_to_gap_node)

    return ld