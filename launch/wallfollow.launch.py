from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    dist_finder_node = Node(
        package="f1tenth_sim",
        executable="dist_finder",
    )

    pid_drive_node = Node(
        package="f1tenth_sim",
        executable="control"
    )

    ld.add_action(dist_finder_node)
    ld.add_action(pid_drive_node)

    return ld