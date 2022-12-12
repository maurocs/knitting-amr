
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    base_node = Node(
        package="amr_hw",
        executable="amr_hw",
        parameters=[{"speed_mult":900.0}]
    )

    control_node = Node(
        package="amr_hw",
        executable="amr_controller"
    )
    
    odom_node = Node(
        package="amr_hw",
        executable="amr_odometry"
    )

    ld.add_action(base_node)
    ld.add_action(odom_node)
    ld.add_action(control_node)

    return ld