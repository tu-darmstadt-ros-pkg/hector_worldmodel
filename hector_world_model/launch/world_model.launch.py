from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="ec_swift", description="Top-level namespace"
    )

    use_bag_detections = DeclareLaunchArgument(
        "use_bag_detections",
        default_value="false",
        description="Use detections from a bag file instead of the world model",
    )

    # Use a TimerAction to delay node start by 5 seconds
    delayed_node = TimerAction(
        period=1.0,
        actions=[
            PushRosNamespace(namespace=LaunchConfiguration("namespace")),
            Node(
                package="hector_world_model",
                executable="world_model",
                name="world_model_node",
                prefix=["gdbserver localhost:3000"],
                output="screen",
                parameters=[
                    {"use_bag_detections": LaunchConfiguration("use_bag_detections")}
                ],
            ),
        ],
    )

    return LaunchDescription([namespace_arg, delayed_node])
