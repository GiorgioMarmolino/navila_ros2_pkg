from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    args = [
        DeclareLaunchArgument("rgb_topic", default_value="rgb_image"),
        DeclareLaunchArgument("depth_topic", default_value="depth_image"),
        DeclareLaunchArgument("camera_info_topic", default_value="camera_info"),
        DeclareLaunchArgument("camera_pcd_topic", default_value="camera_pcd"),
        DeclareLaunchArgument("frames_topic", default_value="camera_frames"),
        DeclareLaunchArgument("publish_separated_frames", default_value="true"),
        DeclareLaunchArgument("camera_resolution", default_value="HD"),
        DeclareLaunchArgument("compressed_image", default_value="true"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("serial_number", default_value=""),
        DeclareLaunchArgument("publish_rgb", default_value="true"),
        DeclareLaunchArgument("publish_depth", default_value="true"),
        DeclareLaunchArgument("publish_camera_info", default_value="true"),
        DeclareLaunchArgument("camera_type", default_value="zed"),
        DeclareLaunchArgument("device_idx", default_value="0"),
        # QoS dei topic immagine: "best_effort" (consigliato per stream ad alto rate) | "reliable"
        DeclareLaunchArgument("reliability", default_value="best_effort"),
        DeclareLaunchArgument("qos_depth", default_value="5"),
    ]

    # le LaunchConfiguration sono SEMPRE stringhe: int/bool vanno wrappati con
    # ParameterValue(..., value_type=...) o il nodo da' "parameter type mismatch"
    camera_node = Node(
        package="camera_utils_ros",
        executable="camera_publisher",
        name="camera_publisher",
        output="screen",
        respawn=False,
        parameters=[{
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "camera_pcd_topic": LaunchConfiguration("camera_pcd_topic"),
            "frames_topic": LaunchConfiguration("frames_topic"),
            "camera_resolution": LaunchConfiguration("camera_resolution"),
            "compressed_image": ParameterValue(LaunchConfiguration("compressed_image"), value_type=bool),
            "fps": ParameterValue(LaunchConfiguration("fps"), value_type=int),
            "serial_number": LaunchConfiguration("serial_number"),
            "publish_rgb": ParameterValue(LaunchConfiguration("publish_rgb"), value_type=bool),
            "publish_depth": ParameterValue(LaunchConfiguration("publish_depth"), value_type=bool),
            "publish_camera_info": ParameterValue(LaunchConfiguration("publish_camera_info"), value_type=bool),
            "publish_separated_frames": ParameterValue(LaunchConfiguration("publish_separated_frames"), value_type=bool),
            "camera_type": LaunchConfiguration("camera_type"),
            "device_idx": ParameterValue(LaunchConfiguration("device_idx"), value_type=int),
            "reliability": LaunchConfiguration("reliability"),
            "qos_depth": ParameterValue(LaunchConfiguration("qos_depth"), value_type=int),
        }],
    )

    return LaunchDescription(args + [camera_node])