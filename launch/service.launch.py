from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 
def generate_launch_description():
    publisher_freq_launch_arg = DeclareLaunchArgument(
        "publish_freq", default_value=TextSubstitution(text="500")
    )
    record_bag_flag_arg = DeclareLaunchArgument(
        "record_flag", default_value=TextSubstitution(text="1")
    )
    talker_node = Node(
        package="cpp_srvcli",
        executable="talker",
        parameters=[{"publish_frequency": LaunchConfiguration('publish_freq'), "record_bag": LaunchConfiguration('record_bag')}]
    )
    return LaunchDescription([publisher_freq_launch_arg, record_bag_flag_arg, talker_node])