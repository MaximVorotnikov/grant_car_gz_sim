import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r diff_drive_rgbd_two_cars.sdf'
        }.items(),
    )

#     bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         arguments=['/model/r0/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
#                     '/model/r0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
#                     '/lidar0@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
#                     '/rgbd_camera0/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
#                     '/rgbd_camera0/image@sensor_msgs/msg/Image@gz.msgs.Image',
#                     '/rgbd_camera0/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
#                     '/rgbd_camera0/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
#                     '/rgbd_camera1/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
#                     '/rgbd_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
#                     '/rgbd_camera1/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
#                     '/rgbd_camera1/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
#                     '/model/r1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
#                     '/model/r1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
#                     '/lidar1@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
#         # parameters=[{'qos_overrides./model/r0.subscriber.reliability': 'reliable',
#         #                 'qos_overrides./model/r1.subscriber.reliability': 'reliable'}],
#         output='screen'
# )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/r0/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/r0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/r0/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/r0/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    '/r0/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/r0/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/r0/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                    '/r1/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    '/r1/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/r1/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/r1/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                    '/model/r1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/r1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/r1/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        # parameters=[{'qos_overrides./model/r0.subscriber.reliability': 'reliable',
        #                 'qos_overrides./model/r1.subscriber.reliability': 'reliable'}],
        remappings=[
                    ('/model/r0/cmd_vel', '/r0/cmd_vel'),
                    ('/model/r0/odometry', '/r0/odometry'),
                    ('/model/r1/cmd_vel', '/r1/cmd_vel'),
                    ('/model/r1/odometry', '/r1/odometry')],
        output='screen'
)

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        # rviz
    ])