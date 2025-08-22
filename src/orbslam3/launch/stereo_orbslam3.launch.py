from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orbslam3',
            executable='stereo-inertial',
            name='stereo_inertial',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
            arguments=[
                '/home/sid/ORB_SLAM3/Vocabulary/ORBvoc.txt',          # Vocabulary
                '/home/sid/orb_ws/ZED_stereo_initial.yaml',           # Config file
                'false'                                               # DoRectify flag
            ],
            # remappings=[
            #     ('/zed/zed_node/left/image_rect_color', '/camera/left/image_raw'),
            #     ('/zed/zed_node/right/image_rect_color', '/camera/right/image_raw'),
            #     ('/zed/zed_node/left/camera_info', '/camera/left/camera_info'),
            #     ('/zed/zed_node/right/camera_info', '/camera/right/camera_info'),
            #     ('/zed/zed_node/imu/data', '/imu'),
            # ]


        )
    ])
