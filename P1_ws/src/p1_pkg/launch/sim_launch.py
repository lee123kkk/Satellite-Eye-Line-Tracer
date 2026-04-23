import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'p1_pkg'
    pkg_path = get_package_share_directory(pkg_name)
    
    urdf_file = os.path.join(pkg_path, 'urdf', 'rc_car.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'track.world')
    
    doc = xacro.process_file(urdf_file)
    robot_description = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 모던 Gazebo 서버 실행
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'rc_car', 
            '-x', '1.0',    # 트랙 시작점(1.5) 근처로 X 좌표 이동
            '-y', '-1.0',   # 트랙 시작점(-1.5) 근처로 Y 좌표 이동
            '-z', '0.3'     # 바닥을 파고들지 않도록 30cm 공중에서 생성 (자유낙하)
        ],
        output='screen'
    )

    # ROS 2 <-> Gazebo 통신 브릿지 (모터 제어 및 카메라 영상)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge
    ])
