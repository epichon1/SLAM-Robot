"""Run the vertex_planning stack against the simulated drivetrain.

   ros2 launch vertex_planning/vertex_media/demo.launch.py

   This is the laptop.launch.py of the repo with the hardware-only pieces
   swapped out: odom_vertex runs as a kinematic simulator (its cb_vcmdmsg
   already integrates /cmd_vel and broadcasts odom->world), and goal_sender.py
   supplies the /planner_goal_pose messages a human would otherwise click in
   RVIZ.  No lidar, camera, or wheel hardware is involved.
"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                import LaunchDescription
from launch.actions        import (Shutdown, ExecuteProcess, TimerAction,
                                   DeclareLaunchArgument)
from launch.conditions     import IfCondition
from launch.substitutions  import LaunchConfiguration
from launch_ros.actions    import Node


MEDIA = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():

    with open(os.path.join(MEDIA, 'vertex.urdf'), 'r') as file:
        robot_description = file.read()

    rvizcfg = os.path.join(MEDIA, 'demo_view.rviz')
    mapfile = os.path.join(pkgdir('shared169'), 'maps/churchsidemaze1b.yaml')

    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher',
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}])

    node_rviz = Node(
        name       = 'rviz',
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())

    node_map_server = Node(
        name       = 'map_server',
        package    = 'nav2_map_server',
        executable = 'map_server',
        output     = 'screen',
        parameters = [{'yaml_filename': mapfile},
                      {'topic_name':    "map"},
                      {'frame_id':      "map"}])

    node_lifecycle = Node(
        name       = 'lifecycle_manager_localization',
        package    = 'nav2_lifecycle_manager',
        executable = 'lifecycle_manager',
        output     = 'screen',
        parameters = [{'autostart':  True},
                      {'node_names': ['map_server']}])

    # Dead-reckoning localizer.  Without a /scan it never runs its ICP
    # correction, but it still publishes map->odom and the map->grid transform
    # taken from the map origin, which is what the planner plans in.
    node_localize = Node(
        name       = 'localize',
        package    = 'packages',
        executable = 'localize',
        output     = 'screen',
        on_exit    = Shutdown())

    # RRT planner: /planner_goal_pose in, /path out.  Its /initialpose
    # subscription is remapped away: on the real robot that message arms an
    # autonomous coverage mode that picks its own random goals, which would
    # race the scripted tour.  The localizer still gets the real /initialpose.
    node_planner = Node(
        name       = 'planner',
        package    = 'packages',
        executable = 'planner',
        output     = 'screen',
        remappings = [('/initialpose', '/planner_initialpose')],
        on_exit    = Shutdown())

    # Waypoint follower: /path in, /cmd_vel out.
    node_autodrive = Node(
        name       = 'autodrive',
        package    = 'packages',
        executable = 'auto_vertex',
        output     = 'screen',
        on_exit    = Shutdown())

    # Drivetrain simulator: /cmd_vel in, /odom and odom->world out.
    node_odometry = Node(
        name       = 'odometry',
        package    = 'packages',
        executable = 'odom_vertex',
        output     = 'screen',
        on_exit    = Shutdown())

    # Goal source.  Held back until the map has been published and the
    # planner has finished its (slow) wall-distance precomputation.
    node_goals = TimerAction(
        period  = 25.0,
        condition = IfCondition(LaunchConfiguration('goals')),
        actions = [ExecuteProcess(
            cmd    = ['python3', '-u', os.path.join(MEDIA, 'goal_sender.py')],
            name   = 'goal_sender',
            output = 'screen')])

    return LaunchDescription([
        DeclareLaunchArgument('goals', default_value='true',
            description='run goal_sender.py to drive the scripted tour'),
        node_robot_state_publisher,
        node_rviz,
        node_map_server,
        node_lifecycle,
        node_localize,
        node_planner,
        node_autodrive,
        node_odometry,
        node_goals,
    ])
