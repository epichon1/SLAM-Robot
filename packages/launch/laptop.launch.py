"""Launch RVIZ to show the bot URDF

   ros2 launch bot_description viewbot.launch.py

   Note, you may want to change the URDF for a different bot.

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                             import LaunchDescription
from launch.actions                     import Shutdown
from launch_ros.actions                 import Node
from launch.substitutions import Command


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # SELECT THE OPTIONS

    # Locate the URDF default file.
    
    x = os.path.join(pkgdir('bot_description'),
                        'urdf/vertex.urdf.xacro')

    # Load the robot's URDF file (XML).
    with open(x, 'r') as file:
        doc = xacro.parse(file)
        xacro.process_doc(doc)
        robot_description = doc.toxml()
    # robot_description = Command([
    # 'xacro ', os.path.join(pkgdir('bot_description'),
    #                        'urdf/vertex.urdf.xacro')
    # ])
    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('packages'), 'rviz/viewbot.rviz')

    mapfile = os.path.join(pkgdir('shared169'), 'maps/churchsideroom.yaml')


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher',
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}])

    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz',
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())
    
    node_localize = Node(
        name       = 'localize',
        package    = 'packages',
        executable = 'localize',
        on_exit    = Shutdown())

    node_planner = Node(
        name       = 'planner',
        package    = 'packages',
        executable = 'planner',
        on_exit    = Shutdown())
    
    node_human = Node(
        name       = 'human',
        package    = 'packages',
        executable = 'human',
        on_exit    = Shutdown())
    
    node_map_server = Node(
        name       = 'map_server',
        package    = 'nav2_map_server',
        executable = 'map_server',
        output     = 'screen',
        parameters = [{'yaml_filename': mapfile},
                      {'topic_name':    "map"},
                      {'frame_id':      "map"}])
    
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=['--ros-args', '-p', 'use_gui:=true']
    )


    # The Lifecycle manager allows for a smooth bring-up and shutdown
    # of many components, in this case just the map server.
    node_lifecycle = Node(
        name       = 'lifecycle_manager_localization',
        package    = 'nav2_lifecycle_manager',
        executable = 'lifecycle_manager',
        output     = 'screen',
        parameters = [{'autostart':    True},
                      {'node_names':   ['map_server']}])



    ######################################################################
    # RETURN THE ELEMENTS, built into a Launch Description list

    return LaunchDescription([
        # Start the robot_state_publisher and RVIZ.
        node_robot_state_publisher,
        node_rviz,
        node_localize,
        node_planner,
        #node_lifecycle,
        node_joint_state_publisher_gui,
        node_human,
        #node_map_server,
    ])
