"""Launch RVIZ to show the lidar data

   ros2 launch shared169 viewlidar.launch.py

   Simply view the lidar data in RVIZ.

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                             import LaunchDescription
from launch.actions                     import Shutdown
from launch_ros.actions                 import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # SELECT THE OPTIONS

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('shared169'), 'rviz/viewlidar.rviz')


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure the standard lidar node.
    # Remap /scanoriginal as the scan data.
    node_lidar = Node(
        name       = 'lidar',
        package    = 'rplidar_ros',
        executable = 'rplidar_composition',
        output     = 'screen',
        parameters = [{'serial_port':      '/dev/ttyUSB0',
                       'serial_baudrate':  115200,          # RPLIDAR A1, A2
                     # 'serial_baudrate':  256000,          # RPLIDAR A3
                       'frame_id':         'lidar',
                       'inverted':         False,
                       'angle_compensate': True,
                       }],
        remappings = [('/scan', '/scanoriginal')])

    # Configure a node for the lidar fix!
    # Remap /scanoriginal as the input, /scan as the output.
    node_lidarfix = Node(
        name       = 'lidarfix',
        package    = 'shared169',
        executable = 'rplidarfix',
        output     = 'screen',
        parameters = [{'timeshift':  0.027,     # This was hand-tuned
                       'startdelay': 3.0,       # Time for the IMU
                       'autostart':  False,     # Turn lidar ON when used
                       'autostop':   False,     # Turn lidar OFF when unused
                       }],
        remappings = [('/scanin',  '/scanoriginal'),
                      ('/scanout', '/scan')])

    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz',
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())


    ######################################################################
    # RETURN THE ELEMENTS, built into a Launch Description list

    return LaunchDescription([
        # Start the lidar, the lidar-fix filter, and RVIZ.
        node_lidar,
        node_lidarfix,
        node_rviz,
    ])
