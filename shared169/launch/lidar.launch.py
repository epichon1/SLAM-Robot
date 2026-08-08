"""Launch the lidar nodes

   ros2 launch shared169 lidar.launch.py

   Note, launch this once and keep running!  This will stop the lidar
   from spinning when not in use.  It automatically starts up again
   when anything listens to the /scan topic.

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
        remappings = [('/scan', '/scanoriginal')],
        on_exit    = Shutdown())

    # Configure a node for the lidar fix!
    # Remap /scanoriginal as the input, /scan as the output.
    # This also stops the lidar when not in use.
    node_lidarfix = Node(
        name       = 'lidarfix',
        package    = 'shared169',
        executable = 'rplidarfix',
        output     = 'screen',
        parameters = [{'timeshift':  0.027,     # This was hand-tuned
                       'startdelay': 3.0,       # Time for the IMU
                       'autostart':  True,      # Turn ON when listening
                       'autostop':   True,      # Turn OFF when unused
                       }],
        remappings = [('/scanin',  '/scanoriginal'),
                      ('/scanout', '/scan')],
        on_exit    = Shutdown())


    ######################################################################
    # RETURN THE ELEMENTS, built into a Launch Description list

    return LaunchDescription([
        # Start the lidar and the lidar-fix filter.
        node_lidar,
        node_lidarfix,
    ])
