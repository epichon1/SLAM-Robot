"""Launch the Map Server

   ros2 launch shared169 mapserver.launch.py

   Please transfer the necessary pieces to your launch file.

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

    # Locate the map.
    mapfile = os.path.join(pkgdir('shared169'), 'maps/churchsidemaze1b.yaml')


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # The map server publishes an existing map.  It is part of the
    # navigation stack, which requires a lifecycle manager to start.
    node_map_server = Node(
        name       = 'map_server',
        package    = 'nav2_map_server',
        executable = 'map_server',
        output     = 'screen',
        parameters = [{'yaml_filename': mapfile},
                      {'topic_name':    "map"},
                      {'frame_id':      "map"}])

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
        # Start the map server and lifecycle manager.
        node_map_server,
        node_lifecycle,
    ])
