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


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # SELECT THE OPTIONS

    # Locate the URDF default file.
    urdf = os.path.join(pkgdir('bot_description'),
                        'urdf/vertex_description.urdf.xacro')

    # Load the robot's URDF file (XML).
    with open(urdf, 'r') as file:
        doc = xacro.parse(file)
        xacro.process_doc(doc)
        robot_description = doc.toxml()

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('packages'), 'rviz/viewbot.rviz')


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS
    
    # Configure a node for odometry
    node_odometry = Node(
        name       = 'odometry',
        package    = 'packages',
        executable = 'odom_vertex',
        on_exit    = Shutdown())
    
    # Configure a node for wheel control
    node_wheelcontrol = Node(
        name       = 'wheelcontrol',
        package    = 'packages',
        executable = 'wheelcontrolplus',
        on_exit    = Shutdown())
    
    node_aruco = Node(
        name       = 'aruco', 
        package    = 'shared169',
        executable = 'detector',
        output     = 'screen',
        remappings = [('/image_raw', '/usb_cam/image_raw')])

    node_usbcam = Node(
        name       = 'usb_cam', 
        package    = 'usb_cam',
        executable = 'usb_cam_node_exe',
        namespace  = 'usb_cam',
        output     = 'screen',
        parameters = [{'camera_name':  'logitech'},
                      {'video_device': '/dev/video0'},
                      {'pixel_format': 'yuyv2rgb'},
                      {'image_width':  640},
                      {'image_height': 480},
                      {'framerate':    5.0}])

    node_autodrive = Node(
        name       = 'autodrive',
        package    = 'packages',
        executable = 'auto_vertex',
        on_exit    = Shutdown())

    node_localize = Node(
        name       = 'localize',
        package    = 'packages',
        executable = 'localize',
        on_exit    = Shutdown())


    ######################################################################
    # RETURN THE ELEMENTS, built into a Launch Description list

    return LaunchDescription([
        # Start the robot_state_publisher and RVIZ
        node_odometry,
        #node_wheelcontrol,
        node_autodrive,
        node_aruco,
        #node_usbcam,
        #node_localize,
    ])
