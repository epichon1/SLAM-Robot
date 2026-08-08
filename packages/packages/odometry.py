#!/usr/bin/env python3
#
#   odometry_skeleton.py
#
#   THIS IS A SKELETON ONLY.  PLEASE COPY/RENAME AND THEN EDIT!
#
#   Odometry node.  This
#   (a) converts both a body velocity command to wheel velocity commands.
#   (b) estimates the body velocity and pose from the wheel motions
#       and the gyroscope.
#
#   Node:       /odometry
#   Subscribe:      /cmd_vel            geometry_msgs/Twist
#                   /wheel_state        sensor_msgs/JointState
#   Publish:        /wheel_command      sensor_msgs/JointState
#                   /odom               nav_msgs/Odometry
#   TF Broadcast:   odom -> base        geometry_msgs/TransformStamped
#
import rclpy
import traceback

from math import pi, sin, cos
import numpy as np

from rclpy.node         import Node
from tf2_ros            import TransformBroadcaster
from geometry_msgs.msg  import Point, Quaternion, Twist
from geometry_msgs.msg  import TransformStamped, Vector3
from nav_msgs.msg       import Odometry
from sensor_msgs.msg    import JointState


#
#   Constants
#
R = 0.033              # Wheel radius
d = 0.0645             # Halfwidth between wheels

LEFTNAME  = 'leftwheel'
RIGHTNAME = 'rightwheel'

#
#   Odometry Node Class
#
class OdometryNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Set the initial pose to zero.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.lpsi = 0.0
        self.rpsi = 0.0

        # Create the publishers for the wheel commands and the
        # odometry information.
        self.pubwcmd = self.create_publisher(JointState, 'wheel_command', 3)
        self.pubodom = self.create_publisher(Odometry, 'odom', 10)

        # Initialize the transform broadcaster
        self.tfbroadcaster = TransformBroadcaster(self)

        # Create the subscribers to listen to wheel state and twist
        # comamnds.
        self.subwact = self.create_subscription(
            JointState, 'wheel_state', self.cb_wheelmsg, 10)
        self.subvcmd = self.create_subscription(
            Twist, 'cmd_vel', self.cb_vcmdmsg, 10)

        # Report and return.
        self.get_logger().info("Odometry running")

    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()


    # Velocity Command Message Callback
    def cb_vcmdmsg(self, msg):
        # Grab the forward and spin (velocity) commands.
        vxcmd = msg.linear.x
        wzcmd = msg.angular.z

        lpsi_dot = (vxcmd - d*wzcmd)/R
        rpsi_dot = (vxcmd + d*wzcmd)/R

        # Create the wheel command msg and publish.  Note the incoming
        # message does not have a time stamp, so generate one here.
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = [LEFTNAME, RIGHTNAME]
        msg.velocity     = [lpsi_dot, rpsi_dot]
        self.pubwcmd.publish(msg)


    # Wheel State Message Callback
    def cb_wheelmsg(self, msg):
        # Grab the timestamp, wheel and gyro position/velocities.
        try:
            timestamp = msg.header.stamp
            lpsi      = msg.position[msg.name.index('leftwheel')]
            lpsi_dot  = msg.velocity[msg.name.index('leftwheel')]
            rpsi      = msg.position[msg.name.index('rightwheel')]
            rpsi_dot  = msg.velocity[msg.name.index('rightwheel')]
            #theta     = msg.position[msg.name.index('gyro')]
            #omega     = msg.velocity[msg.name.index('gyro')]
        except:
            self.get_logger().error("Ill-formed /wheel_state message!")
            return

        dpsi_l = lpsi - self.lpsi
        dpsi_r = rpsi - self.rpsi
        self.lpsi = lpsi
        self.rpsi = rpsi

        dp = 0.5*R*(dpsi_l + dpsi_r)
        dtheta = (0.5*R/d)*(-dpsi_l + dpsi_r)

        self.x += dp*cos(self.theta + dtheta/2)*np.sinc(dtheta/(2*pi))
        self.y += dp*sin(self.theta + dtheta/2)*np.sinc(dtheta/(2*pi))
        self.theta += dtheta
        #self.theta = theta

        vxact = (lpsi_dot + rpsi_dot)*(0.5*R)
        wzact = (-lpsi_dot + rpsi_dot)*(0.5*R/d)
        #wzact = omega

        # print(f"encoder: theta = {self.theta}, omega ={wzact}         gyro: theta = {theta}, omega = {omega}")
        # Create the odometry msg and publish (reuse the time stamp).
        msg = Odometry()
        msg.header.stamp            = timestamp
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'chassis'
        msg.pose.pose.position.x    = self.x
        msg.pose.pose.position.y    = self.y
        msg.pose.pose.orientation.z = sin(self.theta/2)
        msg.pose.pose.orientation.w = cos(self.theta/2)
        msg.twist.twist.linear.x    = vxact
        msg.twist.twist.angular.z   = wzact
        self.pubodom.publish(msg)

        # Create the transform and broadcast (reuse the time stamp).
        trans = TransformStamped()
        trans.header.stamp            = timestamp
        trans.header.frame_id         = 'odom'
        trans.child_frame_id          = 'chassis'
        trans.transform.translation.x = self.x
        trans.transform.translation.y = self.y
        trans.transform.rotation.z    = sin(self.theta/2)
        trans.transform.rotation.w    = cos(self.theta/2)
        self.tfbroadcaster.sendTransform(trans)


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = OdometryNode('odometry')

    # Spin the node until interrupted.
    try:
        rclpy.spin(node)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
