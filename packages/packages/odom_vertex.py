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

from math import pi, sin, cos, sqrt, atan2
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
R = 0.4064            # Wheel radius
d = 1.2446/2             # Halfwidth between wheels
w = 1.6256/2
SERVORATE    = 200.0

LF = [w,d]
RF = [w,-d]
LB = [-w,d]
RB = [-w,-d]

DISTANCES = [LF,RF,LB,RB]

LFNAME = 'front_left_wheel'
RFNAME = 'front_right_wheel'
LBNAME = 'rear_left_wheel'
RBNAME = 'rear_right_wheel'
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

        self.lfpsi = 0.0
        self.rfpsi = 0.0
        self.lbpsi = 0.0
        self.rbpsi = 0.0
        self.last_now = None

        # Create the publishers for the wheel commands and the
        # odometry information.
        self.pubwcmd = self.create_publisher(JointState, 'wheel_command', 3)
        self.pubodom = self.create_publisher(Odometry, 'odom', 10)
        self.timer   = self.create_timer(1/SERVORATE, self.cb_timer)

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

    
    def cb_timer(self):
        # Create the odometry msg and publish (reuse the time stamp).
        timestamp = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp            = timestamp
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'world'
        msg.pose.pose.position.x    = self.x
        msg.pose.pose.position.y    = self.y
        msg.pose.pose.orientation.z = sin(self.theta/2)
        msg.pose.pose.orientation.w = cos(self.theta/2)
        msg.twist.twist.linear.x    = 0.0
        msg.twist.twist.linear.y    = 0.0
        msg.twist.twist.angular.z   = 0.0
        self.pubodom.publish(msg)

        # Create the transform and broadcast (reuse the time stamp).
        trans = TransformStamped()
        trans.header.stamp            = timestamp
        trans.header.frame_id         = 'odom'
        trans.child_frame_id          = 'world'
        trans.transform.translation.x = self.x
        trans.transform.translation.y = self.y
        trans.transform.rotation.z    = sin(self.theta/2)
        trans.transform.rotation.w    = cos(self.theta/2)
        self.tfbroadcaster.sendTransform(trans)


    # Velocity Command Message Callback
    def cb_vcmdmsg(self, msg):
        # Grab the forward and spin (velocity) commands.
        vxcmd = msg.linear.x
        vycmd = msg.linear.y
        wzcmd = msg.angular.z
        
        psi_dot = []
        for dist in DISTANCES:
            v_x = vxcmd - dist[1]*wzcmd
            v_y = vycmd + dist[0]*wzcmd
            psi_dot.append(sqrt(v_x**2+v_y**2)/R)

        # Create the wheel command msg and publish.  Note the incoming
        # message does not have a time stamp, so generate one here.
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = [LFNAME, RFNAME, LBNAME, RBNAME]
        msg.velocity     = psi_dot
        self.pubwcmd.publish(msg)

        self.simulate_drive(vxcmd, vycmd, wzcmd, msg.header.stamp)


    # Wheel State Message Callback
    def cb_wheelmsg(self, msg):
        # Grab the timestamp, wheel and gyro position/velocities.
        try:
            timestamp = msg.header.stamp
            lfpsi      = msg.position[msg.name.index('front_left_wheel')]
            lfpsi_dot  = msg.velocity[msg.name.index('front_left_wheel')]
            rfpsi      = msg.position[msg.name.index('front_right_wheel')]
            rfpsi_dot  = msg.velocity[msg.name.index('front_right_wheel')]
            lbpsi      = msg.position[msg.name.index('rear_left_wheel')]
            lbpsi_dot  = msg.velocity[msg.name.index('rear_left_wheel')]
            rbpsi      = msg.position[msg.name.index('rear_right_wheel')]
            rbpsi_dot  = msg.velocity[msg.name.index('rear_right_wheel')]
            #theta     = msg.position[msg.name.index('gyro')]
            #omega     = msg.velocity[msg.name.index('gyro')]
        except:
            self.get_logger().error("Ill-formed /wheel_state message!")
            return

        dpsi_l = lfpsi - self.lpsi
        dpsi_r = rfpsi - self.rpsi
        self.lpsi = lfpsi
        self.rpsi = rfpsi

        dp = 0.5*R*(dpsi_l + dpsi_r)
        dtheta = (0.5*R/d)*(-dpsi_l + dpsi_r)

        self.x += dp*cos(self.theta + dtheta/2)*np.sinc(dtheta/(2*pi))
        self.y += dp*sin(self.theta + dtheta/2)*np.sinc(dtheta/(2*pi))
        self.theta += dtheta
        #self.theta = theta

        vxact = (lfpsi_dot + rfpsi_dot)*(0.5*R)
        wzact = (-lfpsi_dot + rfpsi_dot)*(0.5*R/d)
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

    def simulate_drive(self, vxcmd, vycmd, wzcmd, timestamp):
        if not self.last_now:
            self.last_now = self.get_clock().now()
        now = self.get_clock().now()
        dt  = (now - self.last_now).nanoseconds * 1e-9
        self.last_now = now

        dtheta = wzcmd*dt
        self.x += vxcmd*dt*cos(self.theta + dtheta/2)*np.sinc(dtheta/(2*pi))
        self.x -= vycmd*dt*sin(self.theta + dtheta/2)*np.sinc(dtheta/(2*pi))
        self.y += vycmd*dt*cos(self.theta + dtheta/2)*np.sinc(dtheta/(2*pi))
        self.y += vxcmd*dt*sin(self.theta + dtheta/2)*np.sinc(dtheta/(2*pi))
        self.theta += dtheta

        # Create the odometry msg and publish (reuse the time stamp).
        msg = Odometry()
        msg.header.stamp            = timestamp
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'world'
        msg.pose.pose.position.x    = self.x
        msg.pose.pose.position.y    = self.y
        msg.pose.pose.orientation.z = sin(self.theta/2)
        msg.pose.pose.orientation.w = cos(self.theta/2)
        msg.twist.twist.linear.x    = vxcmd*cos(self.theta) - vycmd*sin(self.theta)
        msg.twist.twist.linear.y    = vycmd*cos(self.theta) + vxcmd*sin(self.theta)
        msg.twist.twist.angular.z   = wzcmd
        self.pubodom.publish(msg)

        # Create the transform and broadcast (reuse the time stamp).
        trans = TransformStamped()
        trans.header.stamp            = timestamp
        trans.header.frame_id         = 'odom'
        trans.child_frame_id          = 'world'
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
