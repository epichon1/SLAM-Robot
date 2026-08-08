import rclpy
import traceback
import tf2_ros

from math import *
import numpy as np
from numpy import radians

from rclpy.node                 import Node
from tf2_ros                    import TransformBroadcaster
from geometry_msgs.msg          import Point, Quaternion, Twist
from geometry_msgs.msg          import TransformStamped, Vector3
from geometry_msgs.msg          import PoseStamped
from nav_msgs.msg               import Odometry, Path
from sensor_msgs.msg            import LaserScan
from sensor_msgs.msg            import JointState
from scipy.spatial.transform    import Rotation
from rclpy.time                 import Time, Duration
from shared169.planartransform  import PlanarTransform
from rclpy.executors            import MultiThreadedExecutor
from rclpy.callback_groups      import MutuallyExclusiveCallbackGroup
from std_msgs.msg               import Bool


TP = 2.0
TD = 1.7
T_SPIN = 0.7
OMEGA_MAX = 1.0
V_MAX = 1.5
THETA_MARGIN = 0.2
D_MARGIN = 0.07

# Lidar constants
RANGE_MIN = 0.15
RANGE_MAX = 12.0

MAX_BACK_MARGIN = 0.50
MAX_FRONT_MARGIN = 0.50
MAX_SIDE_MARGIN = 0.22

MIN_BACK_MARGIN = 0.20
MIN_FRONT_MARGIN = 0.20
MIN_SIDE_MARGIN = 0.17

K_F = (MAX_FRONT_MARGIN - MIN_FRONT_MARGIN) / V_MAX
K_B = (MAX_BACK_MARGIN - MIN_BACK_MARGIN) / V_MAX
K_S = (MAX_SIDE_MARGIN - MIN_SIDE_MARGIN) / V_MAX


def wrap(theta):
    return theta - 2*pi*round(theta/(2*pi))


def sat(x, limit):
    return min(max(x, -limit), limit)


def quat_to_euler(q):
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=False)


#Autodrive Node
class AutoDriveNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        fastgroup = MutuallyExclusiveCallbackGroup()
        slowgroup = MutuallyExclusiveCallbackGroup()

        #Create Listener
        self.tfBuffer = tf2_ros.Buffer()
        tflisten      = tf2_ros.TransformListener(
            self.tfBuffer, self, spin_thread=True)

        # Set the initial pose to zero.
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.theta = 0.0
        self.lastnow = self.get_clock().now()
        self.lastd = 0.0
        
        self.waypoints = []

        self.detections = {"b": False, "r": False, "f": False, "l": False, "fl": False, "fr": False}
        # Create the publishers for the wheel commands and the
        # odometry information.
        self.pubvcmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pubbool = self.create_publisher(Bool, '/bool', 10)
        self.pubbool_goal = self.create_publisher(Bool, '/pursuing_goal', 10)
        self.pubpose = self.create_publisher(PoseStamped, '/coverage_pose', 10)
        # Initialize the transform broadcaster
        self.tfbroadcaster = TransformBroadcaster(self)

        # Create the subscribers to listen to wheel state and twist
        # comamnds.
        self.subodom = self.create_subscription(
            Odometry, '/odom', self.cb_odommsg, 10, callback_group=fastgroup)
        self.subgpose = self.create_subscription(
            PoseStamped, '/goal_pose', self.cb_goalpose, 10, callback_group=fastgroup)
        self.sublidar = self.create_subscription(
            LaserScan, '/scan', self.cb_scanmsg, 1, callback_group=slowgroup)
        self.subpath = self.create_subscription(
            Path, '/path', self.cb_pathmsg, 10, callback_group=fastgroup)

        # Report and return.
        self.get_logger().info("Autodrive running")


    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()


    def cb_odommsg(self, msg):
        now = self.get_clock().now()
        dt  = (now - self.lastnow).nanoseconds * 1e-9
        self.lastnow = now

        logger = self.get_logger()
        spinning = False
        stuck = False
        pursuing_goal = True

        try:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.vx = msg.twist.twist.linear.x
            self.vy = msg.twist.twist.linear.y
            self.wz = msg.twist.twist.angular.z
            q = msg.pose.pose.orientation
            (_, _, self.theta) = quat_to_euler(q)
        except:
            logger.error("Ill-formed /odom message!")
            return            

        if not self.waypoints:
            omega_cmd = 0.0
            vx_cmd = 0.0
            vy_cmd = 0.0
            pursuing_goal = False
        else: 
            try:
                msgframe = self.waypoints[0][1]
                tfodom   = 'odom'    
                tfchild  = msgframe
                tftime   = Time()       # Leave unspecified (defaulting to zero)
                try:
                    tfmsg = self.tfBuffer.lookup_transform(tfodom, tfchild, tftime)
                except tf2_ros.TransformException as ex:
                    # Warn if unable to get and abort the callback.
                    self.get_logger().warn("Callback unable to get TF '%s' to '%s'" % (tfodom, tfchild))
                    self.get_logger().warn("Exception: %s" % str(ex))
                    return

                # For ease of processing, convert into a planar transform
                odom2child = PlanarTransform.fromTransform(tfmsg.transform)
                child2goal = self.waypoints[0][0]
                odom2goal = odom2child * child2goal

            except BaseException as e:
                self.get_logger().info(e)
                self.get_logger().error("Ill-formed /goal_pose message!")
                return

            (xd, yd, thetad) = (odom2goal.x(),odom2goal.y(),odom2goal.theta())
            theta_target = np.arctan2(yd - self.y, xd - self.x)
            theta_err = wrap(theta_target - self.theta) 

            d = np.sqrt((xd - self.x)**2 + (yd - self.y)**2)
            d_err = abs(d - self.lastd)/dt
            self.lastd = d

            omega_cmd = sat(theta_err/T_SPIN, OMEGA_MAX)
            vx_cmd = sat(d/TP + d_err/TD, V_MAX)
            vy_cmd = 0.0

            if abs(theta_err) > THETA_MARGIN:
                vx_cmd = 0.0
                spinning = True

            if d < D_MARGIN: 
                vx_cmd = 0.0
                theta_err_final = wrap(thetad - self.theta)
                omega_cmd = sat(theta_err_final/T_SPIN, OMEGA_MAX)
                spinning = True
                if abs(theta_err_final) < THETA_MARGIN:
                    omega_cmd = 0.0
                    (waypoint_pose, waypoint_frame) = self.waypoints.pop(0)
                    msg = PoseStamped()
                    msg.header.frame_id = waypoint_frame
                    msg.pose = waypoint_pose.toPose()
                    self.pubpose.publish(msg)

        if (vx_cmd > 0 and self.detections['f']) or (vx_cmd < 0 and self.detections['b']):
            vx_cmd = 0.0
            if (omega_cmd > 0 and self.detections['fl']) or (omega_cmd < 0 and self.detections['fr']):
                omega_cmd = 0.0
            if not spinning:
                stuck = True
        
        if (omega_cmd > 0 and self.detections['l']) or (omega_cmd < 0 and self.detections['r']):
            omega_cmd = 0.0
            if spinning:
                stuck = True
        
        msg = Twist()
        msg.linear.x = vx_cmd
        msg.angular.z = omega_cmd
        msg.linear.y = vy_cmd
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        self.pubvcmd.publish(msg)

        bool_msg = Bool()
        bool_msg.data = stuck
        self.pubbool.publish(bool_msg)

        bool_msg = Bool()
        bool_msg.data = pursuing_goal
        self.pubbool_goal.publish(bool_msg)


    def detect_objects(self, ranges, angle_min, angle_max, angle_increment):
        detections = {"b": False, "r": False, "f": False, "l": False, "fl": False, "fr": False}
        n = (angle_min - angle_max) / (2*pi)

        for i, r in enumerate(ranges): 
            if isinf(r) or isnan(r):
                continue
            
            theta = angle_min + i*angle_increment
            x = abs(r*cos(theta))
            y = abs(r*sin(theta))

            if radians(160)*n <= abs(theta) <= radians(180)*n and not detections['b']:
                detections['b'] = x <= (MIN_BACK_MARGIN - self.vx*K_B)

            if -radians(22.5)*n <= theta <= radians(22.5)*n and not detections['f']:
                detections['f'] = x <= (MIN_FRONT_MARGIN + self.vx*K_F)

            if radians(80)*n <= theta <= radians(125)*n and not detections['l']:
                detections['l'] = y <= (MIN_SIDE_MARGIN - self.wz*K_S)
            
            if radians(48)*n <= theta <= radians(80)*n and not detections['fl']:
                detections['fl'] = y <= MIN_SIDE_MARGIN

            if radians(22.5)*n <= theta <= radians(48)*n and not detections['fl']:
                detections['fl'] = x <= MIN_FRONT_MARGIN

            if -radians(125)*n <= theta <= -radians(80)*n and not detections['r']:
                detections['r'] = y <= (MIN_SIDE_MARGIN + self.wz*K_S)

            if -radians(48)*n <= theta <= -radians(22.5)*n and not detections['fr']:
                detections['fr'] = x <= MIN_FRONT_MARGIN

            if radians(48)*n <= theta <= radians(80)*n and not detections['fr']:
                detections['fr'] = y <= MIN_SIDE_MARGIN

        self.detections = detections



    def cb_goalpose(self, msg):
        try:
            self.waypoints = [(PlanarTransform.fromPose(msg.pose), msg.header.frame_id)]
        except BaseException as e:
            self.get_logger().info(e)
            return


    def cb_pathmsg(self, msg):
        try:
            self.waypoints = []
            for pose in msg.poses:
                self.waypoints.append((PlanarTransform.fromPose(pose.pose), msg.header.frame_id))
        except BaseException as e:
            self.get_logger().info(e)
            return


    def cb_scanmsg(self, msg):
        self.detect_objects(msg.ranges, msg.angle_min, msg.angle_max, msg.angle_increment)
        # self.get_logger().info(f"{msg.angle_min}, {msg.angle_max}, {msg.angle_increment}, {len(msg.ranges)}")
        # self.get_logger().info(f"f: {self.detections['f']}    b: {self.detections['b']}    r: {self.detections['r']}    l: {self.detections['l']}    fl: {self.detections['fl']}    fr: {self.detections['fr']}")


def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = AutoDriveNode('autodrive')

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # Spin the node until interrupted.
    try:
        executor.spin()
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
