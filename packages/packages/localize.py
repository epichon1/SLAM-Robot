import rclpy
import traceback
import tf2_ros

from math import *
import numpy as np
from numpy import radians

from rclpy.node                 import Node
from tf2_ros                    import TransformBroadcaster
from geometry_msgs.msg          import Point, Pose, Quaternion, Twist
from geometry_msgs.msg          import TransformStamped, Vector3
from geometry_msgs.msg          import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg               import Odometry
from sensor_msgs.msg            import LaserScan
from sensor_msgs.msg            import JointState
from scipy.spatial.transform    import Rotation
from rclpy.time                 import Time, Duration
from shared169.planartransform  import PlanarTransform
from rclpy.executors            import MultiThreadedExecutor
from rclpy.callback_groups      import MutuallyExclusiveCallbackGroup
from nav_msgs.msg               import OccupancyGrid
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker, MarkerArray


#Constants
WALL_THRESHOLD = 0.8
UPDATE_FRACTION = 0.3
SPIN_UPDATE_FRACTION = 0.001
INIT_UPDATE_FRACTION = 0.8
DISTANCE_THRESHOLD = 0.25
SAMPLE_STEP = 2
OMEGA_TOLERANCE = 0.04
TIME_TOLERANCE = 5.0

HEIGHT = 200
WIDTH = 200


def wrap(theta):
    return theta - 2*pi*round(theta/(2*pi))


def sat(x, limit):
    return min(max(x, -limit), limit)


def quat_to_euler(q):
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=False)


class LocalizeNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        fastgroup = MutuallyExclusiveCallbackGroup()
        slowgroup = MutuallyExclusiveCallbackGroup()

        #Attributes
        self.wallpts = np.zeros((0,2), dtype=int)
        self.nearestwallpt_map = np.zeros((HEIGHT, WIDTH, 2))
        self.res = 0.0
        self.update_fraction = 0.0
        self.new_estimate = False
        self.omega = 0.0
        self.pose_estimate_recieve_time = Duration()
        

        #Create Listener
        self.tfBuffer = tf2_ros.Buffer()
        tflisten      = tf2_ros.TransformListener(
            self.tfBuffer, self, spin_thread=True)

        # Create the publishers for the wheel commands and the
        # odometry information.
        self.pubmarker = self.create_publisher(MarkerArray, '/visualization_marker_array', 1)
        self.pubpose = self.create_publisher(PoseStamped, '/pose', 10)

        # Initialize the transform broadcaster
        self.tfbroadcaster = TransformBroadcaster(self)

        self.map2odom = PlanarTransform.identity()
        self.odom2base = PlanarTransform.identity()
        self.map2grid = PlanarTransform.identity()

        # Create the subscribers to listen to wheel state and twist
        # comamnds.
        self.subodom = self.create_subscription(
            Odometry, '/odom', self.cb_odommsg, 10, callback_group=fastgroup)
        self.sublidar = self.create_subscription(
            LaserScan, '/scan', self.cb_scanmsg,  1, callback_group=slowgroup)
        self.subinitpose = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.cb_initposemsg,  1, callback_group=slowgroup)
        # Map subscriber
        self.statictfbroadcaster = StaticTransformBroadcaster(self)

        # Create a subscriber for the map data.  Note this topic uses
        # a quality of service with durability TRANSIENT_LOCAL
        # allowing new subscribers to get the last sent message.
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        self.create_subscription(
            OccupancyGrid, '/map', self.cb_mapmsg, quality)


    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()


    def cb_odommsg(self, msg):
        try:
            self.odom2base = PlanarTransform.fromPose(msg.pose.pose)
            self.map2base = self.map2odom * self.odom2base
            pose = self.map2base.toPose()
            posemsg = PoseStamped()
            posemsg.pose = pose 
            posemsg.header.stamp = msg.header.stamp
            posemsg.header.frame_id = 'map'
            self.pubpose.publish(posemsg)
            self.omega = msg.twist.twist.angular.z

            trans = TransformStamped()
            trans.header.stamp            = msg.header.stamp
            trans.header.frame_id         = 'map'
            trans.child_frame_id          = 'grid'
            trans.transform = self.map2grid.toTransform()
            self.tfbroadcaster.sendTransform(trans)

            trans = TransformStamped()
            trans.header.stamp            = msg.header.stamp
            trans.header.frame_id         = 'map'
            trans.child_frame_id          = 'odom'
            trans.transform = self.map2odom.toTransform()
            self.tfbroadcaster.sendTransform(trans)
        except:
            return


    def cb_initposemsg(self, msg):
        aframe = msg.header.frame_id
        try:
            tfmsg0 = self.tfBuffer.lookup_transform(aframe, 'map',Time())
            tfmsg  = self.tfBuffer.lookup_transform('base', 'odom', Time())
        except tf2_ros.TransformException as ex:
            # Warn if unable to get and abort the callback.
            self.get_logger().warn("Exception: %s" % str(ex))
            return

         # For ease of processing, convert into a planar transform
        a2map = PlanarTransform.fromTransform(tfmsg0.transform)
        base2odom = PlanarTransform.fromTransform(tfmsg.transform)
        a2base = PlanarTransform.fromPose(msg.pose.pose)
        map2odom = a2map.inv() * a2base * base2odom
        self.map2odom = map2odom
        tfmsg = TransformStamped()
        tfmsg.header.stamp    = msg.header.stamp
        tfmsg.header.frame_id = 'map'
        tfmsg.child_frame_id  = 'odom'
        tfmsg.transform       = map2odom.toTransform()
        self.statictfbroadcaster.sendTransform(tfmsg)
        self.new_estimate = True
        self.pose_estimate_recieve_time = self.get_clock().now()


    def cb_mapmsg(self, mapmsg):
        # Grab the map info.
        width    = mapmsg.info.width
        height   = mapmsg.info.height
        self.res = mapmsg.info.resolution
        map = np.array(mapmsg.data)
        map = map.reshape((height, width))
        
        self.map2grid = PlanarTransform.fromPose(mapmsg.info.origin)

        # Broadcast the static map2grid transform.
        tfmsg = TransformStamped()
        tfmsg.header.stamp    = mapmsg.header.stamp
        tfmsg.header.frame_id = 'map'
        tfmsg.child_frame_id  = 'grid'
        tfmsg.transform       = self.map2grid.toTransform()
        self.statictfbroadcaster.sendTransform(tfmsg)

        wallpts = np.zeros((0,2), dtype=int)
        for v in range(height):
            for u in range(width):
                if map[v,u] > WALL_THRESHOLD:
                    # Also check the adjacent pixels in a 3x3 grid.
                    adjacent = map[max(0,v-1):min(height,v+2), max(0,u-1):min(width,u+2)]
                    if not np.all(adjacent > WALL_THRESHOLD):
                        wallpts = np.vstack([wallpts, np.array([u, v])])

        self.nearestwallpt_map = np.zeros((height, width, 2))
        for v in range(height):
            for u in range(width):
                self.nearestwallpt_map[v, u] = self.nearestwallpt(wallpts, u, v)

        self.wallpts = self.res*wallpts.astype(float)

        self.pubpoints(self.wallpts)


    def cb_scanmsg(self, msg):
        try:
            timestamp = msg.header.stamp
            msgtime  = Time().from_msg(timestamp)
        except:
            self.get_logger().error("Ill-formed /scan message!")
            return
        
        tftime = msgtime
        try:
            tfmsg = self.tfBuffer.lookup_transform(
                'base', 'lidar', tftime)
            # tfmsg2 = self.tfBuffer.lookup_transform(
            #     'grid', 'map', tftime, timeout=Duration(seconds=0.2))
        except tf2_ros.TransformException as ex:
            # Warn if unable to get and abort the callback.
            self.get_logger().warn("Callback unable to get TF '%s' to '%s'" %
                                   ('odom', 'lidar'))
            self.get_logger().warn("Exception: %s" % str(ex))
            return

        base2lidar = PlanarTransform.fromTransform(tfmsg.transform)
        grid2map = self.map2grid.inv()   #PlanarTransform.fromTransform(tfmsg2.transform)
        grid2lidar = grid2map * self.map2odom * self.odom2base * base2lidar
        (scanpts, nearestpts, delta) = self.localize_update(grid2lidar, msg.ranges, msg.angle_min, msg.angle_increment)
        self.set_update_fraction()
        delta = delta.scale(self.update_fraction)
       
        self.map2odom = grid2map.inv() * delta * grid2map * self.map2odom

        trans = TransformStamped()
        trans.header.stamp            = timestamp
        trans.header.frame_id         = 'map'
        trans.child_frame_id          = 'odom'
        trans.transform = self.map2odom.toTransform()
        self.tfbroadcaster.sendTransform(trans)
        self.pubpoints(self.wallpts)
        self.publines(scanpts, nearestpts)


    # Utility to publish a list of (x,y) points.
    def pubpoints(self, points, frame = 'grid', timestamp = None):
        # Grab a timestamp, if none was passed.
        if timestamp is None:
            timestamp = self.get_clock().now().to_msg()

        # Create the marker message.
        markermsg = Marker()
        markermsg.header.stamp    = timestamp
        markermsg.header.frame_id = frame
        # YOU CAN USE ANY FRAME AS REFERENCE!  If your computation is
        # in 'grid' frame, select that!!  AND make sure you have
        # broadcast the static TF (mapsubscriber_sample.py).

        markermsg.ns     = 'points'         # Choose an appropriate namespace
        markermsg.id     = 0                # Overwrite data with the same ID

        markermsg.action = Marker.ADD
        markermsg.type   = Marker.POINTS
        markermsg.pose   = Pose()
        markermsg.scale  = Vector3(x=0.03, y=0.03, z=0.03)
        markermsg.color  = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)

        markermsg.points = []   # For POINTS, draw points at p0, p1, p2, etc.

        for pt in points:
            markermsg.points.append(Point(x=pt[0], y=pt[1], z=0.0))

        # Add the marker message to a marker array (could add more).
        markerarraymsg = MarkerArray()
        markerarraymsg.markers.append(markermsg)

        # Send the marker array.
        self.pubmarker.publish(markerarraymsg)
    

    def publines(self, points1, points2, frame = 'grid', timestamp = None):
        # Grab a timestamp, if none was passed.
        if timestamp is None:
            timestamp = self.get_clock().now().to_msg()

        # Create the marker message.
        markermsg = Marker()
        markermsg.header.stamp    = timestamp
        markermsg.header.frame_id = frame
        # YOU CAN USE ANY FRAME AS REFERENCE!  If your computation is
        # in 'grid' frame, select that!!  AND make sure you have
        # broadcast the static TF (mapsubscriber_sample.py).

        markermsg.ns     = 'lines'          # Choose an appropriate namespace
        markermsg.id     = 0                # Overwrite data with the same ID

        markermsg.action = Marker.ADD
        markermsg.type   = Marker.LINE_LIST
        markermsg.pose   = Pose()
        markermsg.scale  = Vector3(x=0.03, y=0.03, z=0.03)
        markermsg.color  = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        markermsg.points = []   # For LINE_LIST, connect p0-p1, p2-p3, etc.

        for (pt1,pt2) in zip(points1, points2, strict=True):
            markermsg.points.append(Point(x=pt1[0], y=pt1[1], z=0.0))
            markermsg.points.append(Point(x=pt2[0], y=pt2[1], z=0.0))

        # Add the marker message to a marker array (could add more).
        markerarraymsg = MarkerArray()
        markerarraymsg.markers.append(markermsg)

        # Send the marker array.
        self.pubmarker.publish(markerarraymsg) 


    def nearestwallpt(self, wallpts, u, v):
        return wallpts[np.argmin(np.sum((np.array([u,v]) - wallpts)**2, axis=1))]


    def localize_update(self, grid2lidar, ranges_list, angle_min, angle_increment):
        h = HEIGHT
        w = WIDTH
        res = self.res
        ranges = np.array(ranges_list)
        indices = np.arange(len(ranges))
        mask = (indices % SAMPLE_STEP == 0) & (~np.isinf(ranges)) & (~np.isnan(ranges))
        indices = indices[mask]
        r = ranges[mask]
        alpha = angle_min + indices*angle_increment

        xl = r*np.cos(alpha)
        yl = r*np.sin(alpha)

        grid_coords = np.array([grid2lidar.inParent(x, y) for x,y in zip(xl, yl)])
        xg = grid_coords[:, 0]
        yg = grid_coords[:, 1]

        in_bounds = (0 <= xg) & (xg <= res*(w-1)) & (0 <= yg) & (yg <= res*(h-1))
        xg = xg[in_bounds]
        yg = yg[in_bounds]

        ug = np.round(xg/res).astype(int)
        vg = np.round(yg/res).astype(int)

        nearest = self.nearestwallpt_map[vg, ug]
        un = nearest[:, 0]
        vn = nearest[:, 1]
        xw = res*un
        yw = res*vn

        dists = np.sqrt((xg - xw)**2 + (yg - yw)**2)
        valid_pts = dists < DISTANCE_THRESHOLD
        xg = xg[valid_pts]
        yg = yg[valid_pts]
        xw = xw[valid_pts]
        yw = yw[valid_pts]
        scanpts = list(zip(xg, yg))
        nearestpts = list(zip(xw, yw))

        n = len(xg)
        if n == 0:
            return scanpts, nearestpts, PlanarTransform.unity()

        (rx, ry) = (np.mean(xg), np.mean(yg))
        (px, py) = (np.mean(xw), np.mean(yw))
        rr = np.mean(xg**2 + yg**2)
        rp = np.mean(xg*yw - yg*xw)

        denom = rr - (rx**2 + ry**2)

        if denom < 1e-6:
            delta = PlanarTransform.unity()
        else:
            try:
                dtheta = (rp - (rx*py - ry*px))/denom
                dx = px - rx + ry*dtheta
                dy = py - ry - rx*dtheta
                delta = PlanarTransform.basic(dx, dy, dtheta)
            except Exception as e:
                self.get_logger().info(f'{denom}')
                delta = PlanarTransform.unity()

        return scanpts, nearestpts, delta


    def pointpairs(self, grid2lidar, ranges, angle_min, angle_increment):
        scanpts = []
        nearestpts = []
        l = 200.0
        rx = ry = px = py = rr = rp = n = 0.0
        dtheta = dx = dy = 0.0
        for i, r in enumerate(ranges): 
            if i % SAMPLE_STEP != 0 or isinf(r) or isnan(r):
                continue 
            alpha = angle_min + i*angle_increment
            (xl, yl) = (r*cos(alpha), r*sin(alpha))
            (xg, yg) = grid2lidar.inParent(xl, yl)
            if 0 <= xg <= self.res*(l-1) and 0 <= yg <= self.res*(l-1):
                ug = round(xg/self.res)
                vg = round(yg/self.res)
                (un, vn) = self.nearestwallpt_map[vg,ug]
                (xw, yw) = (self.res*un, self.res*vn)
                d = sqrt((xg - xw)**2 + (yg - yw)**2)
                if d < DISTANCE_THRESHOLD:
                    scanpts.append((xg, yg))
                    nearestpts.append((xw, yw))
                    rx += xg 
                    ry += yg
                    px += xw
                    py += yw
                    rr += xg**2 + yg**2 
                    rp += xg*yw - yg*xw
                    n += 1.0
        if n != 0:
            rx *= 1/n
            ry *= 1/n
            px *= 1/n
            py *= 1/n
            rr *= 1/n
            rp *= 1/n
            denom = rr - (rx**2 + ry**2)
            if denom < 10e-6:   
                delta = PlanarTransform.unity()
            else: 
                try:
                    dtheta = (rp - (rx*py - ry*px))/denom
                    dx = px - rx + ry*dtheta
                    dy = py - ry - rx*dtheta
                    delta = PlanarTransform.basic(dx, dy, dtheta)
                except:
                    self.get_logger().info(f' {denom}')
        else:
            delta = PlanarTransform.unity()
        return scanpts, nearestpts, delta
    
    
    def set_update_fraction(self):
        time_elapsed = (self.get_clock().now() - self.pose_estimate_recieve_time).nanoseconds * 1e-9
        if time_elapsed > TIME_TOLERANCE:   
            self.new_estimate = False

        if self.omega > OMEGA_TOLERANCE:
            self.update_fraction = SPIN_UPDATE_FRACTION
        elif self.new_estimate:
            self.update_fraction = INIT_UPDATE_FRACTION
        else:
            self.update_fraction = UPDATE_FRACTION


        

def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = LocalizeNode('localize')

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
