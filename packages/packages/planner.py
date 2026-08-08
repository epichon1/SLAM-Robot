import rclpy
import traceback
import tf2_ros

from math import *
import numpy as np
from numpy import radians
import imutils
import random

from rclpy.node                 import Node
from tf2_ros                    import TransformBroadcaster
from geometry_msgs.msg          import Point, Pose, Quaternion, Twist
from geometry_msgs.msg          import TransformStamped, Vector3, PointStamped
from geometry_msgs.msg          import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg               import Odometry
from sensor_msgs.msg            import LaserScan
from sensor_msgs.msg            import JointState
from scipy.spatial.transform    import Rotation
from rclpy.time                 import Time, Duration
from shared169.planartransform  import PlanarTransform
from rclpy.executors            import MultiThreadedExecutor
from rclpy.callback_groups      import MutuallyExclusiveCallbackGroup
from nav_msgs.msg               import OccupancyGrid, Path
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from std_msgs.msg               import ColorRGBA, Bool 
from visualization_msgs.msg     import Marker, MarkerArray
from pacman_msgs.msg            import GameState, PowerPelletInfo, PointWithIDStamped

DELTA = 9
BLOB_DELTA = 9

THETA_SPACE = 0.5
DSTEP = 0.125
D_SPACE = DSTEP / 8

X_MIN = Y_MIN = 0.0
RES = 0.200
WIDTH = 200
HEIGHT = 200
X_MAX = WIDTH * RES
Y_MAX = HEIGHT * RES

S_MAX = int(10e4)
N_MAX = int(10e3)
REPLAN_FRACTION = 1/30

WALL_THRESHOLD = 6
OBSTACLE_THRESHOLD = 30
DISTANCE_THRESHOLD = 0.1
SAMPLE_STEP = 10
OCCUPANCY_INCREMENT = 5
COVERAGE_RADIUS = 15

BLINKY_ID = 94
BLINKY_BUFFER_DIST = 0.1
BOT_BUFFER_DIST = 0.3

ALPHA = 0.1

# ROB = [
#     '                   ',
#     '                   ',
#     '                   ',
#     '                   ',
#     '      ######       ',
#     '      ########     ',
#     '      #xxxxx####   ',
#     '      #xxxxxxxx##  ',
#     '      #xxxxxxxxx## ',
#     '      #xx0xxxxxx@# ',
#     '      #xxxxxxxxx## ',
#     '      #xxxxxxxx##  ',
#     '      #xxxxx####   ',
#     '      ########     ',
#     '      #####        ',
#     '                   ',
#     '                   ',
#     '                   ',
#     '                   ',
# ]

ROB = [
    '                   ',
    '                   ',
    '                   ',
    '                   ',
    '                   ',
    '###################',
    '#######xxxxx#######',
    '#######xxxxxxxx####',
    '#######xxxxxxxxx###',
    '#######xx0xxxxxx@##',
    '#######xxxxxxxxx###',
    '#######xxxxxxxx####',
    '#######xxxxx#######',
    '###################',
    '                   ',
    '                   ',
    '                   ',
    '                   ',
    '                   ',
]

# ROB = [
#     '                   ',
#     '                   ',
#     '                   ',
#     '      ######       ',
#     '      #######      ',
#     '      #########    ',
#     '      #xxxxx#####  ',
#     '      #xxxxxxxx####',
#     '      #xxxxxxxxx###',
#     '      #xx0xxxxxx@##',
#     '      #xxxxxxxxx###',
#     '      #xxxxxxxx####',
#     '      #xxxxx#####  ',
#     '      #########    ',
#     '      #######      ',
#     '                   ',
#     '                   ',
#     '                   ',
#     '                   ',
# ]
BLOB = [ 
    '                   ',  
    '###################',
    '###################',
    '###################',
    '###################',
    '###################',
    '###################',
    '###################',
    '###################',
    '###################',
    '###################',
    '###################',
    '###################',
    '###################',
    '###################',
    '###################',
    '###################',
    '                   ',
]


def generate_circle_mask(r, type=bool):
    diameter = 2 * r + 1
    mask = []

    for y in range(diameter):
        row = ''
        for x in range(diameter):
            dx = x - r
            dy = y - r
            if dx * dx + dy * dy <= r * r:
                row += '#'
            else:
                row += ' '
        mask.append(row)
    
    return np.array([[(cell != ' ') for cell in row] for row in mask], dtype=type)


ROBOT = np.array([[(cell != ' ') for cell in row] for row in ROB], dtype="uint8")
ROUND_BOT = generate_circle_mask(BLOB_DELTA, type="uint8")
COVERAGE_MASK = generate_circle_mask(COVERAGE_RADIUS)
assert np.size(ROBOT,0) == np.size(ROBOT,1), "Mask must be square"
assert np.size(ROBOT,0) % 2, "Mask must be odd-sized"


def vandercorput(fraction):
    # Recursively resort a list: first the middle element, then
    # alternating element from the top and bottom halves, where each
    # half has been individually resorted.
    def resort(list):
        k = int(len(list)/2)
        if k > 0:
            top = resort(list[:k])
            bot = resort(list[k+1:])
            list[0] = list[k]
            list[1::2] = top
            list[2::2] = bot
        return list

    # Return the resorted list of 1/n ... (n-1)/n
    n = ceil(1.0/fraction)
    return resort([i/n for i in range(1,n)])


def post_process(path, mapdata):
    i = 1
    while True:
        if(i+1 >= len(path)):
            return path
        if(path[i-1].connectsTo(path[i+1], mapdata)):
            path.pop(i)
        else:
            i += 1


#Planner Node Class
class PlannerNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)
        self.mapdata = np.zeros((HEIGHT, WIDTH), dtype=np.int8)
        self.obstacle_map = np.zeros((HEIGHT, WIDTH), dtype=np.int8)
        self.coverage_map = np.zeros((HEIGHT, WIDTH), dtype=np.int8)
        self.res = RES
        self.width = WIDTH
        self.height = HEIGHT
        self.replan_index = 0.0
        self.scan_count = 0
        self.last_gn = None
        self.last_waypoint = None
        self.replan = False
        self.replan_count = 0
        self.pursuing_goal = False
        self.start = False
        self.start_timer = None
        self.plan_timer = None
        self.fail_count = 0
        self.start_time = None
        self.last_pp_pose = None
        self.new_pp = False
        self.planning = False
        self.last_path = []
        self.map2grid = PlanarTransform.identity()

        #Create Listener
        self.tfBuffer = tf2_ros.Buffer()
        tflisten      = tf2_ros.TransformListener(
            self.tfBuffer, self, spin_thread=True)
        # Create the publishers
        self.pubpath = self.create_publisher(Path, '/path', 10)
        # Initialize the transform broadcaster
        self.tfbroadcaster = TransformBroadcaster(self)

        # Create the subscribers
        self.subcoverage_pose = self.create_subscription(
            PoseStamped, '/coverage_pose', self.cb_coverage_pose, 10)

        self.subaruco_obstacles = self.create_subscription(
            PointWithIDStamped, '/aruco_obstacles', self.cb_aruco_obs, 10)
        
        self.subplannerpose = self.create_subscription(
            PoseStamped, '/planner_goal_pose', self.cb_plannergoalpose, 10)
    
        self.subinitpose = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.cb_initposemsg,  1)
        # Map subscriber
        self.statictfbroadcaster = StaticTransformBroadcaster(self)

        self.subbool = self.create_subscription(
            Bool, '/bool', self.cb_stuck, 10)
        
        self.subbool_goal = self.create_subscription(
            Bool, '/pursuing_goal', self.cb_currgoal, 10)
        
        self.subgame_state = self.create_subscription(
            GameState, '/pacman_info/game_state', self.cb_gamestate, 1)
        
        self.subpp_info = self.create_subscription(
            PowerPelletInfo,  '/pacman_info/power_pellet_info', self.cb_pp_info, 1)
        
        self.sublidar = self.create_subscription(LaserScan, '/scan', self.cb_scanmsg, 1)
        
        self.subodom = self.create_subscription(
            Odometry, '/odom', self.cb_odommsg, 10)
        # Create a subscriber for the map data.  Note this topic uses
        # a quality of service with durability TRANSIENT_LOCAL
        # allowing new subscribers to get the last sent message.
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        self.create_subscription(
            OccupancyGrid, '/map', self.cb_mapmsg, quality)

        # Report and return.
        self.pubmap = self.create_publisher(OccupancyGrid, '/obstacle_map', quality)
        self.pubcoverage_map = self.create_publisher(OccupancyGrid, '/coverage_map', quality)
        self.get_logger().info("Planner running")


    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()


    def brain(self):
        #self.get_logger().info("In brain")
        #self.get_logger().info(f'{self.planning}')
        if self.start and not self.planning and (not self.pursuing_goal or self.replan_count > 3 or self.new_pp):
            self.planning = True
            #self.get_logger().info(f'{self.pursuing_goal} , {self.replan_count}, {self.new_pp}')
            try:
                tfmsg = self.tfBuffer.lookup_transform('grid', 'base', Time(), timeout=Duration(seconds=0.2))
            except tf2_ros.TransformException as ex:
                # Warn if unable to get and abort the callback.
                self.get_logger().warn("Exception: %s" % str(ex))
                self.planning = False
                return

            # For ease of processing, convert into a planar transform
            start_grid = PlanarTransform.fromTransform(tfmsg.transform)
            sn = RrtNode(start_grid.x(), start_grid.y(), start_grid.theta())
            d = (DELTA+1) * RES
            if self.new_pp:
                try:
                    tfmsg = self.tfBuffer.lookup_transform('grid', self.last_pp_pose.header.frame_id, Time(), timeout=Duration(seconds=0.2))
                except tf2_ros.TransformException as ex:
                    # Warn if unable to get and abort the callback.
                    self.get_logger().warn("Exception: %s" % str(ex))
                    self.planning = False
                    return
                grid2a = PlanarTransform.fromTransform(tfmsg.transform)
                (x,y) = grid2a.inParent(self.last_pp_pose.point.x, self.last_pp_pose.point.y)
                self.get_logger().info(f'pellet pose {x}, {y}')
                gn = RrtNode(x, y, 0.0)
                self.new_pp = False
            else:
                gn = RrtNode(random.uniform(X_MIN+d, X_MAX-d), random.uniform(Y_MIN+d, Y_MAX-d), random.uniform(-pi, pi))
                if not gn.is_free(self.coverage_map):
                    self.planning = False
                    return
            if gn.is_free(self.obstacle_map):
                self.last_gn = gn
                self.plan(sn, self.get_clock().now().to_msg())

            self.plan_timer = self.get_clock().now()
            self.planning = False

            
    def cb_odommsg(self, msg):
        if (self.start_timer) and ((self.get_clock().now() - self.start_timer).nanoseconds * 1e-9 > 5.0):
            self.start = True
        if (self.plan_timer) and ((self.get_clock().now() - self.plan_timer).nanoseconds * 1e-9 > 0.5):
            self.planning = False
            self.plan_timer = None
            self.get_logger().info('timer reset')
        if self.fail_count > 2:
            self.obstacle_map = self.mapdata.copy()
            self.fail_count = 0
        if np.count_nonzero(self.coverage_map == 0)/np.count_nonzero(self.mapdata == 0) < 0.3:
            self.coverage_map = self.mapdata.copy()
        self.brain()


    def cb_currgoal(self,msg):
        #self.get_logger().info(f'goal update {msg.data}')
        self.pursuing_goal = msg.data


    # Velocity Command Message Callback
    def cb_mapmsg(self, msg):
        self.get_logger().info("Recieved map")
        self.width    = msg.info.width
        self.height   = msg.info.height
        self.res = msg.info.resolution

        self.mapdata = np.array(msg.data).reshape((self.height, self.width))
        self.obstacle_map = self.mapdata.copy()
        self.coverage_map = self.mapdata.copy()
        self.map2grid = PlanarTransform.fromPose(msg.info.origin)

        wallpts = np.zeros((0,2), dtype=np.int8)
        for v in range(self.height):
            for u in range(self.width):
                if self.mapdata[v,u] > WALL_THRESHOLD:
                    adjacent = self.mapdata[max(0,v-1):min(self.height,v+2), max(0,u-1):min(self.width,u+2)]
                    if not np.all(adjacent > WALL_THRESHOLD):
                        wallpts = np.vstack([wallpts, np.array([u, v])])

        self.nearestwallpt_map = np.zeros((self.height, self.width, 2))
        for v in range(self.height):
            for u in range(self.width):
                self.nearestwallpt_map[v, u] = self.nearestwallpt(wallpts, u, v)


    # Wheel State Message Callback
    def cb_initposemsg(self, msg):
        aframe = msg.header.frame_id
        try:
            tfmsg = self.tfBuffer.lookup_transform('grid', aframe, Time())
        except tf2_ros.TransformException as ex:
            # Warn if unable to get and abort the callback.
            self.get_logger().warn("Exception: %s" % str(ex))
            return

         # For ease of processing, convert into a planar transform
        grid2a = PlanarTransform.fromTransform(tfmsg.transform)
        pt_grid = grid2a * PlanarTransform.fromPose(msg.pose.pose)
        self.get_logger().info(f'In free space:  {self.is_free(pt_grid)}')
        self.start_timer = self.get_clock().now()


    def cb_stuck(self, msg):
        update = float(msg.data)
        self.replan_index += REPLAN_FRACTION * (update - self.replan_index)
        if self.replan_index > 0.9:
            self.get_logger().info("stuck :(")
            self.replan = True
            try:
                tfmsg = self.tfBuffer.lookup_transform('grid', 'base', Time(), timeout=Duration(seconds=0.2))
            except tf2_ros.TransformException as ex:
                # Warn if unable to get and abort the callback.
                self.get_logger().warn("Exception: %s" % str(ex))
                return

            # For ease of processing, convert into a planar transform
            start_grid = PlanarTransform.fromTransform(tfmsg.transform)
            sn = RrtNode(start_grid.x(), start_grid.y(), start_grid.theta())
            if self.scan_count > 8:
                self.get_logger().info("replanning now")
                self.obstacle_map[self.obstacle_map >= OBSTACLE_THRESHOLD] = 100
                self.obstacle_map[(self.obstacle_map < OBSTACLE_THRESHOLD) & (self.obstacle_map >= 0)] = 0
                self.plan(sn, self.get_clock().now().to_msg())
                self.replan_index = 0.0
                self.scan_count = 0
                self.replan = False
                self.replan_count += 1

                flattened_obstacle_map = self.obstacle_map.flatten().tolist()
                mapmsg = OccupancyGrid()
                mapmsg.header.frame_id = 'map'
                mapmsg.info.width = self.width
                mapmsg.info.height = self.height
                mapmsg.info.resolution = self.res
                mapmsg.info.origin = self.map2grid.toPose()
                mapmsg.data = flattened_obstacle_map
                #self.get_logger().info(f"{flattened_obstacle_map}")
                self.pubmap.publish(mapmsg)
    

    def cb_scanmsg(self, msg):
        if not self.replan:
            return
        
        self.scan_count += 1
        # try:
        #     timestamp = msg.header.stamp
        #     msgtime  = Time().from_msg(timestamp)
        # except:
        #     self.get_logger().error("Ill-formed /scan message!")
        #     return
        
        tftime = Time()
        try:
            tfmsg = self.tfBuffer.lookup_transform(
                'grid', 'lidar', tftime, timeout=Duration(seconds=0.8))
        except tf2_ros.TransformException as ex:
            # Warn if unable to get and abort the callback.
            self.get_logger().warn("Callback unable to get TF '%s' to '%s'" %
                                   ('grid', 'lidar'))
            self.get_logger().warn("Exception: %s" % str(ex))
            return

        grid2lidar = PlanarTransform.fromTransform(tfmsg.transform)
        self.update_obstacles(grid2lidar, msg.ranges, msg.angle_min, msg.angle_increment)


    def cb_plannergoalpose(self, msg):
        aframe = msg.header.frame_id
        try:
            tfmsg = self.tfBuffer.lookup_transform('grid', aframe, Time(), timeout=Duration(seconds=0.5))
            tfmsg2 = self.tfBuffer.lookup_transform('grid', 'world', Time(), timeout=Duration(seconds=0.5))
        except tf2_ros.TransformException as ex:
            # Warn if unable to get and abort the callback.
            self.get_logger().warn("Exception: %s" % str(ex))
            return

         # For ease of processing, convert into a planar transform
        grid2a = PlanarTransform.fromTransform(tfmsg.transform)
        goal_grid = grid2a * PlanarTransform.fromPose(msg.pose)
        start_grid = PlanarTransform.fromTransform(tfmsg2.transform)
        sn = RrtNode(start_grid.x(), start_grid.y(), start_grid.theta())
        self.last_gn = RrtNode(goal_grid.x(), goal_grid.y(), goal_grid.theta())
        self.plan(sn, msg.header.stamp)
        

    def plan(self, sn, stamp):
        self.get_logger().info("planning")
        if self.last_gn == None:
            self.get_logger().info("no goal somehow")
            self.fail_count += 1
            return
        if not self.last_gn.is_free(self.obstacle_map,bit_mask=ROUND_BOT, delta=BLOB_DELTA):
            self.get_logger().info("Goalnode not free.")
            self.fail_count += 1
            return
        nodes = rrt(sn, self.last_gn, self.obstacle_map)
        if nodes == None:
            self.get_logger().info("Couldn't find path!!!")
            self.fail_count += 1
            return
        
        nodes = post_process(nodes, self.obstacle_map)
        self.replan_count = 0
        path = []
        for node in nodes:
            pt = PlanarTransform.basic(node.x, node.y, node.theta)
            pose = pt.toPose()
            posemsg = PoseStamped()
            posemsg.pose = pose 
            posemsg.header.stamp = stamp
            posemsg.header.frame_id = 'grid'
            path.append(posemsg)

        pathmsg = Path()
        pathmsg.header.stamp = stamp
        pathmsg.header.frame_id = 'grid'
        pathmsg.poses = path
        self.pubpath.publish(pathmsg)
        self.planning = False
        self.obstacle_map = self.mapdata.copy()
        self.last_path = nodes

    

    def is_free(self, pt, bit_mask=ROBOT, delta= DELTA):
        (u, v) = (round(pt.x()/self.res), round(pt.y()/self.res))
        mask = imutils.rotate(bit_mask, angle = -180.0/pi * pt.theta())
        mapslice = self.obstacle_map[v-delta:v+1+delta, u-delta:u+1+delta]
        return not np.any(np.logical_and(mapslice, mask))
    
    
    def update_obstacles(self, grid2lidar, ranges, angle_min, angle_increment):
        for i, r in enumerate(ranges): 
            if i % SAMPLE_STEP != 0 or isinf(r) or isnan(r):
                continue 
            alpha = angle_min + i*angle_increment
            (xl, yl) = (r*cos(alpha), r*sin(alpha))
            (xg, yg) = grid2lidar.inParent(xl, yl)
            if 0 <= xg <= self.res*(WIDTH-1) and 0 <= yg <= self.res*(HEIGHT-1):
                ug = round(xg/self.res)
                vg = round(yg/self.res)
                (un, vn) = self.nearestwallpt_map[vg,ug]
                (xw, yw) = (self.res*un, self.res*vn)
                d = sqrt((xg - xw)**2 + (yg - yw)**2)
                if d >= DISTANCE_THRESHOLD and self.obstacle_map[vg,ug] != -1:
                    self.obstacle_map[vg,ug] += OCCUPANCY_INCREMENT
    

    def nearestwallpt(self, wallpts, u, v):
        return wallpts[np.argmin(np.sum((np.array([u,v]) - wallpts)**2, axis=1))]

    
    def cb_gamestate(self, msg):
        self.start_time = msg.game_start_time
    
    
    def cb_pp_info(self, msg):
        if self.last_pp_pose == None or self.last_pp_pose != msg.current_pose:
            self.get_logger().info("new powerpellet")
            self.last_pp_pose = msg.current_pose
            self.new_pp = True
            self.planning = False
    

    def cb_aruco_obs(self, msg):
        if not self.start:
            return
        (xc, yc) = (msg.point.x, msg.point.y)
        try:
            tfmsg = self.tfBuffer.lookup_transform('grid', msg.header.frame_id, Time(), timeout=Duration(seconds=0.2))
            tfmsg2 = self.tfBuffer.lookup_transform('grid', 'base', Time(), timeout=Duration(seconds=0.5))
        except tf2_ros.TransformException as ex:
            # Warn if unable to get and abort the callback.
            self.get_logger().warn("Exception: %s" % str(ex))
            return
        grid2camera = PlanarTransform.fromTransform(tfmsg.transform)
        (xg, yg) = grid2camera.inParent(xc, yc)
        (xb, yb) = (tfmsg2.transform.translation.x, tfmsg2.transform.translation.y)
        (dx, dy) = (xg - xb, yg - yb)
        d = sqrt(dx**2 + dy**2)
        #(xg, yg) = (0.1*dx/d, 0.1*dy/d)
        (u, v) = (round(xg/self.res), round(yg/self.res))
        mask_buffer_dist = BLINKY_BUFFER_DIST if msg.id == BLINKY_ID else BOT_BUFFER_DIST
        r = max(1/2*sqrt((xg - xb)**2 + (yg - yb)**2) - mask_buffer_dist, 0.0)
        r = round(r/self.res)
        mask = generate_circle_mask(r)
        #self.get_logger().info(f'{(xg,xb)}, {(yg,yb)}')
        #self.get_logger().info(f'{u}, {v}, {r}')
        mapslice = self.obstacle_map[v-r:v+1+r, u-r:u+1+r]
        (rows, cols) = mapslice.shape
        mask = mask[0:rows, 0:cols]
        #self.get_logger().info(f'{mapslice.shape}, {mask.shape}')
        mapslice[mask] = 100
        self.obstacle_map[v-r:v+1+r, u-r:u+1+r] = mapslice

        flattened_obstacle_map = self.obstacle_map.flatten().tolist()
        mapmsg = OccupancyGrid()
        mapmsg.header.frame_id = 'map'
        mapmsg.info.width = self.width
        mapmsg.info.height = self.height
        mapmsg.info.resolution = self.res
        mapmsg.info.origin = self.map2grid.toPose()
        mapmsg.data = flattened_obstacle_map
        #self.get_logger().info(f"{flattened_obstacle_map}")
        self.pubmap.publish(mapmsg)
        if len(self.last_path) > 0 and not recheck_path(self.last_path, self.obstacle_map):
            self.replan_count = 4
        #self.replan_count = 4


    def cb_coverage_pose(self, msg):
        pt = PlanarTransform.fromPose(msg.pose)
        (u1, v1) = (round(pt.x()/self.res), round(pt.y()/self.res))
        
        pts = []
        if self.last_waypoint != None:
            (u0, v0) = (round(self.last_waypoint.x()/self.res), round(self.last_waypoint.y()/self.res))
            t = 0.0
            while t < 1.0:
                u_inter = round((1 - t)*u0 + t*u1)
                v_inter = round((1 - t)*v0 + t*v1)
                pts.append((u_inter, v_inter))
                t += ALPHA
        else: 
            pts.append((u1, v1))

        for (u, v) in pts:
            mask = COVERAGE_MASK.copy()
            mapslice = self.coverage_map[v-COVERAGE_RADIUS:v+1+COVERAGE_RADIUS, u-COVERAGE_RADIUS:u+1+COVERAGE_RADIUS]
            (rows, cols) = mapslice.shape
            mask = mask[0:rows, 0:cols]
            mapslice[mask] = 100
            self.coverage_map[v-COVERAGE_RADIUS:v+1+COVERAGE_RADIUS, u-COVERAGE_RADIUS:u+1+COVERAGE_RADIUS] = mapslice
            flattened_coverage_map = self.coverage_map.flatten().tolist()
            mapmsg = OccupancyGrid()
            mapmsg.header.frame_id = 'map'
            mapmsg.info.width = self.width
            mapmsg.info.height = self.height
            mapmsg.info.resolution = self.res
            mapmsg.info.origin = self.map2grid.toPose()
            mapmsg.data = flattened_coverage_map
            self.pubcoverage_map.publish(mapmsg)
        
        self.last_waypoint = pt
        

class RrtNode:
    def __init__(self, x, y, theta):
        # Define a parent (cleared for now).
        self.parent = None

        # Define/remember the state/coordinates (x,y,theta).
        self.x = x
        self.y = y
        self.theta = theta
        self.res = RES

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<Point %5.2f,%5.2f>" % (self.x, self.y, self.theta))

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    def intermediate_orient(self, other, alpha):
        return RrtNode(self.x, 
                    self.y,
                    self.theta + alpha * (other.theta - self.theta))

    def intermediate_pos(self, other, alpha):
        return RrtNode(self.x + alpha * (other.x - self.x),
                    self.y + alpha * (other.y - self.y),
                    self.theta)

    # Compute the relative Euclidean distance to another node.
    def theta_distance(self, other):
        return sqrt((other.theta - self.theta)**2)

    def euler_distance(self, other):
        return sqrt((other.x - self.x)**2 + (other.y - self.y)**2)
    
    def is_free(self, mapdata, bit_mask=ROBOT, delta=DELTA):
        (u, v) = (round(self.x/self.res), round(self.y/self.res))
        mask = imutils.rotate(bit_mask, angle = -180.0/pi * self.theta)
        mapslice = mapdata[v-delta:v+1+delta, u-delta:u+1+delta]
        return not np.any(np.logical_and(mapslice, mask))

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other, mapdata):
        intermediate = RrtNode(self.x, self.y, other.theta)
        if other.theta != None and self.theta_distance(intermediate) != 0.0:
            for alpha in vandercorput(THETA_SPACE / self.theta_distance(intermediate)):
                if not self.intermediate_orient(intermediate, alpha).is_free(mapdata):
                    return False
        
        if intermediate.euler_distance(other) != 0.0:
            for alpha in vandercorput(D_SPACE / intermediate.euler_distance(other)):
                if not intermediate.intermediate_pos(other, alpha).is_free(mapdata):
                    return False
        
        return True
    

    
def recheck_path(path, mapdata):
    for i in range(len(path) - 1):
        if not path[i].connectsTo(path[i+1], mapdata):
            return False
        
    return True



def rrt(startnode, goalnode, mapdata):
    # Start the tree with the startnode (set no parent just in case).
    startnode.parent = None
    if not startnode.is_free(mapdata) or not goalnode.is_free(mapdata, bit_mask=ROUND_BOT, delta=BLOB_DELTA):
        return None
    tree = [startnode]

    # Function to attach a new node to an existing node: attach the
    # parent, add to the tree, and show in the figure.
    def addtotree(oldnode, newnode):
        newnode.parent = oldnode
        tree.append(newnode)
        # visual.drawEdge(oldnode, newnode, color='g', linewidth=1)
        # visual.show()

    # Loop - keep growing the tree.
    steps = 0
    while True:
        # Determine the target state.
        if random.random() < 0.10:
            targetnode = goalnode
        else:
            targetnode = RrtNode(random.uniform(X_MIN, X_MAX), random.uniform(Y_MIN, Y_MAX), 0.0)


        # Directly determine the distances to the target node.
        distances = np.array([node.euler_distance(targetnode) for node in tree])
        index     = np.argmin(distances)
        nearnode  = tree[index]
        d         = distances[index]

        targetnode.theta = np.arctan2(targetnode.y - nearnode.y, targetnode.x - nearnode.x)

        # Determine the next node.
        direction = np.array([targetnode.x - nearnode.x, targetnode.y - nearnode.y])
        direction = direction/d
        delta = min(DSTEP, d) # Ensure we don't overshoot targetnode
        nextnode = RrtNode(nearnode.x + delta*direction[0], nearnode.y + delta*direction[1], targetnode.theta)

        # Check whether to attach.
        if nextnode.is_free(mapdata) and nearnode.connectsTo(nextnode, mapdata):
            addtotree(nearnode, nextnode)

            # If within DSTEP, also try connecting to the goal.  If
            # the connection is made, break the loop to stop growing.
            if nextnode.euler_distance(goalnode) <= DSTEP and nextnode.connectsTo(goalnode, mapdata):
                goalnode.theta = nextnode.theta
                addtotree(nextnode, goalnode)
                break

        # Check whether we should abort - too many steps or nodes.
        steps += 1
        if (steps >= S_MAX) or (len(tree) >= N_MAX):
            return None

    # Build the path.
    path = [goalnode]
    while path[0].parent is not None:
        path.insert(0, path[0].parent)

    # Report and return.
    print("Finished after %d steps and the tree having %d nodes" %
          (steps, len(tree)))
    return path


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = PlannerNode('planner')

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
