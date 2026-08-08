#!/usr/bin/env python3
#
#   goal_sender.py
#
#   The vertex_planning stack is idle until something asks it to go
#   somewhere: planner.py only plans when a goal arrives on
#   /planner_goal_pose (or when /initialpose has armed its random-coverage
#   brain).  This node walks the robot through a fixed tour of goals so the
#   demo runs without a human clicking in RVIZ.
#
#   Node:       /goal_sender
#   Publish:    /planner_goal_pose      geometry_msgs/PoseStamped  (frame 'grid')
#   TF Listen:  grid -> world
#
import sys
import math
import traceback

import rclpy
import tf2_ros
from rclpy.node        import Node
from rclpy.time        import Time, Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# Poses below are in the planner's 'grid' frame: the churchsidemaze1b map is
# 200x200 cells at 0.2 m, so grid coordinates run 0..40 m with the map origin
# at grid (19, 19).  Every pose sits in a cell whose 19-cell neighbourhood is
# clear, which is what planner.py's inflated footprint check demands -- the
# default odometry origin, grid (19, 19), does not qualify, so the robot has
# to be placed before it can be sent anywhere.
START = (12.4, 22.0, 0.303)
MAP_ORIGIN = (-19.0, -19.0)     # from churchsidemaze1b.yaml

# Waypoints roughly 4 m apart along a route through the maze.  Short legs
# matter: planner.py's RRT samples uniformly over the whole 40 m x 40 m grid
# with a 0.125 m step, so a goal on the far side of the maze regularly hits
# its node cap and reports "Couldn't find path".
DEFAULT_TOUR = [
    (15.6, 23.0,  1.571),       # into the north-south corridor
    (15.6, 27.0,  1.720),       # north past the pillars
    (15.0, 31.0,  2.356),
    (12.0, 34.0,  0.559),       # around the corner into the top corridor
    (13.6, 35.0,  0.000),
    (17.6, 35.0,  0.000),       # east along the top
    (21.6, 35.0, -0.175),
    (25.0, 34.4, -1.571),       # around the far corner
    (25.0, 30.4, -1.300),       # and back down the east side
]

ARRIVAL_RADIUS = 0.35       # m, close enough to call the goal reached
GOAL_TIMEOUT   = 75.0       # s, give up on a goal and move on
STALL_PERIOD   = 25.0       # s of standing still before resending the goal
STALL_DISTANCE = 0.25       # m, motion smaller than this counts as stalled


class GoalSender(Node):
    def __init__(self, name, tour, loop):
        super().__init__(name)

        self.tour  = tour
        self.loop  = loop
        self.index = 0
        self.sent_time = None
        self.last_send = None
        self.stall_pose = None
        self.stall_time = None

        self.tfBuffer = tf2_ros.Buffer()
        self.tflisten = tf2_ros.TransformListener(
            self.tfBuffer, self, spin_thread=True)

        self.pubgoal = self.create_publisher(
            PoseStamped, '/planner_goal_pose', 10)
        self.pubinit = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.last_place = None
        self.timer = self.create_timer(0.5, self.cb_timer)
        self.get_logger().info("Goal sender running, %d goals queued"
                               % len(self.tour))

    def shutdown(self):
        self.destroy_node()

    def send(self, goal):
        (x, y, theta) = goal
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'grid'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(theta / 2)
        msg.pose.orientation.w = math.cos(theta / 2)
        self.pubgoal.publish(msg)
        self.last_send = self.get_clock().now()
        self.get_logger().info("Goal %d/%d: (%.2f, %.2f, %.2f)"
                               % (self.index + 1, len(self.tour), x, y, theta))

    def pose(self):
        # Where is the robot in the grid frame?  Returns None until the
        # planner/localizer have published the grid frame.
        try:
            tfmsg = self.tfBuffer.lookup_transform(
                'grid', 'world', Time(), timeout=Duration(seconds=0.2))
        except tf2_ros.TransformException:
            return None
        t = tfmsg.transform.translation
        return (t.x, t.y)

    def place(self):
        # localize.py starts with map->odom at identity, which parks the robot
        # at grid (19, 19) -- a cell the planner considers occupied once the
        # footprint is inflated.  An /initialpose moves map->odom so the robot
        # starts somewhere it can actually plan from.
        (x, y, theta) = START
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x + MAP_ORIGIN[0]
        msg.pose.pose.position.y = y + MAP_ORIGIN[1]
        msg.pose.pose.orientation.z = math.sin(theta / 2)
        msg.pose.pose.orientation.w = math.cos(theta / 2)
        self.pubinit.publish(msg)
        self.get_logger().info("Placed robot at grid (%.1f, %.1f)" % (x, y))

    def cb_timer(self):
        if self.index >= len(self.tour):
            return

        here = self.pose()
        if here is None:
            return

        # Place the robot, and keep placing it until the TF tree agrees.  A
        # single /initialpose is easy to lose: it is published seconds after
        # this node starts, and if discovery with the localizer has not
        # finished the message goes nowhere and the robot never leaves grid
        # (19, 19).  Waiting for the transform to actually move also keeps a
        # goal from being planned against the old start pose.
        if self.sent_time is None:
            if math.hypot(here[0] - START[0], here[1] - START[1]) <= 0.5:
                pass                        # placed, fall through to goal 1
            elif (self.last_place is None or
                  (self.get_clock().now() - self.last_place).nanoseconds
                  * 1e-9 > 2.0):
                self.place()
                self.last_place = self.get_clock().now()
                return
            else:
                return

        # First goal, once the TF tree is up.
        if self.sent_time is None:
            self.send(self.tour[self.index])
            self.sent_time = self.get_clock().now()
            return

        goal    = self.tour[self.index]
        dist    = math.hypot(goal[0] - here[0], goal[1] - here[1])
        now     = self.get_clock().now()
        elapsed = (now - self.sent_time).nanoseconds * 1e-9

        if dist < ARRIVAL_RADIUS:
            self.get_logger().info("Reached goal %d after %.1f s"
                                   % (self.index + 1, elapsed))
        elif elapsed > GOAL_TIMEOUT:
            self.get_logger().warn("Goal %d timed out at %.2f m away"
                                   % (self.index + 1, dist))
        else:
            # Still working.  When RRT hits its node cap the planner logs
            # "Couldn't find path" and publishes nothing, and the only
            # symptom out here is a robot that has stopped moving -- so
            # resend the goal once it has been still long enough.
            moved = (self.stall_pose is None or
                     math.hypot(here[0] - self.stall_pose[0],
                                here[1] - self.stall_pose[1]) > STALL_DISTANCE)
            if moved:
                self.stall_pose = here
                self.stall_time = now
            elif (now - self.stall_time).nanoseconds * 1e-9 > STALL_PERIOD:
                self.send(goal)
                self.stall_time = now
            return

        self.stall_pose = None
        self.index += 1
        if self.index >= len(self.tour):
            if self.loop:
                self.index = 0
            else:
                self.get_logger().info("Tour complete")
                return
        self.send(self.tour[self.index])
        self.sent_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)

    argv = [a for a in (args or sys.argv[1:]) if not a.startswith('--ros-args')]
    loop = '--loop' in argv

    node = GoalSender('goal_sender', DEFAULT_TOUR, loop)
    try:
        rclpy.spin(node)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
