#!/usr/bin/env python3
#
#   rplidarfix.py
#
#   Process the RPLidar scan from /scanin to /scanout.  This addresses
#   two bugs in the plain RPLidar scan:
#
#     (a) The plain scan presents the samples ordered from angles
#         (-pi) to (pi). But the actual scanner rotates clockwise (in
#         opposite direction), so the temporal order is backwards.
#         This is inconsistent with the time increment parameter.
#         FIX: Reverse the order to match the actual temporal
#         sampling.  This means the time increment parameter works.
#
#     (b) The timestamp should be the time of the first sample.  But
#         the stamp appears to be the time when the software reordered
#         the samples and generated the ROS message.  FIX: Offset the
#         time stamp.  This shifts by the scan time (being the time
#         between scans) plus a fixed delay incurred by the RPLidar
#         software (defaulting to 0.027s)
#
#   It also allows the lidar to automatically start/stop depending on
#   whether any node subscribes to the /scan topic.  Note the auto-
#   start further includes a variable delay to allow IMU calibration
#   before the lidar starts.
#
#   To launch, use:
#
#       node_lidarfix = Node(
#           name       = 'lidarfix',
#           package    = 'shared169',
#           executable = 'rplidarfix',
#           output     = 'screen',
#           parameters = [{'timeshift':  0.027,     # This was hand-tuned
#                          'startdelay': 3.0,       # Time for the IMU
#                          'autostart':  True,      # Turn ON when listening
#                          'autostop':   True,      # Turn OFF when unused
#                          }],
#
#   To set/adjust the parameter while running, run:
#
#       ros2 param set /lidarfix /timeshift  0.027      # Shift data by 0.027s
#       ros2 param set /lidarfix /startdelay 3.0        # Delay start by 3s
#
#
#   Node:       lidarfix
#
#   Subscribe:  scanin                  sensor_msgs/LaserScan
#
#   Publish:    scanout                 sensor_msgs/LaserScan
#
#   Parameter:  timeshift               double
#               startdelay              double
#               autostart               boolean
#               autostop                boolean
#
import rclpy
import time
import traceback

from rclpy.node                 import Node
from rclpy.parameter            import Parameter
from rclpy.time                 import Time, Duration

from rcl_interfaces.msg         import ParameterDescriptor, ParameterType
from rcl_interfaces.msg         import SetParametersResult
from sensor_msgs.msg            import LaserScan
from std_msgs.msg               import Float64

from std_srvs.srv               import Empty


#
#   Parameters: Names and Default Values
#
# Time shift between the original RPLidar scan time and the ROS LaserScan msg
TIMESHIFTNAME   = 'timeshift'
TIMESHIFTVALUE  = 0.027

# Auto-start delay (to allow other initializations to complete)
STARTDELAYNAME  = 'startdelay'
STARTDELAYVALUE = 3.0

# Whether to automatically start/stop the lidar motor
AUTOSTARTNAME   = 'autostart'
AUTOSTARTVALUE  = False
AUTOSTOPNAME    = 'autostop'
AUTOSTOPVALUE   = False

# Check interval - check the subscribers after each interval.
CHECKINTERVAL   = 2.0


#
#   RP Lidar Fix Node Class
#
class RPNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        ### Parameters
        # Declare the parameters, so we can set from the launch
        # file and adjust at run time from the command line.
        booltype         = ParameterType.PARAMETER_BOOL
        doubletype       = ParameterType.PARAMETER_DOUBLE
        booldescriptor   = ParameterDescriptor(type = booltype)
        doubledescriptor = ParameterDescriptor(type = doubletype)
        self.declare_parameter(
            TIMESHIFTNAME,  descriptor=doubledescriptor, value=TIMESHIFTVALUE)
        self.declare_parameter(
            STARTDELAYNAME, descriptor=doubledescriptor, value=STARTDELAYVALUE)
        self.declare_parameter(
            AUTOSTARTNAME,  descriptor=booldescriptor,   value=AUTOSTARTVALUE)
        self.declare_parameter(
            AUTOSTOPNAME,   descriptor=booldescriptor,   value=AUTOSTOPVALUE)

        # Get the initial parameter values.
        # param          = self.get_parameter(TIMESHIFTNAME)
        # self.timeshift = param.get_parameter_value().double_value
        self.settimeshift( self.get_parameter(TIMESHIFTNAME).value)
        self.setstartdelay(self.get_parameter(STARTDELAYNAME).value)
        self.setautostart( self.get_parameter(AUTOSTARTNAME).value)
        self.setautostop(  self.get_parameter(AUTOSTOPNAME).value)

        # Add the parameter callback for changes in the parameter.
        self.add_on_set_parameters_callback(self.cb_parameter)

        ### Scan Subscribers/Publishers
        # Create the publisher for the republished scan messages.
        self.pubscan = self.create_publisher(LaserScan, 'scanout', 10)

        # Then create a subscriber to listen to the scan.
        self.create_subscription(LaserScan, 'scanin', self.cb_scanmsg, 10)

        ### Start/Stop Service Client and Timer.
        # Assume the lidar motor is on.
        self.running = True

        # Create the service clients.
        self.clistartmotor = self.create_client(Empty, 'start_motor')
        self.clistopmotor  = self.create_client(Empty, 'stop_motor')
        while not (self.clistartmotor.wait_for_service(timeout_sec=0.5) and
                   self.clistopmotor.wait_for_service(timeout_sec=0.5)):
            self.get_logger().info('Waiting for basic rplidar services...')

        # Create the timer to check the subscribers.
        self.timer = self.create_timer(CHECKINTERVAL, self.cb_timer)

        # Report and return.
        self.get_logger().info("LidarFix running")

    # Shutdown
    def shutdown(self):
        # Destroy the timer and shut down the node.
        self.timer.destroy()
        self.destroy_node()


    # Parameter Change Callback
    # NOTE this is changing after HUMBLE!  Technically, this callback
    # asks whether it is ok to change the parameter.  But the change
    # may still be rejected elsewhere.  new methods appear in JAZZY!
    def cb_parameter(self, params):
        for param in params:
            if (param.name == TIMESHIFTNAME and
                param.type_ == Parameter.Type.DOUBLE):
                self.settimeshift(param.value)
            if (param.name == STARTDELAYNAME and
                param.type_ == Parameter.Type.DOUBLE):
                self.setstartdelay(param.value)
            if (param.name == AUTOSTARTNAME and
                param.type_ == Parameter.Type.BOOL):
                self.setautostart(param.value)
            if (param.name == AUTOSTOPNAME and
                param.type_ == Parameter.Type.BOOL):
                self.setautostop(param.value)
        return SetParametersResult(successful=True)

    # Timeshift Parameter
    def settimeshift(self, timeshift):
        self.timeshift = timeshift
        self.get_logger().info("LidarFix set timeshift to %5.3fs" % timeshift)

    # Startdelay Parameter
    def setstartdelay(self, startdelay):
        self.startdelay = startdelay
        self.get_logger().info("LidarFix set startdelay to %4.1fs" % startdelay)

    # Auto-Start Parameter
    def setautostart(self, autostart):
        self.autostart = autostart
        self.get_logger().info("LidarFix set autostart to %r" % autostart)

    # Auto-Start Parameter
    def setautostop(self, autostop):
        self.autostop = autostop
        self.get_logger().info("LidarFix set autostop to %r" % autostop)


    # Timer Callback
    def cb_timer(self):
        # Check the number of subscribers.
        n = self.pubscan.get_subscription_count()
        # self.get_logger().info("LidarFix seeing %d subscribers" % n)

        # Decide whether to start/stop
        if (n > 0) and not self.running:
            self.startmotor()
        elif (n == 0) and self.running:
            self.stopmotor()

    # Start/Stop
    def startmotor(self):
        # Check whether to auto-start.
        if self.autostart:
            # Start the motor.  Use asynchronous service (do not wait).
            self.get_logger().info(
                "LidarFix waiting %4.1fs to start..." % self.startdelay)
            time.sleep(self.startdelay)
            self.get_logger().info("LidarFix starting lidar motor")
            self.clistartmotor.call_async(Empty.Request())
        else:
            self.get_logger().info("LidarFix not starting lidar motor")

        # Mark.
        self.running = True

    def stopmotor(self):
        # Check whether to auto-stop.
        if self.autostop:
            # Stop the motor.  Use asynchronous service (do not wait).
            self.get_logger().info("LidarFix stopping lidar motor")
            self.clistopmotor.call_async(Empty.Request())
        else:
            self.get_logger().info("LidarFix not stopping lidar motor")

        # Mark.
        self.running = False


    # Scan Message Callback
    def cb_scanmsg(self, msg):
        # Shift the start time by the fixed delay.
        msg.header.stamp = (Time().from_msg(msg.header.stamp) -
                            Duration(seconds=self.timeshift)).to_msg()

        # Reverse the alpha sample order.
        (msg.angle_min, msg.angle_increment, msg.angle_max) = \
            (msg.angle_max, -msg.angle_increment, msg.angle_min)

        # Reverse the range order.
        msg.ranges = msg.ranges[::-1]

        # Also reverse the intensity order.
        msg.intensities = msg.intensities[::-1]

        # Re-publish the scan data.
        self.pubscan.publish(msg)


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the custom node.
    node = RPNode('lidarfix')

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
