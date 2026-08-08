#!/usr/bin/env python3
#
#   wheelcontrol_skeleton.py
#
#   THIS IS A SKELETON ONLY.  PLEASE COPY/RENAME AND THEN EDIT!
#
#   Node:       /wheelcontrol
#   Publish:    /wheel_state            sensor_msgs/JointState
#               /wheel_desired          sensor_msgs/JointState
#   Subscribe:  /wheel_command          sensor_msgs/JointState
#
#   Other Inputs:   Encoder Channels (GPIO)
#   Other Outputs:  Motor Driver Commands (via I2C)
#
import rclpy
import smbus
import traceback
import numpy as np
from packages.gyro import Gyro

from math import pi
from packages.encoder import Encoder
from packages.driver  import Driver

from rclpy.node         import Node
from sensor_msgs.msg    import JointState


#
#   Constants
#
# Left/right definitions
LEFT   = 0
RIGHT  = 1
MOTORS = 2

LEFTNAME  = 'leftwheel'
RIGHTNAME = 'rightwheel'
GYRONAME  = 'gyro'
MOTORNUM  = {LEFTNAME:LEFT, RIGHTNAME:RIGHT}
assert MOTORS == len(MOTORNUM)

# Desired loop rates
SERVORATE    = 200.0            
PUBLISHRATE  =  50.0
PUBLISHCYCLE = int(SERVORATE / PUBLISHRATE)

GEAR_RATIO = 49.0
ENC_TICKS_PER_REV = 16.0
MAX_VEL = 15.0
MAX_ACCEL = 18.0
K_PWM_R = 10.0
K_PWM_L = 10.0
PWM_DEADZONE = 80.0
T_FBK = 0.1
T_FILTER = 0.1
ERROR_MAX = 1.0
VEL_TOL = 0.35  

#
#   Utilities
#
def sat(x, limit):
    # Saturate the value to +/- the limit.
    return min(max(x, -limit), limit)

def inflate(x, offset):
    # Add the offset in each direction, leaving just a tiny deadband.
    if   (x >  1.0): return (x + offset)
    elif (x < -1.0): return (x - offset)
    else:            return (x)

#
#   Wheel Control Node Class
#
class WheelControlNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Incoming commands.
        self.cmdvel  = [0.0, 0.0]
        self.cmdtime = [self.get_clock().now(), self.get_clock().now()]

        # Initialize any other state variables.
        self.pos = [0.0, 0.0]
        self.vel = [0.0, 0.0]

        self.pos_des = [0.0, 0.0]
        self.vel_des = [0.0, 0.0]
        self.pwm_des = [0.0, 0.0]
        self.omega = 0.0
        self.phi = 0.0

        self.pubcount = 0               # Publishing counter

        # Connect to the I2C bus #1.
        #i2cbus = smbus.SMBus(1)

        # Initialize the I/O objects for the encoders/motors.
        #self.encoder = Encoder()
        #self.gyro = Gyro(i2cbus)
        #self.driver = Driver(i2cbus)

        # Create the publishers for the actual and (internal) desired.
        self.pubdes = self.create_publisher(JointState, 'wheel_desired', 10)
        self.pubact = self.create_publisher(JointState, 'wheel_state',   10)

        # Create a subscriber to listen to wheel commands.
        self.subcmd = self.create_subscription(
            JointState, 'wheel_command', self.cb_cmdmsg, 10)

        # Create the timer to drive the node.
        self.lastnow = self.get_clock().now()
        self.timer   = self.create_timer(1/SERVORATE, self.cb_timer)
        rate         = 1e9 / self.timer.timer_period_ns

        # Report and return.
        self.get_logger().info("Wheel control running at %fHz" % rate)

    # Shutdown
    def shutdown(self):
        # Destroy the timer.
        self.timer.destroy()

        # Clean up the low level.
        #self.driver.shutdown()
        #self.encoder.shutdown()

        # Finally, shut down the node.
        self.destroy_node()


    # Command subscription callback
    def cb_cmdmsg(self, msg):
        # Check the message structure.
        assert len(msg.name) == len(msg.velocity), \
            "Wheel command msg name/velocity must have same length"

        # Note the current time (to timeout the command).
        now = self.get_clock().now()

        # Extract and save the velocity commands at this time.
        for i in range(len(msg.name)):
            mtr = MOTORNUM[msg.name[i]]
            # First set the velocity, then mark the time (so we don't
            # temporarily declare an old command as new).
            self.cmdvel[mtr]  = msg.velocity[i]
            self.cmdtime[mtr] = now


    # Timer callback
    def cb_timer(self):
        # Grab the current time and measure dt.
        now = self.get_clock().now()
        dt  = (now - self.lastnow).nanoseconds * 1e-9
        self.lastnow = now
        assert dt > 0.0, f"Timer did not advance time (dt = {dt}s)!"

        #(self.omega,_) = self.gyro.read()
        #self.phi += dt * self.omega

        # self.get_logger().info(f"omega: {self.omega}     phi: {self.phi}")
        
        if max((now - self.cmdtime[RIGHT]).nanoseconds*1e-9, (now - self.cmdtime[LEFT]).nanoseconds*1e-9) > 0.25:
            self.cmdvel[RIGHT] = 0.0
            self.cmdvel[LEFT] = 0.0

        rpos = self.encoder.right() * 2*pi / (ENC_TICKS_PER_REV*GEAR_RATIO)
        lpos = self.encoder.left() * 2*pi / (ENC_TICKS_PER_REV*GEAR_RATIO)

        rvel_raw = (rpos - self.pos[RIGHT]) / dt
        lvel_raw = (lpos - self.pos[LEFT]) / dt

        self.pos[RIGHT] = rpos
        self.pos[LEFT] = lpos

        self.vel[RIGHT] += (dt/T_FILTER)*(rvel_raw - self.vel[RIGHT])
        self.vel[LEFT] += (dt/T_FILTER)*(lvel_raw - self.vel[LEFT])

        # print(f'right: {self.vel[RIGHT]}      left: {self.vel[LEFT]}')

        raccel = sat((self.cmdvel[RIGHT] - self.vel_des[RIGHT])/dt, MAX_ACCEL)
        laccel = sat((self.cmdvel[LEFT] - self.vel_des[LEFT])/dt, MAX_ACCEL)

        self.vel_des[RIGHT] = sat(self.vel_des[RIGHT] + raccel*dt, MAX_VEL)
        self.vel_des[LEFT] = sat(self.vel_des[LEFT] + laccel*dt, MAX_VEL)

        self.pos_des[RIGHT] += self.vel_des[RIGHT]*dt
        self.pos_des[LEFT] += self.vel_des[LEFT]*dt
        
        self.pos_des[RIGHT] = min(max(self.pos_des[RIGHT], self.pos[RIGHT] - ERROR_MAX), self.pos[RIGHT] + ERROR_MAX)
        self.pos_des[LEFT] = min(max(self.pos_des[LEFT], self.pos[LEFT] - ERROR_MAX), self.pos[LEFT] + ERROR_MAX)
        
        rvel_fbk = (self.pos_des[RIGHT] - self.pos[RIGHT]) / T_FBK
        lvel_fbk = (self.pos_des[LEFT] - self.pos[LEFT]) / T_FBK
        
        rvel_new = self.vel_des[RIGHT] + rvel_fbk
        lvel_new = self.vel_des[LEFT] + lvel_fbk
        
        # print(f'right: {rvel_new}      left: {lvel_new}')

        self.pwm_des[RIGHT] = K_PWM_R*(rvel_new) + np.sign(rvel_new)*PWM_DEADZONE
        self.pwm_des[LEFT] = K_PWM_L*(lvel_new) + np.sign(lvel_new)*PWM_DEADZONE

        if abs(rvel_new) < VEL_TOL:
            self.pwm_des[RIGHT] = 0.0
        if abs(lvel_new) < VEL_TOL:
            self.pwm_des[LEFT] = 0.0

        # Send the PWM.
        try:
            self.driver.pwm(self.pwm_des[LEFT], self.pwm_des[RIGHT])
        except Exception as e:
            print(e)
            self.get_logger().warn("Unable to commmand PWM!")

        # Publish the actual and desired wheel states.  The number
        # position/velocity/efforts elements should either match the
        # number of names or be zero.
        self.pubcount = (self.pubcount + 1) % PUBLISHCYCLE
        if not self.pubcount:
            msg = JointState()
            msg.header.stamp = now.to_msg()

            msg.name         = [LEFTNAME,       RIGHTNAME, GYRONAME]
            msg.position     = [self.pos[LEFT], self.pos[RIGHT], 0.0] #self.phi]
            msg.velocity     = [self.vel[LEFT], self.vel[RIGHT], 0.0] #self.omega]
            msg.effort       = []
            self.pubact.publish(msg)

            msg.name         = [LEFTNAME,           RIGHTNAME]
            msg.position     = [self.pos_des[LEFT], self.pos_des[RIGHT]]
            msg.velocity     = [self.vel_des[LEFT], self.vel_des[RIGHT]]
            msg.effort       = [self.pwm_des[LEFT], self.pwm_des[RIGHT]]
            self.pubdes.publish(msg)


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = WheelControlNode('wheelcontrol')

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
