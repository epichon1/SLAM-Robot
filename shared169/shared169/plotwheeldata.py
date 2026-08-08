#!/usr/bin/env python3
#
#   plotwheeldata.py bagfoldername
#
#   Plot the wheel command, desired, and actual for left/right wheels.
#   If 'bagfoldername' is not given or given as 'latest', use the most
#   recent bag folder.
#
import rclpy
import numpy as np
import matplotlib.pyplot as plt

import glob, os, sys

from rosbag2_py                 import SequentialReader
from rosbag2_py._storage        import StorageOptions, ConverterOptions
from rclpy.serialization        import deserialize_message

from sensor_msgs.msg            import JointState


#
#  Grab the wheel data
#
def wheeldata(msgs, t0, jointname):
    # Make sure we have something.
    if len(msgs) == 0:
        return ([], [], [], [])

    # Grab the dimensions.
    N = len(msgs[0].name)       # Number of joints
    M = len(msgs)               # Samples

    # Grab the data (for all joints).
    sec  = np.array([msg.header.stamp.sec     for msg in msgs])
    nano = np.array([msg.header.stamp.nanosec for msg in msgs])
    t = sec + nano*1e-9 - t0

    pos = np.array([msg.position for msg in msgs])
    vel = np.array([msg.velocity for msg in msgs])
    eff = np.array([msg.effort   for msg in msgs])

    if pos.size == 0:
        pos = np.full((M,N), np.nan)
    if vel.size == 0:
        vel = np.full((M,N), np.nan)
    if eff.size == 0:
        eff = np.full((M,N), np.nan)
        
    # Extract the named joint.
    try:
        index = msgs[0].name.index(jointname)
    except Exception:
        raise ValueError("No data for joint '%s'" % jointname)

    pos = pos[:,index]
    vel = vel[:,index]
    eff = eff[:,index]

    # # Re-zero time.
    # tstart = min(t)
    # print("Starting at time ", tstart)
    # t = t - tstart

    # Return the data
    return (t, pos, vel, eff)

    
#
#  Plot the Wheel Data
#
def plotwheel(commandmsgs, desiredmsgs, statemsgs, t0, bagname, jointname):
    # Grab the data.
    (tc, pc, vc, ec) = wheeldata(commandmsgs, t0, jointname);
    (td, pd, vd, ed) = wheeldata(desiredmsgs, t0, jointname);
    (ta, pa, va, ea) = wheeldata(statemsgs,   t0, jointname);

    # Create a figure to plot pos/vel/eff vs. t
    fig, axs = plt.subplots(nrows=3, sharex=True)

    # Plot the data in the subplots.
    axs[0].plot(tc, pc, 'b', linestyle='dashed', marker='o', markersize=0.5)
    axs[0].plot(td, pd, 'r', linestyle='dotted', marker='o', markersize=0.5)
    axs[0].plot(ta, pa, 'g', linestyle='solid' , marker='x', markersize=0.5)
    axs[0].set(ylabel='Position (rad)')
    axs[1].plot(tc, vc, 'b', linestyle='dashed', marker='o', markersize=0.5)
    axs[1].plot(td, vd, 'r', linestyle='dotted', marker='o', markersize=0.5)
    axs[1].plot(ta, va, 'g', linestyle='solid' , marker='x', markersize=0.5)
    axs[1].set(ylabel='Velocity (rad/sec)')
    axs[2].plot(tc, ec, 'b', linestyle='dashed', marker='o', markersize=0.5)
    axs[2].plot(td, ed, 'r', linestyle='dotted', marker='o', markersize=0.5)
    axs[2].plot(ta, ea, 'g', linestyle='solid' , marker='x', markersize=0.5)
    axs[2].set(ylabel='PWM (level)')

    # Connect the time.
    axs[2].set(xlabel='Time (sec)')

    # Add the title and legend.
    axs[0].set(title="'%s' data in '%s'" % (jointname, bagname))
    axs[0].legend(('Command', 'Desired', 'Actual'))

    # Draw grid lines and allow only "outside" ticks/labels in each subplot.
    for ax in axs.flat:
        ax.grid()
        ax.label_outer()


#
#  Main Code
#
def main():
    # Grab the arguments.
    bagname   = 'latest' if len(sys.argv) < 2 else sys.argv[1]

    # Check for the latest ROS bag:
    if bagname == 'latest':
        # Report.
        print("Looking for latest ROS bag...")

        # Look at all bags, making sure we have at least one!
        dbfiles = glob.glob('*/*.db3')
        if not dbfiles:
            raise FileNoFoundError('Unable to find a ROS2 bag')

        # Grab the modification times and the index of the newest.
        dbtimes = [os.path.getmtime(dbfile) for dbfile in dbfiles]
        i = dbtimes.index(max(dbtimes))

        # Select the newest.
        bagname = os.path.dirname(dbfiles[i])

    # Report.
    print("Reading ROS bag '%s'"  % bagname)


    # Set up the BAG reader.
    reader = SequentialReader()
    try:
        reader.open(StorageOptions(uri=bagname, storage_id='sqlite3'),
                    ConverterOptions('', ''))
    except Exception as e:
        print("Unable to read the ROS bag '%s'!" % bagname)
        print("Does it exist and WAS THE RECORDING Ctrl-c KILLED?")
        raise OSError("Error reading bag - did recording end?") from None

    # Get the starting time.
    t0 = reader.get_metadata().starting_time.nanoseconds * 1e-9 - 0.01

    # Get the topics and types:
    print("The bag contain message for:")
    for x in reader.get_all_topics_and_types():
        print("  topic %-20s of type %s" % (x.name, x.type))


    # Pull out the relevant messages.
    commandmsgs = []
    desiredmsgs = []
    statemsgs = []
    while reader.has_next():
        # Grab a message.
        (topic, rawdata, timestamp) = reader.read_next()

        # Pull out the deserialized message.
        if   topic == '/wheel_command':
            commandmsgs.append(deserialize_message(rawdata, JointState))
        elif topic == '/wheel_desired':
            desiredmsgs.append(deserialize_message(rawdata, JointState))
        elif topic == '/wheel_state':
            statemsgs.append(deserialize_message(rawdata, JointState))


    # Process the data
    print("Plotting the left wheel...")
    plotwheel(commandmsgs, desiredmsgs, statemsgs, t0, bagname, 'leftwheel')
    print("Plotting the right wheel...")
    plotwheel(commandmsgs, desiredmsgs, statemsgs, t0, bagname, 'rightwheel')

    # Show
    plt.show()


#
#   Run the main code.
#
if __name__ == "__main__":
    main()
