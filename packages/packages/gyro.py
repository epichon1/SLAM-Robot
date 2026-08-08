#!/usr/bin/env python3
#
#   gyro_notquite.py
#
#   THIS IS A SKELETON ONLY.  PLEASE COPY/RENAME AND THEN EDIT!
#   IN PARTICULAR SEARCH FOR 'FIXME'!
#
#   Create a Gyro object to read the gyroscope from the IMU.  This
#   uses the I2C bus and has to be used in the same node as driver.py.
#
import math
import smbus
import time


#
#   Constants
#
# FIXME: You will  want to adjust this for your IMU.
GYRO_SCALE_ADJUSTMENT = 1.014     # Multiplicative scale adjustment


#
#   Gyro Object
#
#   This implements the gyro readings.
#
class Gyro:
    # I2C Definitions and Communication
    I2C_ADDR = 0x68             # I2C Device Number

    REG_CONFIG   = 0x1A         # Register to set configuration
    REG_GYROCFG  = 0x1B         # Register to set gyro config
    REG_GYROX    = 0x43         # Registers to read gyro X axis
    REG_GYROY    = 0x45         # Registers to read gyro Y axis
    REG_GYROZ    = 0x47         # Registers to read gyro Z axis
    REG_PWRMGMT1 = 0x6B         # Register to set power management
    REG_ADDR     = 0x75         # Register reporting I2C address

    # Read/Write an I2C bus register.  This is only used in the
    # initialization.  So if it fails, retry (after a small delay).
    def readReg(self, reg):
        while True:
            try:
                result = self.i2cbus.read_byte_data(self.I2C_ADDR, reg)
                return result
            except:
                print("Gyro ReadReg(0x%02x) failed.  Retrying..." % reg)
                time.sleep(0.05)

    def writeReg(self, reg, byte):
        while True:
            try:
                self.i2cbus.write_byte_data(self.I2C_ADDR, reg, byte)
                return
            except:
                print("Gyro WriteReg(0x%02x, 0x%02x) failed.  Retrying..." %
                      (reg, byte))
                time.sleep(0.05)

    # Burst read/write multiple I2C bus registers.  Do not catch errors.
    def readRegList(self, reg, N):
        return self.i2cbus.read_i2c_block_data(self.I2C_ADDR, reg, N)
    def writeRegList(self, reg, bytelist):
        self.i2cbus.write_i2c_block_data(self.I2C_ADDR, reg, bytelist)


    # Initialize.
    def __init__(self, i2cbus, range = math.radians(500.0),
                 scale = GYRO_SCALE_ADJUSTMENT):
        # Save the I2C bus object.
        self.i2cbus = i2cbus
    
        # Confirm a connection to the IMU and gyro.
        if (self.readReg(self.REG_ADDR) != self.I2C_ADDR):
            raise Exception("IMU not connected!")

        # Set the clock source to match Gyro Z (more precise).
        self.writeReg(self.REG_PWRMGMT1, 0x03)

        # Set the gyroscope low-pass filter to 42Hz so we have to
        # sample at >=42Hz to avoid aliasing.
        self.writeReg(self.REG_CONFIG, 0x03)

        # Wait 50ms to let the setup and filter change settle.
        time.sleep(0.05)

        # Set the gyro full-range (default 500 deg/sec). Feel free to change.
        self.setrange(range)

        # Set the hard-coded scale adjustment and offset by calibration.
        self.scale  = scale
        self.offset = self.calibrate()

        # Assume the current reading is thus zero.
        self.reading = (0.0, False)

        # Report.
        print("Gyro enabled.")

    # Cleanup.
    def shutdown(self):
        # Nothing to do.
        pass


    # Set the Gyro full range (in rad/sec).
    def setrange(self, range):
        # Select the range (has to be 250, 500, 1000, or 2000 deg/sec).
        rangenum = int(math.ceil(math.log2(range / math.radians(250.0))))
        rangenum = min(max(rangenum, 0), 3)

        # Determine and set the actual range.
        self.range = math.radians(250.0) * (2 ** rangenum)
        self.writeReg(self.REG_GYROCFG, rangenum << 3)

        # Let the change take effect before the next sample is read.
        time.sleep(0.01)

        # Report.
        print("Setting gyro full range to %.3f rad/sec (%.0f deg/sec)"
              % (self.range, math.degrees(self.range)))

    # Calibrate the Gyro Offset (assuming the IMU is not moving!).
    def calibrate(self, N = 200):
        # Report.
        print("Measuring the gyro offset - please put down/don't move")

        # Grab the samples.
        cnt  = 0
        sum  = 0.0
        sum2 = 0.0
        while (cnt < N):
            # Skip bad reads.
            try:
                (speed, _) = self.readraw()
                cnt  = cnt  + 1
                sum  = sum  + speed
                sum2 = sum2 + speed**2
            except:
                print("Bad Gyro read during calibration.  Skipping.")

            # Wait just a moment for the next sample.
            time.sleep(0.01)
        avg = sum/cnt
        std = math.sqrt((sum2 - cnt*avg**2)/(cnt-1))

        # Report and check whether the std is above an acceptable
        # limit which would imply movement.
        stdlim = 0.01
        print("Gyro offset %.3f rad/sec (std %.3f <= %.3f limit)"
              % (avg, std, stdlim))
        if (std > stdlim):
            raise Exception("IMU was held or moving during gyro calibration")

        # Return the offset, being the average reading.
        return avg


    def readraw(self):
        # Grab the high (first) and low byte (second) in one read.
        bytes = self.readRegList(self.REG_GYROZ, 2)

        # Convert into a signed 16bit number.
        value = (bytes[0] << 8) | bytes[1]
        if value > 2**15 - 1:
            value -= 2**16

        # Check for saturation.
        saturated = ((value > 32700) or (value < -32700))

        # Convert into rad/sec.
        omegaraw = -(value/32767)*self.range*self.scale
    
        # Return the speed and saturation flag.
        return (omegaraw, saturated)


    def read(self):
        # Place the code in a try statement, in case the read fails.
        try:
            # Take the reading.
            (omega, saturated) = self.readraw()

            # Subtract the offset and save the reading.
            self.reading = (omega - self.offset, saturated)

        except:
            # Do not update the reading.
            print("Bad Gyro read.  Returning last valid measurement.")

        # Return the reading (speed and saturation flag).
        return self.reading


#
#   Main
#
def main(args=None):
    # Grab the I2C bus.
    i2cbus = smbus.SMBus(1)
    
    # Initialize the motor gyro.
    # gyro = Gyro(i2cbus, range=math.radians(200.0))
    gyro = Gyro(i2cbus)

    # Try reading.
    try:
        while True:
            (omega, sat) = gyro.read()
            print("GyroZ = %7.3f rad/sec (sat = %d) " % (omega, sat))
            time.sleep(0.1)
    except:
        print("Breaking the loop...")

    # Cleanup (does nothing).
    gyro.shutdown()

if __name__ == "__main__":
    main()
