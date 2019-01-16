"""This program handles the communication over I2C
between a Raspberry Pi and a MPU-6050 Gyroscope / Accelerometer combo.
Made by: MrTijn/Tijndagamer
Released under the MIT License
Copyright (c) 2015, 2016, 2017 MrTijn/Tijndagamer
"""

import smbus
import time


class mpu6050:
    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Rate Divider Register
    RATE_DIVIDER = 0x19

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # MPU-6050 Registers

    # Power Management registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    # Interrupt registers
    INT_ENABLE = 0x38
    INT_STATUS = 0x3A

    def __init__(self, address, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        # Software Calibration to zero-mean.
        # must run self.zero_mean_calibration() on level ground to properly calibrate.
        self.use_calibrated_values = False
        self.mean_calibrations = [0, 0, 0, 0, 0, 0]
        # if return_gravity == FALSE, then m/s^2 are returned
        self.return_gravity = False
        self.dplf_enabled = False

    # I2C communication methods

    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.

        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        if value >= 0x8000:
            return -((65535 - value) + 1)
        else:
            return value

    # MPU-6050 Methods

    def set_sleep_mode(self):
        """
        set the mpu6050 in sleep mode
        """
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0b01000000)

    def set_cycle_mode(self, frequency):
        """
        set the mpu6050 in cycle mode
        frequency is a value between 0 and 3 inclusive which is the frequency with which to collect data from the
        accelerometer

                -------------
                | 0 | 1.25Hz |
                | 1 |    5Hz |
                | 2 |   20Hz |
                | 3 |   40Hz |
                -------------
        :param frequency: int between 0 and 3
        """
        if 0 <= frequency <= 3:
            self.bus.write_byte_data(self.address, self.PWR_MGMT_2, frequency << 6 & 0xFF)
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0b00100000)

    def set_low_power_mode(self, frequency):
        """
        Set the mpu6050 in low power mode which means is enabled only the accelerometer and a sample is collected
        by frequency level
        :param frequency: int between 0 and 3
        """
        if 0 <= frequency <= 3:
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0b00100000)
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0b00101000)
            self.bus.write_byte_data(self.address, self.PWR_MGMT_2, frequency << 6 | 0b00000111)

    def full_power(self):
        """
        reset power byte to 0 exiting sleep or cycle mode
        """
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    def set_temperature_sensor(self, enabled):
        """
        Enable or disable the temperature sensor
        :param enabled: boolean
        """
        val = self.bus.read_byte_data(self.address, self.PWR_MGMT_1)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, val & 0b11111110 if enabled else val | 0b00000001)

    def set_standby_axis(self, enabled):
        """
        Enable or disable the accelerometer and gyroscope individual axis.
        :param enabled: list, tuple, iterable with exactly 6 elements of boolean values true means the axis is enabled
        false mean is disabled
        [ acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
        """
        byte = 0b00000000
        i = 5
        if len(enabled) != 6:
            return
        for val in enabled:
            if not val:
                byte |= 1 << i
            i -= 1
        self.bus.write_byte_data(self.address, self.PWR_MGMT_2, byte)

    def set_int_enable(self, enabled):
        """
        Enable or disable the interrupt byte of the mpu6050
        :param enabled: boolean
        """
        self.bus.write_byte_data(self.address, self.INT_ENABLE, 0b00000001 if enabled else 0b00000000)

    def get_int_status(self):
        status = self.bus.read_byte_data(self.address, self.INT_STATUS)
        if status == 0b00000001:
            return 1
        return 0

    def set_rate(self, rate):
        """
        Set the rate of how data is sampled by the mpu6050
        :param rate: int value that represent the rate of how data is sampled in microseconds
        rate must be a number not less than 1000 if dplf has been set or not less than 125 if dplf is disabled
        :return: False if rate is not a legal value and True if it is
        """
        if self.dplf_enabled:
            if rate < 1000:
                return False
        else:
            if rate < 125:
                return False
        hz = 1 / rate * 1000000
        gyrorate = 1000 if self.dplf_enabled else 8000
        divider = int((gyrorate - hz) / hz)
        if divider < 255:
            self.bus.write_byte_data(self.address, self.RATE_DIVIDER, divider & 0xFF)
            return True
        return False

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050.

        Returns the temperature in degrees Celcius.
        """
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)

        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53

        return actual_temp

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range.

        accel_range -- the range to set the accelerometer to. Using a
        pre-defined range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw=False):
        """Reads the range the accelerometer is set to.

        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g=False):
        """Gets and returns the X, Y and Z values from the accelerometer.

        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier = None
        accel_range = self.read_accel_range(True)

        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if self.use_calibrated_values:
            # zero-mean the data
            x -= self.mean_calibrations[0]
            y -= self.mean_calibrations[1]
            z -= self.mean_calibrations[2]

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope to range.

        gyro_range -- the range to set the gyroscope to. Using a pre-defined
        range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def read_gyro_range(self, raw=False):
        """Reads the range the gyroscope is set to.

        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope.

        Returns the read values in a dictionary.
        """
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_range = self.read_gyro_range(True)

        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier

        if self.use_calibrated_values:
            # zero mean the data. (Gyro scope seems to need calibration more than Accel)
            x -= self.mean_calibrations[3]
            y -= self.mean_calibrations[4]
            z -= self.mean_calibrations[5]

        return {'x': x, 'y': y, 'z': z}

    def get_all_data(self):
        """Reads and returns all the available data."""
        temp = self.get_temp()
        accel = self.get_accel_data(g=self.return_gravity)
        gyro = self.get_gyro_data()

        return [accel, gyro, temp]

    def set_calibrated_flag(self, value=True):
        """
        Set to TRUE to used the calculated zero-mean calibration, FALSE
        to use the default values. Is initialized to FALSE
        :param value: boolean
        """
        self.use_calibrated_values = value

    def set_calibration_values(self, values):
        """
        Load calibration mean from the dictionary values
        :param values: a dictionary with a subset of 'ax', 'ay', 'az', 'gx', 'gy', 'gz' as keys.
        """
        calibration = []
        for name in ['ax', 'ay', 'az', 'gx', 'gy', 'gz']:
            if name in values:
                calibration.append(values[name])
            else:
                calibration.append(0)
        self.mean_calibrations = calibration
        self.use_calibrated_values = True

    def zero_mean_calibration(self):
        print("** Calibrating the IMU **")
        print("** Place on level ground. re-run is not level at start **")
        # number of samples to collect. 200 == approx 5 seconds worth.
        N = 200
        # initialize the accumulators to 0
        ax, ay, az, gx, gy, gz = [0] * 6

        for i in range(N):
            # calibrate based on gravity, not m/s^2
            accel = self.get_accel_data(g=True)
            gyro = self.get_gyro_data()
            if i % 25 == 0:
                print('.')
            ax += accel['x']
            ay += accel['y']
            az += accel['z']
            gx += gyro['x']
            gy += gyro['y']
            gz += gyro['z']
            # wait 10ms for next sample
            time.sleep(10 / 1000.)
        # calculate the mean of each reading.
        ax /= float(N)
        ay /= float(N)
        az /= float(N)
        gx /= float(N)
        gy /= float(N)
        gz /= float(N)
        # compensate for 1g of gravity on 'z' axis.
        az -= 1
        # save the calibrations
        self.mean_calibrations = [ax, ay, az, gx, gy, gz]
        print("\n** Calibration Complete **")

        print('** offsets: ')
        print(''.join('{:02.4f}  '.format(n) for n in self.mean_calibrations))


if __name__ == "__main__":
    mpu = mpu6050(0x68)
    print(mpu.get_temp())
    accel_data = mpu.get_accel_data()
    print(accel_data['x'])
    print(accel_data['y'])
    print(accel_data['z'])
    gyro_data = mpu.get_gyro_data()
    print(gyro_data['x'])
    print(gyro_data['y'])
    print(gyro_data['z'])
