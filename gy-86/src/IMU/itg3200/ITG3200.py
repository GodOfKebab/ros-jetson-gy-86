#!/usr/bin/python
# vim: ai:ts=4:sw=4:sts=4:et:fileencoding=utf-8
#
# ITG3200 gyroscope control class
#
# Copyright 2013 Michal Belica <devel@beli.sk>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

from __future__ import print_function
import smbus
import sys


def int_sw_swap(x):
    """Interpret integer as signed word with bytes swapped"""
    xl = x & 0xff
    xh = x >> 8
    xx = (xl << 8) + xh
    return xx - 0xffff if xx > 0x7fff else xx


class ITG3200(object):
    """ITG3200 digital gyroscope control class.
    Supports data polling at the moment.
    """

    def __init__(self, bus_nr=1, addr=0x68):
        """ Sensor class constructor
        Params:
            bus_nr .. I2C bus number
            addr   .. ITG3200 device address
        """
        self.bus = smbus.SMBus(bus_nr)
        self.addr = addr
        self.default_init()

        self.total_gyr_offset = [0, 0, 0]
        self.calibration_cycle_count = 1
        self.offsets = {'gx': 0.0,
                        'gy': 0.0,
                        'gz': 0.0}

    def sample_rate(self, lpf, div):
        """Set internal sample rate, low pass filter frequency.
        Sets device parameters DLPF_CFG and SMPLRT_DIV.
        Also sets FS_SEL to 0x03 which is required to initialize the device.
        Params:
            lpf .. (code from the list)
              code   LPF  sample rate
                 0 256Hz  8kHz
                 1 188Hz  1kHz
                 2  98Hz  1kHz
                 3  42Hz  1kHz
                 4  20Hz  1kHz
                 5  10Hz  1kHz
                 6   5Hz  1kHz
            div .. internal sample rate divider (SMPLRT_DIV will be set to div-1)
        """
        if not (lpf >= 0 and lpf <= 0x6):
            raise ValueError("Invalid low pass filter code (0-6).")
        if not (div >= 0 and div <= 0xff):
            raise ValueError("Invalid sample rate divider (0-255).")
        self.bus.write_byte_data(self.addr, 0x15, div - 1)
        self.bus.write_byte_data(self.addr, 0x16, 0x18 | lpf)

    def default_init(self):
        """Initialization with default values:
        8kHz internal sample rate, 256Hz low pass filter, sample rate divider 8.
        """
        self.sample_rate(0, 8)

    def read_data(self):
        """Read and return data tuple for x, y and z axis
        as signed 16-bit integers.
        """
        gx = int_sw_swap(self.bus.read_word_data(self.addr, 0x1d)) / 14.375 + self.offsets['gx']
        gy = int_sw_swap(self.bus.read_word_data(self.addr, 0x1f)) / 14.375 + self.offsets['gy']
        gz = int_sw_swap(self.bus.read_word_data(self.addr, 0x21)) / 14.375 + self.offsets['gz']

        return (gx, gy, gz)

    def calibrate(self, num_of_calibration_measurements=4500, print_status=False):

        import rospy

        max_line_length = 0
        self.offsets = {'gx': 0.0,
                        'gy': 0.0,
                        'gz': 0.0}

        if print_status:
            print("Calibrating gyroscope offsets, please hold still...")

        while (self.calibration_cycle_count < num_of_calibration_measurements) and (not rospy.is_shutdown()):

            (gx, gy, gz) = self.read_data()
            self.total_gyr_offset[0] -= gx
            self.total_gyr_offset[1] -= gy
            self.total_gyr_offset[2] -= gz

            self.calibration_cycle_count += 1

            if print_status:
                avg_gx = self.total_gyr_offset[0] / self.calibration_cycle_count
                avg_gy = self.total_gyr_offset[1] / self.calibration_cycle_count
                avg_gz = self.total_gyr_offset[2] / self.calibration_cycle_count

                s = "[{}/{}] gx:{:0.4f}  gy:{:0.4f}  gz:{:0.4f}".format(self.calibration_cycle_count,
                                                                                    num_of_calibration_measurements,
                                                                                    avg_gx,
                                                                                    avg_gy,
                                                                                    avg_gz)

                max_line_length = len(s) if len(s) > max_line_length else max_line_length
                num_of_space = max_line_length - len(s)

                print(s + ' ' * num_of_space, end='\r')

        if print_status:
            print('\nDone.')

        self.offsets = {'gx': self.total_gyr_offset[0] / self.calibration_cycle_count,
                        'gy': self.total_gyr_offset[1] / self.calibration_cycle_count,
                        'gz': self.total_gyr_offset[2] / self.calibration_cycle_count}

        return not rospy.is_shutdown()


if __name__ == '__main__':
    import time

    sensor = ITG3200()  # update with your bus number and address

    while True:
        gx, gy, gz = sensor.read_data()
        print(gx, gy, gz)
        time.sleep(0.01)
