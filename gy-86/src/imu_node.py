#! /usr/bin/env python
import time

from IMU.IMU import IMU
from sensor_msgs.msg import Imu, MagneticField
from numpy import matrix
import rospy

imu = IMU()

imu.mpu6050.gyroscope_offsets = {'gx': rospy.get_param("/imu_node/gyroscope_offsets/gx"),
                                 'gy': rospy.get_param("/imu_node/gyroscope_offsets/gy"),
                                 'gz': rospy.get_param("/imu_node/gyroscope_offsets/gz")}

imu.mpu6050.accelerometer_offsets = {'ax': rospy.get_param("/imu_node/accelerometer_offsets/ax"),
                                     'ay': rospy.get_param("/imu_node/accelerometer_offsets/ay"),
                                     'az': rospy.get_param("/imu_node/accelerometer_offsets/az")}

imu.compass.offsets = {'cx': rospy.get_param("/imu_node/compass_offsets/cx"),
                       'cy': rospy.get_param("/imu_node/compass_offsets/cy"),
                       'cz': rospy.get_param("/imu_node/compass_offsets/cz")}

m = rospy.get_param("/imu_node/compass_calibration_matrix")
if len(m) != 9: raise Exception("Calibration Matrix is not 3 x 3")
imu.compass.calibration_matrix = matrix([m[0:3], m[3:6], m[6:9]])

print(imu.compass.calibration_matrix)

rospy.init_node('imu_node')
imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)


def publish_imu(timer_event):
    imu_msg, mag_msg, s = imu.read_ros_compatible_data(sensors_to_read=('gyr', 'acc', 'comp'))

    if not rospy.is_shutdown():
        try:
            imu_pub.publish(imu_msg)
            mag_pub.publish(mag_msg)
        except:
            pass
        # print(time.time() - s)


imu_timer = rospy.Timer(rospy.Duration(0.005), publish_imu)
rospy.spin()
