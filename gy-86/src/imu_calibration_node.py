#! /usr/bin/env python

from IMU import IMU
import rospy
import yaml
import datetime

imu = IMU()
timestamp = datetime.datetime.now()

rospy.init_node('imu_calibration_node')

gyr_offsets = {'gyroscope_offsets': {'gx': rospy.get_param("/gyroscope_offsets/gx"),
                                     'gy': rospy.get_param("/gyroscope_offsets/gy"),
                                     'gz': rospy.get_param("/gyroscope_offsets/gz")}}

acc_offsets = {'accelerometer_offsets': {'ax': rospy.get_param("/accelerometer_offsets/ax"),
                                         'ay': rospy.get_param("/accelerometer_offsets/ay"),
                                         'az': rospy.get_param("/accelerometer_offsets/az")}}

comp_offsets = {'compass_offsets': {'cx': rospy.get_param("/compass_offsets/cx"),
                                    'cy': rospy.get_param("/compass_offsets/cy"),
                                    'cz': rospy.get_param("/compass_offsets/cz")}}
comp_matrix = {'compass_calibration_matrix': rospy.get_param("/compass_calibration_matrix")}


# Gyroscope calibration
success = False
if rospy.get_param("~calibrate_gyr") and (not rospy.is_shutdown()):
    success = imu.mpu6050.calibrate_gyroscope(rospy.get_param("~gyr_num_of_calibration_measurements"),
                                              rospy.get_param("~print_gyr_status"))
    if success:
        gyr_offsets['gyroscope_offsets'] = imu.mpu6050.gyroscope_offsets
else:
    print("Skipping gyroscope calibration...")

# Accelerometer calibration
success = False
if rospy.get_param("~calibrate_acc") and (not rospy.is_shutdown()):
    rospy.sleep(2.0)
    success = imu.mpu6050.calibrate_accelerometer(rospy.get_param("~acc_num_of_calibration_measurements"),
                                                  rospy.get_param("~print_acc_status"))
    if success:
        acc_offsets['accelerometer_offsets'] = imu.mpu6050.accelerometer_offsets
else:
    print("Skipping accelerometer calibration...")

# Compass calibration
success = False
if rospy.get_param("~calibrate_comp") and (not rospy.is_shutdown()):
    rospy.sleep(2.0)
    success = imu.compass.calibrate(rospy.get_param("~comp_num_of_calibration_measurements"),
                                    rospy.get_param("~print_comp_status"),
                                    timestamp)
    if success:
        comp_offsets['compass_offsets'] = imu.compass.offsets
        m = imu.compass.calibration_matrix
        print(m)
        comp_matrix['compass_calibration_matrix'] = [round(float(m[0, 0]), 5), round(float(m[0, 1]), 5), round(float(m[0, 2]), 5),
                                                     round(float(m[1, 0]), 5), round(float(m[1, 1]), 5), round(float(m[1, 2]), 5),
                                                     round(float(m[2, 0]), 5), round(float(m[2, 1]), 5), round(float(m[2, 2]), 5)]
else:
    print("Skipping compass calibration...")

s = ''
s += yaml.dump(gyr_offsets, default_flow_style=False)
s += yaml.dump(acc_offsets, default_flow_style=False)
s += yaml.dump(comp_offsets, default_flow_style=False)
s += yaml.dump(comp_matrix, width=25)
print(s)

new_filename = '/home/nano/catkin_ws/src/gy-86/imu_calibration/yaml_files/{}-imu_calibration.yaml'.format(timestamp)
with open(new_filename, 'w') as the_new_yaml_file:
    the_new_yaml_file.write(s)

old_filename = '/home/nano/catkin_ws/src/gy-86/imu_calibration/yaml_files/imu_calibration.yaml'
with open(old_filename, 'w') as the_old_yaml_file:
    the_old_yaml_file.write(s)

