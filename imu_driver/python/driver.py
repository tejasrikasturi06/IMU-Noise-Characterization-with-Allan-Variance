#!/usr/bin/env python3

import rospy
import sys
import serial
import math
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from imu_driver.msg import imu_msg

def convert_euler_to_quaternion(roll, pitch, yaw):
    cos_yaw = math.cos(yaw * 0.5)
    sin_yaw = math.sin(yaw * 0.5)
    cos_pitch = math.cos(pitch * 0.5)
    sin_pitch = math.sin(pitch * 0.5)
    cos_roll = math.cos(roll * 0.5)
    sin_roll = math.sin(roll * 0.5)

    q_w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw
    q_x = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw
    q_y = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw
    q_z = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw

    return [q_w, q_x, q_y, q_z]

def parse_imu_data(data):
    fields = data.split(',')
    if len(fields) < 13 or "$VNYMR" not in data:
        return None

    roll, pitch, yaw = float(fields[3]), float(fields[2]), float(fields[1])
    magnetometer = [float(fields[4]), float(fields[5]), float(fields[6)]
    accel = [float(fields[7]), float(fields[8]), float(fields[9)]
    gyro = [float(fields[10]), float(fields[11]), float(fields[12].split('*')[0])]

    return accel, gyro, (roll, pitch, yaw), magnetometer

def main():
    rospy.init_node('imu_driver_node')

    imu_serial_port = rospy.get_param('~port', sys.argv[1])
    ser = serial.Serial(imu_serial_port, 115200, timeout=2.)
    
    imu_publisher = rospy.Publisher('/imu', imu_msg, queue_size=10)
    sleep_time = 0.025
    
    try:
        while not rospy.is_shutdown():
            data = ser.readline().decode('utf-8').strip()
            ser.write('$VNWRG,07,40*XX'.encode())
            imu_data = parse_imu_data(data)
            
            if imu_data:
                accel, gyro, orientation, magnetometer = imu_data
                roll, pitch, yaw = orientation
                quaternion = convert_euler_to_quaternion(roll, pitch, yaw)

                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = "IMU1_Frame"
                
                imu_message = Imu()
                imu_message.header = header
                imu_message.angular_velocity.x, imu_message.angular_velocity.y, imu_message.angular_velocity.z = gyro
                imu_message.linear_acceleration.x, imu_message.linear_acceleration.y, imu_message.linear_acceleration.z = accel
                imu_message.orientation.w, imu_message.orientation.x, imu_message.orientation.y, imu_message.orientation.z = quaternion

                mag_field = MagneticField()
                mag_field.header = header
                mag_field.magnetic_field.x, mag_field.magnetic_field.y, mag_field.magnetic_field.z = magnetometer
                
                custom_msg = imu_msg()
                custom_msg.Header = header
                custom_msg.IMU = imu_message
                custom_msg.MagField = mag_field
            
                imu_publisher.publish(custom_msg)

    except rospy.ROSInterruptException:
        ser.close()
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down imu driver")
    except Exception as e:
        rospy.logerr(f"An error occurred: {str(e)}")

if __name__ == '__main__':
    main()
