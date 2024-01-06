#!/usr/bin/env python3

import rosbag
import math
from matplotlib import pyplot as plt
import matplotlib
import numpy as np

def read_data_from_rosbag(bag_filename):
    new_orientation_x = []
    new_orientation_y = []
    new_orientation_z = []
    new_orientation_w = []

    new_linearacc_x = []
    new_linearacc_y = []
    new_linearacc_z = []

    new_angularvel_x = []
    new_angularvel_y = []
    new_angularvel_z = []

    new_magfield_x = []
    new_magfield_y = []
    new_magfield_z = []

    new_sec = []

    with rosbag.Bag(bag_filename, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/imu']):
            new_orientation_x.append(msg.IMU.orientation.x)
            new_orientation_y.append(msg.IMU.orientation.y)
            new_orientation_z.append(msg.IMU.orientation.z)
            new_orientation_w.append(msg.IMU.orientation.w)

            new_linearacc_x.append(msg.IMU.linear_acceleration.x)
            new_linearacc_y.append(msg.IMU.linear_acceleration.y)
            new_linearacc_z.append(msg.IMU.linear_acceleration.z)

            new_angularvel_x.append(msg.IMU.angular_velocity.x)
            new_angularvel_y.append(msg.IMU.angular_velocity.y)
            new_angularvel_z.append(msg.IMU.angular_velocity.z)

            new_magfield_x.append(msg.MagField.magnetic_field.x)
            new_magfield_y.append(msg.MagField.magnetic_field.y)
            new_magfield_z.append(msg.MagField.magnetic_field.z)

            s = float(f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs}")
            new_sec.append(s)

    return (new_orientation_x, new_orientation_y, new_orientation_z, new_orientation_w,
            new_linearacc_x, new_linearacc_y, new_linearacc_z,
            new_angularvel_x, new_angularvel_y, new_angularvel_z,
            new_magfield_x, new_magfield_y, new_magfield_z, new_sec)

def calculate_roll_pitch_yaw(orientation_x, orientation_y, orientation_z, orientation_w):
    roll = []
    pitch = []
    yaw = []

    for i in range(len(orientation_x)):
        t0 = +2.0 * (orientation_w[i] * orientation_x[i] + orientation_y[i] * orientation_z[i])
        t1 = +1.0 - 2.0 * (orientation_x[i] * orientation_x[i] + orientation_y[i] * orientation_y[i])
        roll.append(math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (orientation_w[i] * orientation_y[i] - orientation_z[i] * orientation_x[i])
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch.append(math.degrees(math.asin(t2))

        t3 = +2.0 * (orientation_w[i] * orientation_z[i] + orientation_x[i] * orientation_y[i])
        t4 = +1.0 - 2.0 * (orientation_y[i] * orientation_y[i] + orientation_z[i] * orientation_z[i])
        yaw.append(math.degrees(math.atan2(t3, t4))

    return roll, pitch, yaw

def calculate_statistics(data):
    mean = np.mean(data)
    std = np.std(data)
    return mean, std

def plot_data(timestamps, data, xlabel, ylabel, title):
    plt.plot(timestamps, data)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.show()

def plot_histogram(data, bins, mean, std, xlabel, ylabel, title):
    plt.hist(data, bins)
    plt.axvline(x=mean, color='r')
    plt.axvline(x=mean - std, color='g', linestyle='--')
    plt.axvline(x=mean + std, color='g', linestyle='--')
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.show()

if __name__ == "__main__":
    bag_filename = 'stationary_data.bag'

    (orientation_x, orientation_y, orientation_z, orientation_w,
     linearacc_x, linearacc_y, linearacc_z,
     angularvel_x, angularvel_y, angularvel_z,
     magfield_x, magfield_y, magfield_z, sec) = read_data_from_rosbag(bag_filename)

    roll, pitch, yaw = calculate_roll_pitch_yaw(orientation_x, orientation_y, orientation_z, orientation_w)

    roll_mean, roll_std = calculate_statistics(roll)
    pitch_mean, pitch_std = calculate_statistics(pitch)
    yaw_mean, yaw_std = calculate_statistics(yaw)

    linearacc_x_mean, linearacc_x_std = calculate_statistics(linearacc_x)
    linearacc_y_mean, linearacc_y_std = calculate_statistics(linearacc_y)
    linearacc_z_mean, linearacc_z_std = calculate_statistics(linearacc_z)

    angularvel_x_mean, angularvel_x_std = calculate_statistics(angularvel_x)
    angularvel_y_mean, angularvel_y_std = calculate_statistics(angularvel_y)
    angularvel_z_mean, angularvel_z_std = calculate_statistics(angularvel_z)

    magfield_x_mean, magfield_x_std = calculate_statistics(magfield_x)
    magfield_y_mean, magfield_y_std = calculate_statistics(magfield_y)
    magfield_z_mean, magfield_z_std = calculate_statistics(magfield_z)

    plot_data(sec, roll, 'Time (in seconds)', 'Roll (in degrees)', 'Roll Data')
    plot_data(sec, pitch, 'Time (in seconds)', 'Pitch (in degrees)', 'Pitch Data')
    plot_data(sec, yaw, 'Time (in seconds)', 'Yaw (in degrees)', 'Yaw Data')

    plot_histogram(roll, 50, roll_mean, roll_std, 'Roll (in degrees)', 'Frequency', 'Roll Histogram')
    plot_histogram(pitch, 50, pitch_mean, pitch_std, 'Pitch (in degrees)', 'Frequency', 'Pitch Histogram')
    plot_histogram(yaw, 50, yaw_mean, yaw_std, 'Yaw (in degrees)', 'Frequency', 'Yaw Histogram')

    plot_data(sec, linearacc_x, 'Time (in seconds)', 'Linear Acceleration X', 'Linear Acceleration X Data')
    plot_data(sec, linearacc_y, 'Time (in seconds)', 'Linear Acceleration Y', 'Linear Acceleration Y Data')
    plot_data(sec, linearacc_z, 'Time (in seconds)', 'Linear Acceleration Z', 'Linear Acceleration Z Data')

    plot_histogram(linearacc_x, 100, linearacc_x_mean, linearacc_x_std, 'Linear Acceleration X', 'Frequency', 'Linear Acceleration X Histogram')
    plot_histogram(linearacc_y, 100, linearacc_y_mean, linearacc_y_std, 'Linear Acceleration Y', 'Frequency', 'Linear Acceleration

