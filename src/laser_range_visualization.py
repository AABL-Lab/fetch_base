# !/usr/bin/env python 
# import numpy as np
from matplotlib import pyplot as plt
import numpy as np 
import rospy
# from qualisys.msg import Subject
from sensor_msgs.msg import LaserScan

# from astropy.io import fits
# from astropy.utils.data import get_pkg_data_filename
# from astropy.convolution import Gaussian1DKernel
# from scipy.signal import convolve as scipy_convolve
# from astropy.convolution import convolve

from sklearn.impute import KNNImputer

# def plot_x(msg):
#     global counter
#     if counter % 10 == 0:
#         stamp = msg.header.stamp
#         time = stamp.secs + stamp.nsecs * 1e-9
#         plt.plot(msg.position.y, msg.position.x, '*')
#         plt.axis("equal")
#         plt.draw()
#         plt.pause(0.00000000001)

#     counter += 1

def plotter(msg):
    theta = np.linspace(msg.angle_min, msg.angle_max, 662)
    # print(theta)
    ranges = np.array(msg.ranges)
    imputer = KNNImputer(n_neighbors=2)
    imputer.fit_transform(ranges)
    # ranges[np.isnan(ranges)] = 1

    # kernel = Gaussian1DKernel(1)
    # astropy_conv = convolve(ranges, kernel)
    # print(astropy_conv)

    # print(ranges)
    plt.clf()
    plt.polar(theta, ranges)
    plt.draw()
    plt.pause(0.00000000001)
    # plt.savefig('msg_visual4.png')


if __name__ == '__main__':
    counter = 0
    print("banana")
    try: 
        rospy.init_node("visual")
    except:
        pass
    # lidar_data = rospy.wait_for_message("/base_scan", LaserScan)
    rospy.Subscriber("/base_scan", LaserScan, plotter)
    # print(msg)
    plt.ion()
    plt.show()
    rospy.spin()


    # theta = np.linspace(lidar_data.angle_min, lidar_data.angle_max, 662)
    # print(theta)
    # plt.polar(theta, lidar_data.ranges)
    # plt.savefig('lidar_data_visual4.png')

    # rospy.init_node("plotter")
    # rospy.Subscriber("position_measurements", Subject, plot_x)
    # plt.ion()
    # plt.show()
    # rospy.spin()
