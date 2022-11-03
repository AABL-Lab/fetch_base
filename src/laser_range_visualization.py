# !/usr/bin/env python 
from matplotlib import pyplot as plt
import numpy as np 
import rospy
from sensor_msgs.msg import LaserScan

def smoothen(msg, radius):
    data = np.array(msg.ranges)
    data_copy = data
    nan_indicies = np.argwhere(np.isnan(data))
    for index in nan_indicies:
        index = index[0]
        imin = max(0, index - radius)
        imax = min(len(data), index + radius + 1)
        temp_sqr = data[imin:imax]
        temp_sqr = temp_sqr[~np.isnan(temp_sqr)]
        try: 
            data_copy[index] = np.mean(temp_sqr)
        except:
            data_copy[index] = msg.range_max
    return data_copy


def plotter(msg):
    theta = np.linspace(msg.angle_min, msg.angle_max, 662)
    new_data = smoothen(msg, 3)

    plt.clf()
    plt.polar(theta, new_data)
    plt.draw()
    plt.pause(0.00000000001)
    plt.ion()
    plt.show()


if __name__ == '__main__':
    counter = 0
    print("banana")
    try: 
        rospy.init_node("visual")
    except:
        pass
    rospy.Subscriber("/base_scan", LaserScan, plotter, queue_size=5)
    rospy.sleep(0.01)
    rospy.spin()
