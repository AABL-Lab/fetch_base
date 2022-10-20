# !/usr/bin/env python 
# import numpy as np
from matplotlib import pyplot as plt
import numpy as np 
import rospy
# from qualisys.msg import Subject
from sensor_msgs.msg import LaserScan

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


if __name__ == '__main__':
    counter = 0
    print("banana")
    try: 
        rospy.init_node("visual")
    except:
        pass
    lidar_data = rospy.wait_for_message("/base_scan", LaserScan)
    # rospy.Subscriber("/base_scan", LaserScan, plotter)
    print(lidar_data)


    theta = np.linspace(lidar_data.angle_min, lidar_data.angle_max, 662)
    print(theta)
    plt.polar(theta, lidar_data.ranges)
    plt.savefig('lidar_data_visual4.png')

    # rospy.init_node("plotter")
    # rospy.Subscriber("position_measurements", Subject, plot_x)
    # plt.ion()
    # plt.show()
    # rospy.spin()
