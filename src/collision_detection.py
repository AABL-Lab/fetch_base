# !/usr/bin/env python 
from matplotlib import pyplot as plt
import numpy as np 
import rospy
from sensor_msgs.msg import LaserScan
from laser_range_visualization import smoothen

def collision(msg):
    data = smoothen(msg, 3)
    forward_data = data[int((len(data) / 2) * 0.3):int((len(data) / 2) * 0.7)]
    # print(forward_data)
    for i in range(len(forward_data)):
        if (forward_data[i] < 1):
            print(i)


if __name__ == '__main__':
    counter = 0
    print("orange")
    try: 
        rospy.init_node("detect")
    except:
        pass
    rospy.Subscriber("/base_scan", LaserScan, collision, queue_size=5)
    rospy.sleep(0.01)
    rospy.spin()