# !/usr/bin/env python 
from matplotlib import pyplot as plt
import numpy as np 
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from laser_range_visualization import smoothen

distance = 1.5
percent_range = 0.1

def collision(msg):
    temp = False
    try:
        collision_pub = rospy.Publisher("/in_collision", Bool, latch=True, queue_size=5)
    except:
        pass
    data = smoothen(msg, 3)
    forward_data = data[int((len(data)) * (0.5 - percent_range)):int((len(data)) * (0.5 + percent_range))]
    for i in range(len(forward_data)):
        if (forward_data[i] < distance):
            collision_pub.publish(Bool(True))
    collision_pub.publish(Bool(False))
            


if __name__ == '__main__':
    print("orange")
    try: 
        rospy.init_node("detect")
    except:
        pass
    rospy.Subscriber("/base_scan", LaserScan, collision, queue_size=5)
    rospy.sleep(0.01)
    rospy.spin()