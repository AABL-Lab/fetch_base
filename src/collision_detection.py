# !/usr/bin/env python 
from matplotlib import pyplot as plt
import numpy as np 
import rospy
from std_msgs.msg import Bool
import threading
from sensor_msgs.msg import LaserScan
from laser_range_visualization import smoothen

def collision(msg):
    try:
        collision_pub = rospy.Publisher("/in_collision", Bool, latch=True, queue_size=1)
    except:
        pass
    data = smoothen(msg, 3)
    forward_data = data[int((len(data)) * 0.3):int((len(data)) * 0.8)]
    # print(" ")
    for i in range(len(forward_data)):
        if (forward_data[i] < 1):
            print(i + (662 * 0.3))
            collision_pub.publish(Bool(True))
            # pub_thread.stop()
        else: 
            collision_pub.publish(Bool(False))
            


if __name__ == '__main__':
    counter = 0
    print("orange")
    try: 
        rospy.init_node("detect")
    except:
        pass
    pub_thread = PublishThread(0)
    rospy.Subscriber("/base_scan", LaserScan, collision, queue_size=5)
    # if (collision == True):
    #     print("collision")
    # else:
    #     print("no collision")
    rospy.sleep(0.01)
    rospy.spin()