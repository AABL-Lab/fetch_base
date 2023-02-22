# !/usr/bin/env python 
from matplotlib import pyplot as plt
import numpy as np 
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from collision_detection import collision ## TODO: fix this!! import the publisher so our subsriber works

def stop(data):
    if(data.data):
        stop_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        twist_msg = Twist()
        twist = twist_msg
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        stop_pub.publish(twist_msg)
        print("stop")
    else:
        print("go")

if __name__ == '__main__':
    counter = 0
    print("apple")
    try: 
        rospy.init_node("stop")
    except:
        pass
    rospy.Subscriber("/in_collision", Bool, stop, queue_size=5)
    rospy.sleep(0.01)
    rospy.spin()