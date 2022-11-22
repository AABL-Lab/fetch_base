# !/usr/bin/env python 
from matplotlib import pyplot as plt
import numpy as np 
import rospy
from std_msgs.msg import Bool
import threading
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

TwistMsg = Twist

def stop(data):
    if(data.data):
        stop_pub = rospy.Publisher('cmd_vel', TwistMsg, queue_size=5)
        twist_msg = TwistMsg()
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
    print("mango")
    try: 
        rospy.init_node("move")
    except:
        pass
    move_pub = rospy.Publisher('cmd_vel', TwistMsg, queue_size=3)

    while(counter < 15):
        print(counter)
        twist_msg = TwistMsg()
        twist = twist_msg
        twist.linear.x = 1 * 0.2
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        move_pub.publish(twist_msg)
        rospy.sleep(1)
        move_pub.publish(twist_msg)
        counter+=1

    # stop_msg = TwistMsg()
    # twist_stop = stop_msg
    # twist_stop.linear.x = 0
    # twist_stop.linear.y = 0
    # twist_stop.linear.z = 0
    # twist_stop.angular.x = 0
    # twist_stop.angular.y = 0
    # twist_stop.angular.z = 0
    # move_pub.publish(twist_stop)
    # rospy.sleep(1)


    # rospy.spin()
