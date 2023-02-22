# !/usr/bin/env python 
from matplotlib import pyplot as plt
import numpy as np 
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

if __name__ == '__main__':
    counter = 0
    print("mango")
    try: 
        rospy.init_node("move")
    except:
        pass
    move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=3)

    while(counter < 70):
        print(counter)
        twist_msg = Twist()
        twist = twist_msg
        twist.linear.x = 1 * 0.2
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        move_pub.publish(twist_msg)
        rospy.sleep(.25)
        move_pub.publish(twist_msg)
        counter+=1


    # rospy.spin()
