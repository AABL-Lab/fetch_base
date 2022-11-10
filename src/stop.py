# !/usr/bin/env python 
from matplotlib import pyplot as plt
import numpy as np 
import rospy
from std_msgs.msg import Bool
import threading
from sensor_msgs.msg import LaserScan
from laser_range_visualization import smoothen
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import collision_detection ## TODO: fix this!! import the publisher so our subsriber works

TwistMsg = Twist

# class PublishThread(threading.Thread):
#     def __init__(self, rate):
#         print("thread init")
#         super(PublishThread, self).__init__()
#         self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size = 1)
#         self.x = 0.0
#         self.y = 0.0
#         self.z = 0.0
#         self.th = 0.0
#         self.speed = 0.0
#         self.turn = 0.0
#         self.condition = threading.Condition()
#         self.done = False

#         # Set timeout to None if rate is 0 (causes new_message to wait forever
#         # for new data to publish)
#         if rate != 0.0:
#             self.timeout = 1.0 / rate
#         else:
#             self.timeout = None

#         self.start()
#     def update(self, x, y, z, th, speed, turn):
#         self.condition.acquire()
#         self.x = x
#         self.y = y
#         self.z = z
#         self.th = th
#         self.speed = speed
#         self.turn = turn
#         # Notify publish thread that we have a new message.
#         self.condition.notify()
#         self.condition.release()
#     def stop(self):
#         print("stop")
#         self.done = True
#         self.update(0, 0, 0, 0, 0, 0)
#         self.join()

def stop(data):
    if(data.data):
        stop_pub = rospy.Publisher('cmd_vel', TwistMsg, queue_size=1)
        twist_msg = TwistMsg()
        twist = twist_msg.twist
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        stop_pub.publish(twist_msg)
    else:
        print("go")



if __name__ == '__main__':
    counter = 0
    print("apple")
    try: 
        rospy.init_node("stop")
    except:
        pass
    rospy.Subscriber("/in_collision", Bool, stop, queue_size=1)
    rospy.sleep(0.01)
    rospy.spin()