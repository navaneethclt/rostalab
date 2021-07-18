#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Twist
import sys

class Position_calc:
    __prev_linear_vel = np.array([0.0,0.0,0.0])
    __prev_angular_vel = np.array([0.0,0.0,0.0])
    __linear_pos = np.array([0.0,0.0,0.0])
    __angular_pos = np.array([0.0,0.0,0.0])
    def __init__(self):
     self.pub = rospy.Publisher('Position_calc',Twist,queue_size = 10)
     self.sub = rospy.Subscriber ('/odom', Odometry, self.get_position)

    def get_position (self,msg):

     angular = msg.twist.twist.angular

     linear = msg.twist.twist.linear
     
     linear_vel = np.array([linear.x,linear.y,linear.z])
     angular_vel = np.array([angular.x,angular.y,angular.z])
     dt = 0.01
     
     self.__linear_pos = self.__linear_pos+ np.array(self.__prev_linear_vel+linear_vel)*dt*0.5
     self.__angular_pos = self.__angular_pos +np.array(self.__prev_angular_vel+angular_vel)*dt*0.5
     
     
     self.__prev_linear_vel = linear_vel
     self.__prev_angular_vel = angular_vel
     
     
     pos_msg = Twist()
     pos_msg.linear.x = self.__linear_pos[0]
     pos_msg.linear.y = self.__linear_pos[1]
     pos_msg.linear.z = self.__linear_pos[2]

     pos_msg.angular.x = self.__angular_pos[0] 
     pos_msg.angular.y = self.__angular_pos[1] 
     pos_msg.angular.z = self.__angular_pos[2] 
     self.pub.publish(pos_msg)
    

def main(args):
      
     rospy.init_node('my_Position_calc')
     r = rospy.Rate(100)
     while not rospy.is_shutdown():
      ic =Position_calc()
      r.sleep()
      
if __name__=='__main__':
     main(sys.argv) 
     
     













