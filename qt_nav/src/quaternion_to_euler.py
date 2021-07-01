#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import sys

class EulerAngles:
    def __init__(self):
     self.pub = rospy.Publisher('EulerPos',Twist,queue_size = 10)
     self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
    
    def get_rotation (self,msg):

     orientation_q = msg.pose.pose.orientation
     position = msg.pose.pose.position
     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
     (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
     pos_x = position.x
     pos_y = position.y
     pos_z = position.z
     pos_msg = Twist()
     pos_msg.linear.x = pos_x
     pos_msg.linear.y = pos_y
     pos_msg.linear.z = pos_z

     pos_msg.angular.x = pitch
     pos_msg.angular.y = roll
     pos_msg.angular.z = yaw
     self.pub.publish(pos_msg)
    

def main(args):
      
     rospy.init_node('my_quaternion_to_euler')
     r = rospy.Rate(1)
     while not rospy.is_shutdown():
      ic = EulerAngles()
      r.sleep()
      
if __name__=='__main__':
     main(sys.argv) 
     
     













