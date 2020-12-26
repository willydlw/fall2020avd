#! /usr/bin/env python

import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations 

import math 

# robot state variables
position = Point() 
yaw = 0

# fsm state
state = 0

# goal point
desired_position = Point()
desired_position.x = 0
desired_position.y = 6
desired_position.z = 0

# parameters
yaw_precision = math.pi / 90     # +/- 2 degrees allowed 
dist_precision = 0.3 
max_angular_speed = math.pi/2         # radians per second, 90 degrees/sec)
kp = 0.5 

# publishers
pub = None 


# callbacks
def clbk_odom(msg):
   global position
   global yaw 

   position = msg.pose.pose.position 

   rospy.loginfo_once('function clbk_odom')
   rospy.loginfo_once("whose coordinate reference frame? child_frame_id: [%s]" % msg.child_frame_id)
   rospy.loginfo_once("pose.position x: [%s], y:[%s], z:[%s]" %(position.x, position.y, position.z))

   quaternion = (
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w
   )

   euler = transformations.euler_from_quaternion(quaternion)
   yaw = euler[2]    # roll [0], pitch[1], yaw[2]

   rospy.loginfo_once('initial yaw: [%s] rad, [%s] degrees \n' % (yaw, yaw*180.0/math.pi )) 


def change_state(next_state):
   global state 

   state = next_state
   print 'State changed to [%s]' % state 


def normalize_angle(angle):
   if(math.fabs(angle) > math.pi):
      angle = angle - (2*math.pi * angle) / (math.fabs(angle))
   return angle 

def fix_yaw(des_pos):
   global yaw, pub, yaw_precision, state, position

   desired_yaw = math.atan2( (des_pos.y - position.y), (des_pos.x - position.x))
   err_yaw = desired_yaw - yaw

   #pid = min(math.fabs(kp * err_yaw), math.fabs(max_angular_speed))

   rospy.loginfo_once("fix_yaw function")
   rospy.loginfo_once('des_pos.y:   [%s], des_pos.x   [%s]' % (des_pos.y, des_pos.x))
   rospy.loginfo_once('position.y:  [%s], position.x: [%s]\n' % (position.y, position.x))
   
   rospy.loginfo_once("desired_yaw: [%s], yaw:       [%s]" % (desired_yaw, yaw))
   rospy.loginfo_once("err_yaw:     [%s]  rad, [%s] degrees" % (err_yaw, err_yaw *180.0/math.pi))
   rospy.loginfo_once('yaw_precision: [%s] rad, [%s] degrees\n' % (yaw_precision, yaw_precision*180.0/math.pi))
   
   twist_msg = Twist()
   if math.fabs(err_yaw) > yaw_precision:
      rospy.loginfo("fix_yaw_function, err_yaw > yaw_precision")
      #twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7 
      if err_yaw > 0:
         twist_msg.angular.z = 0.7  #pid
         rospy.loginfo("rotate counter clockwise")
      else:
         twist_msg.angular.z = -0.7 #-pid 
         rospy.loginfo("rotate clockwise")


   print('twist_msg.angular.z [%s]' % twist_msg.angular.z)

   pub.publish(twist_msg)

   # state change conditions
   if math.fabs(err_yaw) <= yaw_precision:
      rospy.loginfo('yaw_error: [%s] deg within allowed precision\n' % (err_yaw*180.0/math.pi))
      change_state(1)


def go_straight_ahead(des_pos):
   global yaw, pub, yaw_precision, state, position

   # atan2 returns value between pi, -pi in radians
   desired_yaw = math.atan2((des_pos.y - position.y),(des_pos.x - position.x))
   err_yaw = desired_yaw - yaw 
   err_pos = math.sqrt(pow(des_pos.y - position.y,2) + pow(des_pos.x - position.x, 2))

   print 'go_straight_ahead, desired_yaw: [%s]' % desired_yaw 
   print 'err_yaw: [%s]' % err_yaw
   print 'err_pos: [%s]\n' % err_pos 

   if err_pos > dist_precision:
      twist_msg = Twist()
      twist_msg.linear.x = 0.6
      print("publishing twist_msg to go straight")
      pub.publish(twist_msg)
   else:
      rospy.loginfo_once('final: position.y: [%s], position.x: [%s]' % (position.y, position.x))
      rospy.loginfo_once('Position error:    [%s]' % err_pos) 
      change_state(2)
   
   # state change conditions 
   if math.fabs(err_yaw) > yaw_precision:
      rospy.loginfo('function go_straight_ahead, yaw error: [%s] > yaw_precision: [%s]' % (err_yaw, yaw_precision))
      change_state(0)

def done():
   twist_msg = Twist()
   twist_msg.linear.x = 0
   twist_msg.angular.z = 0 
   pub.publish(twist_msg)


def main():
   global pub, desired_position

   rospy.init_node('go_to_point')
   pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
   sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

   rospy.sleep(1)

   rate = rospy.Rate(20)
   while not rospy.is_shutdown():
      if state == 0:
         fix_yaw(desired_position)
      elif state == 1:
         go_straight_ahead(desired_position)
      elif state == 2:
         done()
         pass 
      else:
         rospy.logerr('Unknown state: [%d]', state)
         pass 

if __name__ == '__main__':
   main()

