#! /usr/bin/env python

import rospy
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Pose
from tf import transformations


class GoToGoal:

   def __init__(self):

      # initialize publishers and subscribers
      self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
      self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)

      # set parameters
      self.yaw_precision = rospy.get_param('/go_to_goal/yaw_precision')
      self.distance_precision = rospy.get_param('/go_to_goal/distance_precision')
      self.max_velocity = rospy.get_param("/go_to_goal/max_velocity")

      # position and orientation data members
      self.current_position = Point()
      self.desired_position = Point()
      self.pose = Pose()

      # pid control data members
      self.Kp = 0.05
      self.Ki = 0.0
      self.Kd = 0.0

      self.Kpc = 0.5
      self.Kac = 0.2

      self.previous_time = 0
      self.previous_error = 0.0
      self.integral_error = 0.0 


   def set_goal_point(self, goal_point):
      self.desired_position.x = goal_point.x
      self.desired_position.y = goal_point.y
      self.desired_position.z = goal_point.z

   
   def update_curve(self):

      deltaX = self.desired_position.x - self.current_position.x
      deltaY = self.desired_position.y - self.current_position.y
     
      # TODO: change position error to squared difference to speed code execution, remove sqrt
      err_pos = math.sqrt(deltaX*deltaX + deltaY*deltaY)

      # atan2 returns value between pi, -pi in radians
      yaw = self.calc_yaw()
      desired_yaw = math.atan2( deltaY, deltaX )
      err_yaw = desired_yaw - yaw

      # ensure angles are in a range pi, -pi
      if err_yaw > math.pi:
         err_yaw -= 2 * math.pi
      elif err_yaw < -math.pi:
         err_yaw += 2 * math.pi 


      velX = self.Kpc * err_pos * math.cos(err_yaw)
      turnZ = self.Kpc * math.sin(err_yaw) * math.cos(err_yaw) + self.Kac * err_yaw

      rospy.loginfo_once("update_curve function ")
      rospy.loginfo_throttle(1, "goal position:    x: %.4f, y: %.4f" % (self.desired_position.x, self.desired_position.y))
      rospy.loginfo_throttle(1, "current position: x: %.4f, y: %.4f" % (self.current_position.x, self.current_position.y))
      rospy.loginfo_throttle(1, "desired_yaw: %.4f rad, %.4f deg" % (desired_yaw, desired_yaw*180.0/math.pi))
      rospy.loginfo_throttle(1, "yaw: %.4f rad  %.4f deg\n" % (yaw, yaw*180.0/math.pi))
      rospy.loginfo("velX: %f, turnZ: %.4f deg/sec" % (velX, turnZ*180.0/math.pi))

      
      twist_msg = Twist()
     
      if err_pos <= self.distance_precision and err_yaw <= self.yaw_precision:
         twist_msg.linear.x = 0
         twist_msg.angular.z = 0
         self.vel_pub.publish(twist_msg)
         return True
      else:
         twist_msg.linear.x = velX 
         twist_msg.angular.z = turnZ 
         self.vel_pub.publish(twist_msg)
         return False 



   def update_pid(self):

      deltaX = self.desired_position.x - self.current_position.x
      deltaY = self.desired_position.y - self.current_position.y
      current_time = rospy.get_time()
      deltaT = current_time - self.previous_time 

      # TODO: change position error to squared difference to speed code execution, remove sqrt
      err_pos = math.sqrt(deltaX*deltaX + deltaY*deltaY)

      # atan2 returns value between pi, -pi in radians
      yaw = self.calc_yaw()
      desired_yaw = math.atan2( deltaY, deltaX )
      err_yaw = desired_yaw - yaw

      # ensure angles are in a range pi, -pi
      if err_yaw > math.pi:
         err_yaw -= 2 * math.pi
      elif err_yaw < -math.pi:
         err_yaw += 2 * math.pi 
   
      err_p = err_yaw
      err_i = self.integral_error + err_p * deltaT 
      err_d = (err_yaw - self.previous_error) / deltaT


      # calculate angular velocity
      omega = self.Kp * err_p + self.Ki * err_i + self.Kd * err_d 

      # calculate translational velocity
      # as turn rate increases, translational velocity should decrease
      velX = self.max_velocity / ( math.fabs(omega) + 1)**0.5 

      # update 
      self.previous_error = err_p 
      self.integral_error = err_i 
      self.previous_time = current_time
      

      rospy.loginfo_once("update function ")
      rospy.loginfo_throttle(1, "goal position:    x: %.4f, y: %.4f" % (self.desired_position.x, self.desired_position.y))
      rospy.loginfo_throttle(1, "current position: x: %.4f, y: %.4f" % (self.current_position.x, self.current_position.y))
      rospy.loginfo_throttle(1, "desired_yaw: %.4f rad, %.4f deg" % (desired_yaw, desired_yaw*180.0/math.pi))
      rospy.loginfo_throttle(1, "yaw: %.4f rad  %.4f deg\n" % (yaw, yaw*180.0/math.pi))
      
      
      twist_msg = Twist()
     
      if err_pos <= self.distance_precision and err_yaw <= self.yaw_precision:
         twist_msg.linear.x = 0
         twist_msg.angular.z = 0
         rospy.loginfo("velX: %f, omega: %.4f deg/sec" % (velX, omega*180.0/math.pi))
         self.vel_pub.publish(twist_msg)
         return True
      else:
         twist_msg.linear.x = velX 
         twist_msg.angular.z = omega
         rospy.loginfo("velX: %f, omega: %.4f deg/sec" % (velX, omega*180.0/math.pi))
         self.vel_pub.publish(twist_msg)
         return False 

      '''
      # TODO what happens when yaw error nears pi?
      if math.fabs(err_yaw) > self.yaw_precision or err_pos > self.distance_precision:
         twist_msg.linear.x = velX
         rospy.loginfo_throttle(1, 'need yaw adustment: err_yaw %.4f deg, yaw_precision: %.4f' % (err_yaw*180.0/math.pi, self.yaw_precision))
         if err_yaw > 0:
            rospy.loginfo_throttle(1, 'rotate counter clockwise, omega: %.4f deg/sec, velX: %.4f' % (omega*180.0/math.pi, velX))
            twist_msg.angular.z = omega 
         else:
            rospy.loginfo_throttle(1, 'rotate clockwise, omega %.4f deg/sec, velX: %.4f' % (-omega*180.0/math.pi, velX))
            twist_msg.angular.z = -omega 
         self.vel_pub.publish(twist_msg)
         return False

      else:
         rospy.loginfo_throttle(1, "yaw within limits and distance within limits")
         twist_msg.linear.x = 0
         twist_msg.angular.z = 0
         self.vel_pub.publish(twist_msg)
         return True 
      '''

   def  calc_yaw(self):

      quaternion = (
         self.pose.orientation.x,
         self.pose.orientation.y,
         self.pose.orientation.z,
         self.pose.orientation.w
      )

      euler = transformations.euler_from_quaternion(quaternion)
      return euler[2]    # roll [0], pitch[1], yaw[2]

      


   def callback_odometry(self, msg):
      
      self.current_position = msg.pose.pose.position 
      self.pose = msg.pose.pose 
      


def get_goal_point():

   goal_point = Point()

   print("enter goal point")
   goal_point.x = input("x: ")
   goal_point.y = input("y: ")
   #goal_point.z = input("z: ") 

   return goal_point 


def main():

   rospy.init_node('go_to_goal')

   # local variables
   state = 0
   goal_point = Point()
   goToGoal = GoToGoal()
   
   # allow time for subscriber callbacks to begin receiving data
   rospy.sleep(2)

   # set the rate for executing the loop below
   rate = rospy.Rate(20)               # Hz

   # ros attempts to keep the loop running at this rate by accounting
   # for time used by any operations in the loop
   while not rospy.is_shutdown():
      if state == 0:
         goal_point = get_goal_point()
         rospy.loginfo("setting goal point to (%.2f, %.2f, %.2f)" % 
                     (goal_point.x, goal_point.y, goal_point.z))
         goToGoal.set_goal_point(goal_point)
         
         state = 1

      elif state == 1:
         done = goToGoal.update_pid()
         if done:
            state = 2
         rospy.loginfo_once("state is 1")

      elif state == 2:
         rospy.loginfo_once("state is 2, shutting down")
         rospy.signal_shutdown('at destination')
      else:
         rospy.logerr('Unknown state: [%d]', state)
         pass 
      rate.sleep()

if __name__ == '__main__':
   main()