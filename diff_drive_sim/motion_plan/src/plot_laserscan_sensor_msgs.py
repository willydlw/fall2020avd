#! /usr/bin/env python
import rospy

import matplotlib.pyplot as plt
import numpy as np 

from sensor_msgs.msg import LaserScan 


class AvoidObstacle:

   def __init__(self):

      # initialize publishers and subscribers here
      self.laser_subscriber = rospy.Subscriber('/m2wr/laser/scan', LaserScan,self.laser_callback, queue_size=1)
      

   def laser_callback(self, msg):
      rospy.loginfo_once("frame id: %s", msg.header.frame_id)
      rospy.loginfo_once("angle_min: %f", msg.angle_min)
      rospy.loginfo_once("angle_max: %f", msg.angle_max)
      rospy.loginfo_once("angle_increment: %f: ", msg.angle_increment)
      rospy.loginfo_once("range_min: %f", msg.range_min)
      rospy.loginfo_once("range_max: %f", msg.range_max)
      rospy.loginfo_once('len(msg.ranges): %d', len(msg.ranges))

      # create an array of angles for plotting
      theta = np.arange(msg.angle_min, msg.angle_max+msg.angle_increment, msg.angle_increment)

      plt.polar(theta, msg.ranges)
      plt.title('Laser Scan Polar Plot, max range: ' + str(msg.range_max))
      plt.show(block=False)

def main():
    
   rospy.init_node('avoid_obstacle')
   ao = AvoidObstacle()
   rospy.spin()


if __name__ == "__main__":
   main()
