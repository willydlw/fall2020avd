#! /usr/bin/env python
import rospy
import numpy as np
import math
import matplotlib
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist

class AvoidObstacle:

   # parameters
   SCAN_TOPIC = rospy.get_param("/avoid_obstacle/scan_topic")
   NUM_MEASUREMENTS = rospy.get_param("/avoid_obstacle/num_measurements")
   ANGLE_MIN = rospy.get_param("/avoid_obstacle/angle_min")                   # rad
   ANGLE_MAX = rospy.get_param("/avoid_obstacle/angle_max")                   # rad
   ANGLE_INCREMENT = rospy.get_param("/avoid_obstacle/angle_increment")       # rad
   RANGE_MIN = rospy.get_param("/avoid_obstacle/range_min")                   # meter, m
   RANGE_MAX = rospy.get_param("/avoid_obstacle/range_max")                   # meter, m
   DISTANCE_THRESHOLD = rospy.get_param("/avoid_obstacle/distance_threshold") # meter, m

   def __init__(self):

      # initialize publishers and subscribers here
      self.laser_subscriber = rospy.Subscriber('/m2wr/laser/scan', LaserScan,self.laser_callback, queue_size=1)
      
      #self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
      #sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)

      # pre-compute sin,cos values as they will be the same for each scan
      self.cosVal = np.cos(np.arange(self.ANGLE_MIN, self.ANGLE_MAX, self.ANGLE_INCREMENT))
      self.sinVal = np.sin(np.arange(self.ANGLE_MIN, self.ANGLE_MAX, self.ANGLE_INCREMENT))



   def laser_callback(self, msg):
      
      # apply threshold filter
      rospy.loginfo_throttle(1, "distance threshold: %f", self.DISTANCE_THRESHOLD)
      filteredRanges = np.zeros(len(msg.ranges))

      for i in range(len(msg.ranges)):
         if msg.ranges[i] <= self.DISTANCE_THRESHOLD:
            filteredRanges[i] = msg.ranges[i]
         else:
            filteredRanges[i] = self.DISTANCE_THRESHOLD

      # laser frame coordinates
      # convert polar to cartesian coordinates: radius, angle to x,y
      xval = (np.asarray(filteredRanges).T * self.cosVal).T
      yval = (np.asarray(filteredRanges).T * self.sinVal).T

      # sum the x and y coordinates
      xsum = np.sum(xval)
      ysum = np.sum(yval)

      # calculate avoid obstacle heading
      aoTheta = math.atan2(ysum,xsum)

      # determine x,y of obstacle avoidance heading unit vector
      xheading = math.cos(aoTheta)
      yheading = math.sin(aoTheta)

      # verify unit vector
      magnitudeNorm = math.sqrt(xheading*xheading + yheading*yheading)

      rospy.loginfo_throttle(1, "xheading: %.2f, yheading: %.2f, angle: %.2f degrees", 
               xheading, yheading, aoTheta*180.0/math.pi)


      # create an array of angles for plotting
      scanAngles = np.arange(msg.angle_min, msg.angle_max+msg.angle_increment, msg.angle_increment)

      # radar green, solid grid lines
      plt.rc('grid', color='#316931', linewidth=1, linestyle='-')
      plt.rc('xtick', labelsize=15)
      plt.rc('ytick', labelsize=15)

      # force square figure and square axes looks better for polar, IMO
      width, height = matplotlib.rcParams['figure.figsize']
      size = min(width, height)
      # make a square figure
      fig = plt.figure(figsize=(size, size))
      ax = fig.add_axes([0.1, 0.1, 0.8, 0.8], polar=True, axisbg='#d5de9c')

      ax.plot(scanAngles, filteredRanges)
      plt.grid(True)
      plt.arrow(0, 0, aoTheta*180.0/np.pi, 1, width = 0.15,
                 edgecolor = 'black', facecolor = 'green', lw = 2, zorder = 5)
      plt.title('Laser Scan Polar Plot, max range: ' + str(msg.range_max))
      plt.show(block=False)

def myshutdown():
   rospy.loginfo_once('myshutdown triggered')


def main():
    
   rospy.init_node('avoid_obstacle')

   ao = AvoidObstacle()
   rospy.on_shutdown(myshutdown)

   while not rospy.is_shutdown():
      try:
         rospy.spin()
      except rospy.ROSInterruptException:
         print('avoid_obstacle shutting down')
   

if __name__ == "__main__":
   main()
