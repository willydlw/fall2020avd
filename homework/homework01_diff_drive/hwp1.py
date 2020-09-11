'''
Objective: 
   Calculate left and right wheel velocities for a two-wheled 
   differential drive robot traveling at 1 m/s, given the robot 
   width and radius of curvature.

   Assumed rotation is a left turn.

   Program assumes user will input valid values.
'''

def getInputData():

   print("\n*** Input ***")
   bodyWidth =  float(input("Enter robot's width in meters: "))
   rCurvature = float(input("Enter the radius of curvature in meters: "))

   if bodyWidth < 0.0:
      print("\nWarning: Negative body width entered: %.2f, defaulting to 1 meter" % bodyWidth)
      bodyWidth = 1.0

   return bodyWidth, rCurvature


def main():

   robotVelocity = 1.0       # m/s
   vLeft = 0.0
   vRight = 0.0 

   print("\nObjective: calculate left and right wheel velocities for a\n"
      "two-wheeled differential drive robot traveling at %0.2f m/s.\n\n"
      "Counter-clockwise rotation about the ICC is assumed for a positive\n"
      "radius of curvature and clockwise rotation for a negative radius of\n"
      "curvature.\n\nThe robot width is measured from center of wheel to "
      "center of wheel.\n" % robotVelocity)


   while True:

      bodyWidth, rCurvature = getInputData()

      try:

         # Calculate counter-clockwise rotational velocity about the ICC
         omega = robotVelocity / rCurvature     # unit check: (m/s)/m = s^-1 which is rad/sec
         # counter-clockwise turn, right wheel is farther away from ICC 
         # unit check: rad/s * (m) = m/s
         vRight = omega * (rCurvature + bodyWidth/2.0)
         vLeft =  omega * (rCurvature - bodyWidth/2.0)

         print("\n**** Results ****\n"
               "Robot width:          %6.2f [m]" % bodyWidth)
         print("Radius of curvature:  %6.2f [m]" % rCurvature)
         print("Rotational speed:     %6.2f [rad/s]" % omega)
         print("Robot speed:          %6.2f [m/s]" % robotVelocity)
         print("Left wheel speed:     %6.2f [m/s]" % vLeft)
         print("Right wheel speed:    %6.2f [m/s]" % vRight)

      except ZeroDivisionError as error:
         print("\n** Error, zero radius of curvature. Robot will spin in place **")

      choice = input("\nEnter q to quit or just press enter to run again: ")
      if choice == 'q':
         break

   
if __name__ == '__main__':
   main()
