#!/usr/bin/env python

# Importing all dependencies
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import tf



# Callback function to make sure that robot_1 avoid obstacles
def cb(range):

    # Stops robot_1 if it senses an object within a range of '0.5'
    if min(range.ranges[0:360]) < 0.5:
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0

        # Incrementing by 22.5 degrees
        cmd_vel.angular.z = 1.57/4
        pub = rospy.Publisher('/robot_1/cmd_vel', Twist)
        pub.publish(cmd_vel)
    

# Main function for pursuer
def pursuer_func():

    # Initiating pursuer node
    rospy.init_node('pursuer')

    # Setting the rate of cycles
    rate = rospy.Rate(10) # 5Hz

    # Initiating Tf listener
    listener = tf.TransformListener()

    # Initiating a publisher to '/robot_1/cmd_vel' so that we can publish new Twist to robot_1
    pub  =rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=1)
   

    while not rospy.is_shutdown():
        try:
            # current timestamp for future reference
            now = rospy.Time.now()

            # 2 seconds older timesstamp
            past = rospy.Time.now() - rospy.Duration(2.0)

            # Verify the prospective transfom
            listener.waitForTransformFull('robot_1/odom',now,'robot_0/odom',past,'world', rospy.Duration(1.0))

            # Ask for the specific transform with specific frames & timestamps
            (trans,rot) = listener.lookupTransformFull('robot_1/odom',now,'robot_0/odom',past,'world')

            # Calculate & publish new Twist() to robot_1, based on the incoming transforms
            cmd_vel = Twist()

            # Calculate arc tangent for angular turn
            cmd_vel.angular.z = 4*math.atan2(trans[1], trans[0])

            # Calculate linear velocity in 'x' direction for robot_1
            cmd_vel.linear.x = (0.5*math.sqrt(trans[0] ** 2 + trans[1] ** 2))/2

            # To avoid robot_1 colliding with robot_0, we stop the robot_1 if it gets too near to robot_0
            if 0.5*math.sqrt(trans[0] ** 2 + trans[1] ** 2) < 1:
                cmd_vel.linear.x = 0.0
            
            # publish new Twist() to robot_1
            pub.publish(cmd_vel)
            
        except:
            pass

    
        

if __name__ == '__main__':
    try:
        pursuer_func()
    except rospy.ROSInterruptException:
        pass