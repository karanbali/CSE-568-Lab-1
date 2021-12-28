#!/usr/bin/env python

# Importing all dependencies
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import geometry_msgs
from geometry_msgs.msg import Twist
import tf



# Callback function to publish the new Twist() to robot_0
# It also makes sure that robot_0 avoid obstacles
def cb(range):

    # Finding a new random direction to turn as an 'object' is probably closer that '0.6'
    if min(range.ranges[0:360]) < 0.6:
        cmd_vel = Twist()

        # Incrementing by 22.5 degrees
        cmd_vel.angular.z = 1.57/4
        pub = rospy.Publisher('/robot_0/cmd_vel', Twist)
        pub.publish(cmd_vel)
    
    # Drive straight with a constant speed of 2 m/s.
    else:
        cmd_vel = Twist()
        cmd_vel.linear.x = 2
        pub = rospy.Publisher('/robot_0/cmd_vel', Twist)
        pub.publish(cmd_vel)


# Callback function to broadcast Tf transformation b/w 'robot_0/odom' & 'world'
def cb_tr(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),rospy.Time.now(), 'robot_0/odom',"world")

# Callback function to broadcast Tf transformation b/w 'robot_1/odom' & 'world'
def cb_tr_1(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),rospy.Time.now(), 'robot_1/odom',"world")
    


# Main function for evader
def evader_func():

    # Initiating evader node
    rospy.init_node('evader')

    # Setting the rate of cycles
    rate = rospy.Rate(10) # 5Hz

    # Initiating Tf broadcaster
    bs = tf.TransformBroadcaster()

    # Setting the statics transform b/w 'world' & 'robot_1/odom'
    bs.sendTransform((0.0,0.0,0.0),(0.0,0.0,0.0,1.0),rospy.Time.now(),'world','robot_1/odom')
    
    # Subscribing to '/robot_0/base_scan' to get its scan readings with a callback function 'cb'
    cmd_vel = rospy.Subscriber('/robot_0/base_scan', LaserScan, cb, queue_size=20)

    # Subscribing to '/robot_0/base_pose_ground_truth' to get its odometry readings with a callback function 'cb_tr'
    tr = rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, cb_tr, queue_size=20)

    # Subscribing to '/robot_1/base_pose_ground_truth' to get its odometry readings with a callback function 'cb_tr_1'
    tr_1 = rospy.Subscriber('/robot_1/base_pose_ground_truth', Odometry, cb_tr_1, queue_size=20)

    # Keep ROS from dying
    rospy.spin()

    

if __name__ == '__main__':
    try:
        evader_func()
    except rospy.ROSInterruptException:
        pass