#!/usr/bin/env python

import math
import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position

# Current goal (global state)
goal = None

def goal_callback(msg):
    global goal

    # RViz's "2D Nav Goal" publishes z=0, so add some altitude if needed.
    if msg.pose.position.z == 0.0:
        msg.pose.position.z = 0.4

    rospy.loginfo('New goal set:\n%s', msg)
    goal = msg

def publish_cmd(goal):
    cmd = Position()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = 'map'
    # cmd.x = -3.45
    # cmd.y = 0.75
    # cmd.z =  0.1
    # cmd.yaw = 90

    cmd.x = 3.45
    cmd.y = 0.75
    cmd.z =  0.1
    cmd.yaw = 90


    pub_cmd.publish(cmd)


rospy.init_node('node2')
sub_goal = rospy.Subscriber('/cf1/pose', PoseStamped, goal_callback)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

def main():
    rate = rospy.Rate(500)  # Hz
    while not rospy.is_shutdown():
        if goal:
            publish_cmd(goal)
        rate.sleep()

if __name__ == '__main__':
    main()

