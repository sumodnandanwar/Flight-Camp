#!/usr/bin/env python

import math
import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position


def publish_cmd1():
    rospy.loginfo("In loop 1")
    cmd = Position()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = 'map'
    cmd.x = -4.0
    cmd.y = 1.0
    cmd.z =  0.15
    cmd.yaw = 0.0  
    pub_cmd.publish(cmd)

def publish_cmd2():
    rospy.loginfo("In loop 2")

    cmd1 = Position()
    cmd1.header.stamp = rospy.Time.now()
    cmd1.header.frame_id = 'map'
    cmd1.x = 4.0
    cmd1.y = 1.0
    cmd1.z =  0.15
    cmd1.yaw = 0.0
    
    pub_cmd.publish(cmd1)

def publish_cmd3():
    rospy.loginfo("In loop 3")

    cmd = Position()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = 'map'
    cmd.x = 3.0
    cmd.y = -1.0
    cmd.z =  0.15
    cmd.yaw = 180
    
    pub_cmd.publish(cmd)

def publish_cmd4():
    rospy.loginfo("In loop 4")

    cmd = Position()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = 'map'
    cmd.x = -2.5
    cmd.y = -1.0
    cmd.z =  0.15
    cmd.yaw = 180
    
    pub_cmd.publish(cmd)

rospy.init_node('node22')
#sub_goal = rospy.Subscriber('/cf1/pose', PoseStamped, goal_callback)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

def main():
    #rate = rospy.Rate(1)  # Hz
    pre_time = 0
    while not rospy.is_shutdown():
        #rospy.Time.now().secs= 0

        rospy.loginfo(rospy.Time.now().secs)

        #post_time = rospy.Time.now().secs
        #change_time = post_time - pre_time
        if rospy.Time.now().secs < 65 :
            publish_cmd1()
            # rospy.loginfo("In loop 1")
            # rospy.loginfo(rospy.Time.now().secs)
            
        elif rospy.Time.now().secs >= 65 and rospy.Time.now().secs < 75:
            publish_cmd2()
            # rospy.loginfo("In loop 2")

        elif rospy.Time.now().secs >= 75 and rospy.Time.now().secs < 85:
            publish_cmd3()
            # rospy.loginfo("In loop 3")

        elif rospy.Time.now().secs >= 85 and rospy.Time.now().secs < 90:
            publish_cmd4()
            # rospy.loginfo("In loop 4")


        #pre_time = post_time
        
    #rate.sleep()

if __name__ == '__main__':
    main()

