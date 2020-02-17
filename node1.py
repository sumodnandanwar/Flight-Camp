#!/usr/bin/env python

import sys
import math
import json

import rospy
import tf2_ros
import tf2_geometry_msgs 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from aruco_msgs.msg import MarkerArray


def goal_callback(aruco):
    for m in aruco.markers:
        tf = publish_marker(m)
        if tf:
            broadcaster.sendTransform(tf)
    
def publish_marker(m):
    global broadcaster, tf_buf

    #for m in goal.markers:
    marker_t = PoseStamped()
    marker_t.pose = m.pose.pose
    marker_t.header.frame_id = 'camera_link'
    marker_t.header.stamp = rospy.Time.now()

    #Check if tranform is avaliable 
    if not tf_buf.can_transform(marker_t.header.frame_id, 'map', marker_t.header.stamp):
        rospy.logwarn_throttle(5.0, 'No transform from %s to map' % marker_t.header.frame_id)
        return

    marker_map = tf_buf.transform (marker_t, 'map' ) 

    marktf = TransformStamped()
    marktf.header.frame_id = 'map'
    marktf.child_frame_id = 'aruco/detected' + str(m.id)
    marktf.transform.translation = marker_map.pose.position
    marktf.transform.rotation = marker_map.pose.orientation
    
    #broadcaster.sendTransform(marktf)
    return marktf


#def main():
   # rate = rospy.Rate(10)  # Hz
   # while not rospy.is_shutdown():
  #      if goal:
   #         publish_marker(goal)
   # rate.sleep()

if __name__ == "__main__":
    rospy.init_node('node1')
    aruco_sub = rospy.Subscriber('/aruco/markers', MarkerArray, goal_callback)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    tf_buf   = tf2_ros.Buffer()
    tf_lstn  = tf2_ros.TransformListener(tf_buf)
    rospy.spin()
    