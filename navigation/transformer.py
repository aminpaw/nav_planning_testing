#!/usr/bin/env python3
import tf
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
import tf_helper
from tf2_geometry_msgs import PoseStamped
last_pose = np.array([0,0,0])
poseMsg = PoseStamped()
def update(currentState:Pose):
    global last_pose
    poseMsg.pose = currentState
    last_pose[0] = currentState.position.x
    last_pose[1] = currentState.position.y
    last_pose[2] = currentState.orientation.w   

rospy.init_node("transformer")
rospy.Subscriber("/carmaker/odom", Pose, callback=update)
framedPose = rospy.Publisher("/tf_pose", PoseStamped, queue_size=10)

# Broadcast tf
while (rospy is not rospy.is_shutdown):
    br = tf.TransformBroadcaster()
    yaw = last_pose[2]
    s, c = np.sin(yaw), np.cos(yaw)
    rot_mat = np.array([[c,s],[-s,c]])
    pose = rot_mat@(last_pose[:2].reshape(2,1))

    br.sendTransform((-pose[0][0], -pose[1][0], 0),
    tf.transformations.quaternion_from_euler(0, 0, -yaw),
    rospy.Time.now(),
    "map","local")
    poseMsg.header.frame_id = "map"
    poseMsg.header.stamp = rospy.Time.now()

    framedPose.publish(poseMsg)