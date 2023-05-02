#!/usr/bin/env python3
"""
.
"""
import tf
import rospy
import numpy as np
import numpy.typing as npt
from geometry_msgs.msg import Pose, PoseStamped


class LastPose:
    """
    Last pose of the vehicle
    """

    def __init__(self) -> None:
        self.pose = np.array([0, 0, 0])

    def update(self, currentState: Pose) -> None:
        """
        Update the current state of the vehicle
        """
        poseMsg.pose = currentState
        self.pose = np.array(
            [currentState.position.x, currentState.position.y, currentState.orientation.w]
        )


rospy.init_node("transformer")
framedPose = rospy.Publisher("/tf_pose", PoseStamped, queue_size=10)
transPose = LastPose()
rospy.Subscriber("/carmaker/odom", Pose, callback=transPose.update)

# Broadcast tf
while rospy is not rospy.is_shutdown:
    poseMsg = PoseStamped()
    br = tf.TransformBroadcaster()
    yaw = transPose.pose[2]
    s, c = np.sin(yaw), np.cos(yaw)
    rot_mat = np.array([[c, s], [-s, c]])
    pose = rot_mat @ (transPose.pose[:2].reshape(2, 1))

    br.sendTransform(
        (-pose[0][0], -pose[1][0], 0),
        tf.transformations.quaternion_from_euler(0, 0, -yaw),
        rospy.Time.now(),
        "map",
        "local",
    )
    poseMsg.header.frame_id = "map"
    poseMsg.header.stamp = rospy.Time.now()

    framedPose.publish(poseMsg)
