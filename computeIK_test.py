#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from amiga_moveit_wrapper.srv import GetPose,GoToPose,GoToXYZ
from moveit_msgs.srv import GetPositionIK
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import PointStamped
import actionlib
from sensor_msgs.msg import JointState
import socket

SEED_STATE = {
    "amiga_arm_elbow_joint": -1.2511982917785645, 
    "amiga_arm_shoulder_lift_joint": -2.976866384545797, 
    "amiga_arm_shoulder_pan_joint": -1.2123454252826136, 
    "amiga_arm_wrist_1_joint": 2.882155104274414, 
    "amiga_arm_wrist_2_joint": 0.8456740379333496, 
    "amiga_arm_wrist_3_joint": -2.84684664407839
    }


class DeltaXYZSubscriberNode:
    # def __init__(self):
    def  __init__(self, target_pose) -> None:
        rospy.init_node('delta_xyz_subscriber_node', anonymous=True)

        # Initialize instance variables to store delta values
        self.delta_x_value = 0.0
        self.delta_y_value = 0.0
        self.delta_z_value = 0.0

        # Subscribe to the delta XYZ topics and store the received values
        rospy.Subscriber('delta_x', Float64, self.delta_x_callback)
        rospy.Subscriber('delta_y', Float64, self.delta_y_callback)
        rospy.Subscriber('delta_z', Float64, self.delta_z_callback)

        # get current eef pose
        self.get_eef_pose = rospy.ServiceProxy("/amiga_moveit_wrapper/get_curr_eef_pose_in_bl", GetPose)
        # ik compute
        self.compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)


    def delta_x_callback(self, delta_x_msg):
        self.delta_x_value = delta_x_msg.data
        rospy.loginfo("Received delta_x: {}".format(self.delta_x_value))

    def delta_y_callback(self, delta_y_msg):
        self.delta_y_value = delta_y_msg.data
        rospy.loginfo("Received delta_y: {}".format(self.delta_y_value))

    def delta_z_callback(self, delta_z_msg):
        self.delta_z_value = delta_z_msg.data
        rospy.loginfo("Received delta_z: {}".format(self.delta_z_value))

    def action_adjust(self):
        # Get current eef position
        pose_now = self.get_eef_pose.call().data.pose
        # print("current pose:")
        rospy.loginfo("\n Current pose:\n{}".format(pose_now))
   
        # Target object position (after correction)
        # FAKE delta xyz
        self.delta_x_value = 0.1
        self.delta_y_value = 0.0
        self.delta_z_value = 0.1

        target_pose = pose_now
        target_pose.position.x = pose_now.position.x + self.delta_x_value
        target_pose.position.y = pose_now.position.y + self.delta_y_value
        target_pose.position.z = pose_now.position.z + self.delta_z_value
     
        rospy.loginfo("Target pose:\n{}".format(target_pose))

        # Compute ik
        response = self.compute_ik_srv.call[target_pose.position.x, target_pose.position.y, target_pose.position.z]
        

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        delta_xyz_subscriber = DeltaXYZSubscriberNode()
        delta_xyz_subscriber.action_adjust()
        delta_xyz_subscriber.run()
    except rospy.ROSInterruptException:
        pass


