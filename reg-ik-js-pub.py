#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from amiga_moveit_wrapper.srv import GetPose,GoToPose,GoToXYZ
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import actionlib
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Quaternion
import socket
from std_msgs.msg import Float64MultiArray

class DeltaXYZSubscriberNode:
    def __init__(self):
        rospy.init_node('delta_xyz_subscriber_node', anonymous=True)

        # Init
        self.delta_x_value = 0.0
        self.delta_y_value = 0.0
        self.delta_z_value = 0.0

        # Subscribe to delta XYZ
        rospy.Subscriber('delta_x', Float64, self.delta_x_callback)
        rospy.Subscriber('delta_y', Float64, self.delta_y_callback)
        rospy.Subscriber('delta_z', Float64, self.delta_z_callback)

        self.get_eef_pose = rospy.ServiceProxy("/amiga_moveit_wrapper/get_curr_eef_pose_in_bl", GetPose)
        self.compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        self.joint_state_pub = rospy.Publisher('update_joint_states', Float64MultiArray, queue_size=10)


    def delta_x_callback(self, delta_x_msg):
        self.delta_x_value = delta_x_msg.data
        #rospy.loginfo("Received delta_x: {}".format(self.delta_x_value))

    def delta_y_callback(self, delta_y_msg):
        self.delta_y_value = delta_y_msg.data
        #rospy.loginfo("Received delta_y: {}".format(self.delta_y_value))

    def delta_z_callback(self, delta_z_msg):
        self.delta_z_value = delta_z_msg.data
        #rospy.loginfo("Received delta_z: {}".format(self.delta_z_value))

    def run(self):
        rate = rospy.Rate(10)  

        while not rospy.is_shutdown():
            # Get current EEF position
            pose_now = self.get_eef_pose.call().data.pose
            # print("current pose: ", pose_now)

            # Calculate adjusted target pose based on delta XYZ values
            target_pose_stamped = PoseStamped()
            target_pose_stamped.header.stamp = rospy.Time.now()
            target_pose_stamped.header.frame_id = "base_link"
            
            target_pose = Pose()
            target_pose.position.x = pose_now.position.x + self.delta_x_value
            target_pose.position.y = pose_now.position.y + self.delta_y_value
            target_pose.position.z = pose_now.position.z + self.delta_z_value
            target_pose.orientation = pose_now.orientation
            target_pose_stamped.pose = target_pose
            #print("target pose: ", target_pose_stamped)
            
            # Compute IK
            compute_ik_request = GetPositionIKRequest()
            compute_ik_request.ik_request.group_name = "arm_no_gripper"
            compute_ik_request.ik_request.pose_stamped = target_pose_stamped

            response = self.compute_ik_srv(compute_ik_request)

            # Print computed IK joint values
            if response.error_code.val == response.error_code.SUCCESS:
                #rospy.loginfo("IK computed success!")
                #rospy.loginfo("Computed IK Joint Values: {}".format(response.solution.joint_state.position[:6]))
                # Pub
                data=list(response.solution.joint_state.position[:6])
                data[0]+=1.0
                data[1]+=0.2
                data[2]+=-0.2
                data[3]+=-0.1
                data[4]+=0.75
                data[5]+=-0.1
                #joint_state_msg = Float64MultiArray(data)
                #joint_state_msg = Float64MultiArray(data=response.solution.joint_state.position[:6])
                joint_state_msg = Float64MultiArray()
                joint_state_msg.data = data
                self.joint_state_pub.publish(joint_state_msg)

                print("Published: ", joint_state_msg.data)
                print("-----------------")
            else:
                rospy.logwarn("Failed to compute IK: {}".format(response.error_code))

            rate.sleep()

if __name__ == '__main__':
    try:
        delta_xyz_subscriber = DeltaXYZSubscriberNode()
        delta_xyz_subscriber.run()
        
    except rospy.ROSInterruptException:
        pass
