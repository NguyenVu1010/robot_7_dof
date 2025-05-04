#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tf

def publish_target_pose():
    rospy.init_node('send_target_pose_node')
    pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(1)

    # === Đặt tọa độ đích ở đây ===
    target_position = [0.9, 0.1, 0.5]          # x, y, z
    target_rpy = [0.0, 0.0, 1.57]              # roll, pitch, yaw (rad)

    # Chuyển Euler → Quaternion
    quat = tf.transformations.quaternion_from_euler(*target_rpy)

    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "panda_link0"   # hoặc "base_link" tùy theo robot
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.position.x = target_position[0]
    pose_msg.pose.position.y = target_position[1]
    pose_msg.pose.position.z = target_position[2]
    pose_msg.pose.orientation.x = quat[0]
    pose_msg.pose.orientation.y = quat[1]
    pose_msg.pose.orientation.z = quat[2]
    pose_msg.pose.orientation.w = quat[3]

    pub.publish(pose_msg)
    rospy.loginfo("Gửi target pose...")
    rate.sleep()

if __name__ == '__main__':
    try:
        publish_target_pose()
    except rospy.ROSInterruptException:
        pass
