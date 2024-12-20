#!/usr/bin/env python3

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from intera_interface import Limb
from functools import partial

def callback(data, limb, yaw):
    for marker in data.markers:

        joints = limb.joint_names()
        marker_id = marker.id
        pose = marker.pose.pose

        # Extract the orientation as a quaternion
        orientation = pose.orientation
        q_original = [orientation.x, orientation.y, orientation.z, orientation.w]

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, current_yaw = euler_from_quaternion(q_original)

        # Transformation: From zero yaw to current yaw
        initial_yaw = 0.0  # Original yaw (default orientation)
        yaw_difference = current_yaw - initial_yaw

        # rospy.loginfo(f"Marker ID: {marker_id}")
        # rospy.loginfo(f"Current Yaw: {current_yaw:.2f} radians")
        # rospy.loginfo(f"Yaw Difference (Transformation): {yaw_difference:.2f} radians")

        # Rotate the 7th joint (wrist)
        try:
            joint_name = joints[6]  # Replace with actual name
            current_joint_angles = limb.joint_angles()

            # rospy.loginfo(f"Current joint positions: {current_joint_angles}")
            yaw = yaw_difference
            # rospy.loginfo(f"Setting {joint_name} to new angle: {new_angle:.2f}")

            # # limb.set_joint_positions({joint_name: new_angle})

            # rospy.loginfo(f"Rotated {joint_name} to yaw angle {new_angle:.2f} radians.")
        except Exception as e:
            rospy.logerr(f"Failed to set joint position for {joint_name}: {e}")


def listener(limb):
    """
    Gets a single reading of the AR marker's yaw angle.
    """
    global yaw_result, marker_found
    
    # Reset globals
    yaw_result = None
    marker_found = False
    
    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, partial(callback, limb=limb, yaw=yaw_result))
    
    # Wait for up to 5 seconds to get a reading
    timeout = rospy.Time.now() + rospy.Duration(5.0)
    while not marker_found and rospy.Time.now() < timeout:
        rospy.sleep(0.1)
    
    # Unsubscribe once we're done
    sub.unregister()
    
    return yaw_result

# def listener():
#     """
#     Subscribes to AR markers and aligns the robot's gripper based on the yaw difference.
#     Expects rospy.init_node() to be called externally.

#     #     rospy.init_node('ar_tag_yaw_transform', anonymous=True)
#     """
#     # rospy.init_node('ar_tag_yaw_transform', anonymous=True)
#     limb = Limb()

#     # Pass the limb object to the callback using partial
#     rospy.Subscriber("/ar_pose_marker", AlvarMarkers, partial(callback, limb=limb))
#     rospy.sleep(2)  # Allow time for the subscriber to process

# if __name__ == '__main__':
#     listener()
