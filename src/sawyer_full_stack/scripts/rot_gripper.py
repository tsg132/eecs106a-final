#!/usr/bin/env python3

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import quaternion_from_euler, quaternion_inverse, euler_from_quaternion
from std_msgs.msg import Float64  # Assuming the gripper joint takes Float64 commands
from intera_interface import gripper as robot_gripper


# Publisher to control the 7th joint (gripper joint)

# Maybe change this.

"""

The yaw that we get from find_rot.py, is the rotation around z axis, that will make the 7th joint align with the block before picking the block up.

We apply this transformation around z to the 7th joint AFTER RUNNING THE CODE WE WROTE FOR CHECK_POINT 1. This make the end effector aligned with the cube. Later,
once we grip the cube, we apply another transformation to yaw 0.0, which makes the cube aligned with 0 yaw, which is what we want. Then we'll develop the algorithm 
to place the cube somewhere else in the table and so on (todo later). find_rot looks to be working fine, but I haven't tested this code, so you should do any fixes that are necessary
and move the code to final_project directory, which is where we implemented the code for the 1st checkpoint. If the code for the checkpoint 1 isn't there, then it is in the lab 7 directory.
We might have updated lab 7 to finish off the checkpoint 1, I am not sure. Nevertheless, I feel like this is enough progress for checkpoint 2. Best
"""
"We dont need to publish where we simply need to setjoint angle ???? a function intera_intera ??"

gripper_pub = rospy.Publisher('/robot/limb/right/joint_command', Float64, queue_size=10)
import intera_interface
limb = intera_interface.Limb(6)
def rotate_gripper(yaw):
    """
    Rotate the gripper to align with the AR tag using the yaw angle.
    """
    joint_angles = limb.joint_angles()
    joint_angles[6] = yaw  # Adjust joint name if needed
    limb.set_joint_positions(joint_angles)

def reset_gripper(yaw):
    """
    Apply the inverse transformation to return the gripper to its original orientation.
    """
    rospy.loginfo(f"Resetting gripper to inverse yaw: {-yaw:.2f} radians")
    gripper_pub.publish(0.0)  # Return to original position

def callback(data):
    for marker in data.markers:
        marker_id = marker.id
        pose = marker.pose.pose

        # Extract orientation as quaternion
        orientation = pose.orientation
        q_original = [orientation.x, orientation.y, orientation.z, orientation.w]

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = euler_from_quaternion(q_original)
        rospy.loginfo(f"Marker ID: {marker_id}")
        rospy.loginfo(f"Current Yaw (z-rotation): {yaw:.2f} radians")

        # Rotate the gripper to align with the AR tag
        rotate_gripper(yaw)

def listener():
    rospy.init_node('gripper_rotation_node', anonymous=True)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
