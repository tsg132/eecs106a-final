import sys
import argparse
import numpy as np
import rospkg
import roslaunch
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander

from paths.trajectories import LinearTrajectory, CircularTrajectory
from paths.paths import MotionPath
from paths.path_planner import PathPlanner
from controllers.controllers import ( 
    PIDJointVelocityController, 
    FeedforwardJointVelocityController
)
from utils.utils import *

from trac_ik_python.trac_ik import IK

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from intera_interface import Limb
from functools import partial

import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics

from intera_interface import gripper as robot_gripper
from pick_up import pick_up
from main import grab_block,place_block
from pick_up import pick_up






def stack_blocks(drop_tag_pos,tag_pos,right_gripper,args,limb,kin,ik_solver):
    """
    Function to stack blocks vertically.

    Parameters:
        mvit (function): Function that detects and grasps blocks autonomously.
        stack_base (tuple): Base position of the stack (x, y, z).
        block_height (float): Height of each block.
        num_blocks (int): Total number of blocks to stack.
    """
    # Initialize the stack height
    current_stack_height = 0
    block_height = 0.5#this is value we have to adjust 
    num_blocks = tag_pos.size 

    for i in range(num_blocks):
        print(f"Attempting to grasp block {i+1}...")
        grab_block(tag_pos,right_gripper,args,limb,kin,ik_solver,i)

      

        #Call pick up block on first ar tag

        print(f"Block {i+1} grasped successfully.")

        # Step 2: Calculate the placement position
        place_position = (
            drop_tag_pos[0[0]],  # x-coordinate remains the same
            drop_tag_pos[0[1]],  # y-coordinate remains the same
            drop_tag_pos[0[2]] + current_stack_height*block_height # Incremental z-coordinate
        )
        print(f"Moving to stack position {place_position}...")
        #place_block(drop_tag_pos,right_gripper,args,limb,kin,ik_solver)

        place_block(place_position,right_gripper,args,limb,kin,ik_solver)

        # Step 3: Place the block
        print(f"Block {i+1} placed successfully.")

        # Update stack height
        current_stack_height += block_height

    print("Stacking complete!")