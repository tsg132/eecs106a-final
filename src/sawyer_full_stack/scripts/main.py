
#!/usr/bin/env python
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
from rotate_90 import rotate_90 


def get_yaws_and_adjustments(limb, marker_ids,yaws):
    """
    Subscribes to the /ar_pose_marker topic and retrieves the yaw angles for a list of specified AR markers.
    Adjusts each yaw based on the current end-effector orientation.

    :param limb: The limb instance for the robot.
    :param marker_ids: A list of integer IDs of the AR markers whose yaws you want to extract.
    :return: A dictionary with keys as marker IDs and values as their yaw adjustments.
    """
    result = yaws

    def callback(data):
        # Don't re-calculate yaw for markers we've already found.
        found_ids = set(result.keys())
        for marker in data.markers:
            if marker.id in marker_ids and marker.id not in found_ids:
                pose = marker.pose.pose

                # Extract quaternion and convert to Euler angles
                orientation = pose.orientation
                q_original = [orientation.x, orientation.y, orientation.z, orientation.w]
                _, _, current_yaw = euler_from_quaternion(q_original)

                # Get the current end-effector yaw (from the limb's joints)
                joints = limb.joint_names()
                current_joint_angles = limb.joint_angles()
                yaw_difference = current_yaw - current_joint_angles[joints[6]]

                result[marker.id] = yaw_difference

    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    timeout = rospy.Time.now() + rospy.Duration(8.0)

    # Keep checking until we have all markers or we run out of time
    while len(result) < len(marker_ids) and rospy.Time.now() < timeout:
        rospy.sleep(0.1)

    sub.unregister()

    # If some markers weren't found, log a warning
    missing_markers = set(marker_ids) - set(result.keys())
    if missing_markers:
        rospy.logwarn("No AR markers with IDs {} found within the timeout.".format(missing_markers))

    return result



def tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        rospack = rospkg.RosPack()
        path = rospack.get_path('sawyer_full_stack')
        launch_path = path + '/launch/custom_sawyer_tuck.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        launch.start()
    else:
        print('Canceled. Not tucking the arm.')


def get_all_ar_marker_ids(timeout_sec=5.0):
    """
    Subscribes to the /ar_pose_marker topic for a specified duration and
    collects all unique AR marker IDs detected within that time.
    
    :param timeout_sec: The number of seconds to wait for markers to appear.
    :return: A list of all unique AR marker IDs detected.
    """
    marker_ids = set()

    def callback(data):
        for marker in data.markers:
            if marker.id != 4:
                marker_ids.add(marker.id)

    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    timeout = rospy.Time.now() + rospy.Duration(timeout_sec)

    while rospy.Time.now() < timeout:
        rospy.sleep(0.1)

    sub.unregister()

    return list(marker_ids)

def adjust(joint_0):
    limb = intera_interface.Limb('right')
    current_angles = limb.joint_angles()
    # Set just the first joint (right_j0) to 0.5 radians:
    current_angles['right_j0'] = joint_0
    # Command the new position
    limb.move_to_joint_positions(current_angles)



def scan(marker_ids):
    limb = intera_interface.Limb('right')

    AR_tag_pos = {}
    yaws ={}

    adjust(0.2)
    found_tags = get_all_ar_marker_ids()
    print(found_tags)
    for marker_id in get_all_ar_marker_ids():
        location = lookup_tag(marker_id,AR_tag_pos)
        print(location)
        print(AR_tag_pos)
        
    yaws = get_yaws_and_adjustments(limb, get_all_ar_marker_ids(),yaws)     

    adjust(0.1)
    for marker_id in get_all_ar_marker_ids():
        location = lookup_tag(marker_id,AR_tag_pos)
    yaws = get_yaws_and_adjustments(limb, get_all_ar_marker_ids(),yaws)     

    adjust(0)
    for marker_id in get_all_ar_marker_ids():
        location = lookup_tag(marker_id,AR_tag_pos)
    yaws = get_yaws_and_adjustments(limb, get_all_ar_marker_ids(),yaws)     

    adjust(-0.1)
    for marker_id in get_all_ar_marker_ids():
        location = lookup_tag(marker_id,AR_tag_pos)
    yaws = get_yaws_and_adjustments(limb, get_all_ar_marker_ids(),yaws)     

    adjust(-0.2)
    for marker_id in get_all_ar_marker_ids():
        location = lookup_tag(marker_id,AR_tag_pos)
    yaws = get_yaws_and_adjustments(limb, get_all_ar_marker_ids(),yaws)  
    print(AR_tag_pos)   

    return AR_tag_pos,yaws
    


def regular_tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        rospack = rospkg.RosPack()
        path = rospack.get_path('intera_examples')
        launch_path = path + '/launch/sawyer_tuck.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        launch.start()
    else:
        print('Canceled. Not tucking the arm.')

def lookup_tag(tag_number,AR_tag_pos):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """

    tfBuffer = tf2_ros.Buffer()

    tfListener = tf2_ros.TransformListener(tfBuffer)

    if tag_number in AR_tag_pos.keys():
        print("repeat")
        return

    try:
        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
        trans = tfBuffer.lookup_transform("base", f"ar_marker_{tag_number}", rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)
        print("Retrying ...")
        return

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    print(tag_pos)
    if tag_number != 4:
        tag_pos[2] = -0.01
        AR_tag_pos[tag_number] = np.array(tag_pos)
        print(AR_tag_pos)
        return 
    else:
        return np.array(tag_pos)



def get_trajectory(limb, kin, ik_solver, target, args, yaw, marker_id, use_marker_id=True):
    """
    Returns an appropriate robot trajectory. If use_marker_id is True, 
    'target' is expected to be a dictionary keyed by marker_id.
    If use_marker_id is False, 'target' is expected to be a direct position array.
    """

    num_way = args.num_way
    task = args.task

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)

    current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    print("Current Position:", current_position)

    # Decide how to get final_pos based on use_marker_id
    if use_marker_id:
        final_pos = np.copy(target[marker_id])
    else:
        final_pos = np.copy(target)
    
    trajectory = LinearTrajectory(start_position=current_position, goal_position=final_pos, total_time=9)
    
    path = MotionPath(limb, kin, ik_solver, trajectory)
    traj = path.to_robot_trajectory(num_way, True)

    # Apply yaw adjustment
    for point in traj.joint_trajectory.points:
        point.positions = list(point.positions)
        point.positions[6] += yaw

    return traj


def get_controller(controller_name, limb, kin):
    """
    Gets the correct controller from controllers.py

    Parameters
    ----------
    controller_name : string

    Returns
    -------
    :obj:`Controller`
    """
    if controller_name == 'open_loop':
        controller = FeedforwardJointVelocityController(limb, kin)
    elif controller_name == 'pid':
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        # Kw = np.array([0.9, 0, 0, 0, 0, 0, 0])
        controller = PIDJointVelocityController(limb, kin, Kp, Ki, Kd, Kw)
    else:
        raise ValueError('Controller {} not recognized'.format(controller_name))
    return controller

def grab_block(tag_pos,right_gripper,args,limb,kin,ik_solver,marker_id, yaws):

    # Get an appropriate RobotTrajectory for the task (circular, linear, or square)
    # If the controller is a workspace controller, this should return a trajectory where the
    # positions and velocities are workspace positions and velocities.  If the controller
    # is a jointspace or torque controller, it should return a trajectory where the positions
    # and velocities are the positions and velocities of each joint.

    yaw = yaws[marker_id]

    robot_trajectory = get_trajectory(limb, kin, ik_solver, tag_pos, args, yaw, marker_id, use_marker_id=True)

    # This is a wrapper around MoveIt! for you to use.  We use MoveIt! to go to the start position
    # of the trajectory
    planner = PathPlanner('right_arm')
    
    # By publishing the trajectory to the move_group/display_planned_path topic, you should 
    # be able to view it in RViz.  You will have to click the "loop animation" setting in 
    # the planned path section of MoveIt! in the menu on the left side of the screen.
    pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
    disp_traj = DisplayTrajectory()
    disp_traj.trajectory.append(robot_trajectory)
    disp_traj.trajectory_start = RobotState()
    pub.publish(disp_traj)

   
   ##GRABS BLOCK; this code works to grab the Block the tuck doesnt brign i tup butthats what we need next.  
    print("Ready to Pick UP")
    input('Press <Enter> to move to start pos')
    # Move to the trajectory start position
    plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    if args.controller_name != "moveit":
        plan = planner.retime_trajectory(plan, 0.3)
    planner.execute_plan(plan[1])

    if args.controller_name == "moveit":
        try:
            pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
            disp_traj = DisplayTrajectory()
            disp_traj.trajectory.append(robot_trajectory)
            disp_traj.trajectory_start = RobotState()
            pub.publish(disp_traj)
            input('Press <Enter> to execute the trajectory using MOVEIT')
        except KeyboardInterrupt:
            sys.exit()
        # Uses MoveIt! to execute the trajectory.
        planner.execute_plan(robot_trajectory)
        right_gripper.close()

    else:
        controller = get_controller(args.controller_name, limb, kin)
        try:
            input('Press <Enter> to execute the trajectory using YOUR OWN controller')
        except KeyboardInterrupt:
            sys.exit()
        # execute the path using your own controller.
        done = controller.execute_path(
            robot_trajectory, 
            rate=args.rate, 
            timeout=args.timeout, 
            log=args.log
        )
        
        right_gripper.close()

def place_block(drop_tag_pos,right_gripper,args,limb,kin,ik_solver):
    robot_trajectory2_drop = get_trajectory(limb, kin, ik_solver, drop_tag_pos, args, 1.5708, 0, False)
    print(drop_tag_pos)
    planner = PathPlanner('right_arm')
    
    # By publishing the trajectory to the move_group/display_planned_path topic, you should 
    # be able to view it in RViz.  You will have to click the "loop animation" setting in 
    # the planned path section of MoveIt! in the menu on the left side of the screen.
    pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
    disp_traj = DisplayTrajectory()
    disp_traj.trajectory.append(robot_trajectory2_drop)
    disp_traj.trajectory_start = RobotState()
    pub.publish(disp_traj)
    
    ## Pick Up Block 
    # Move to the trajectory start position
    input('Press <Enter> to move to start pos')
    # Move to the trajectory start position
    plan = planner.plan_to_joint_pos(robot_trajectory2_drop.joint_trajectory.points[0].positions)
    if args.controller_name != "moveit":
        plan = planner.retime_trajectory(plan, 0.3)
    planner.execute_plan(plan[1])

    print("READY TO PLACE")
    if args.controller_name == "moveit":
        try:
            pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
            disp_traj = DisplayTrajectory()
            disp_traj.trajectory.append(robot_trajectory2_drop)
            disp_traj.trajectory_start = RobotState()
            pub.publish(disp_traj)
            input('Press <Enter> to execute the trajectory using MOVEIT')
        except KeyboardInterrupt:
            sys.exit()
        # Uses MoveIt! to execute the trajectory.
        planner.execute_plan(robot_trajectory2_drop)

        print("inside of place block",drop_tag_pos)
        right_gripper.open()

    else:
        controller = get_controller(args.controller_name, limb, kin)
        try:
            input('Press <Enter> to execute the trajectory using YOUR OWN controller')
        except KeyboardInterrupt:
            sys.exit()
        # execute the path using your own controller.
        done = controller.execute_path(
            robot_trajectory2_drop, 
            rate=args.rate, 
            timeout=args.timeout, 
            log=args.log
        )
        
        right_gripper.open()

def rotate_block(drop_tag_pos,right_gripper,args,limb,kin,ik_solver):
    robot_trajectory2_drop = get_trajectory(limb, kin, ik_solver, drop_tag_pos, args, 1.5708, 0, False)
    print(drop_tag_pos)
    planner = PathPlanner('right_arm')
    
    # By publishing the trajectory to the move_group/display_planned_path topic, you should 
    # be able to view it in RViz.  You will have to click the "loop animation" setting in 
    # the planned path section of MoveIt! in the menu on the left side of the screen.
    pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
    disp_traj = DisplayTrajectory()
    disp_traj.trajectory.append(robot_trajectory2_drop)
    disp_traj.trajectory_start = RobotState()
    pub.publish(disp_traj)
    
    ## Pick Up Block 
    # Move to the trajectory start position
    input('Press <Enter> to move to start pos')
    # Move to the trajectory start position
    plan = planner.plan_to_joint_pos(robot_trajectory2_drop.joint_trajectory.points[0].positions)
    if args.controller_name != "moveit":
        plan = planner.retime_trajectory(plan, 0.3)
    planner.execute_plan(plan[1])

    print("READY TO PLACE")
    
        

def simple_place_block(place,right_gripper,args,limb,kin,ik_solver):
   
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik',timeout =  10.0)
    # rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    input('Press [Enter]:')
        # Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    request.ik_request.ik_link_name = "right_gripper_tip" 
        # request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
        # Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = place[0]
    request.ik_request.pose_stamped.pose.position.y = place[1]
    request.ik_request.pose_stamped.pose.position.z = place[2]


    request.ik_request.pose_stamped.pose.orientation.x = 0
    request.ik_request.pose_stamped.pose.orientation.y = 1
    request.ik_request.pose_stamped.pose.orientation.z = 0
    request.ik_request.pose_stamped.pose.orientation.w = 0
    try:
            # Send the request to the service
        response = compute_ik(request)
        
            # Print the response HERE
        print(response)
        group = MoveGroupCommander("right_arm")
            # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)
            # Plan IK
        plan = group.plan()
        user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            # Execute IK if safe
        if user_input == 'y':
            group.execute(plan[1], wait=True)
            right_gripper.open()
            
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



def place_at_center(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws):
    pick_up() # Replace this with pickup later
    tuck()
    
    place_position = drop_tag_pos
    
    print("Attempting to place first block at the center")
    
    grab_block(tag_pos, right_gripper, args, limb, kin, ik_solver, 0, yaws)
    
    print("Block grabbed successfully.")
    
    place_position = np.array(place_position)
    
    pick_up() # Replace this with pickup later
    tuck()

    place_block(place_position, right_gripper, args, limb, kin, ik_solver)
    
    #place_block(place_position, right_gripper, args, limb, kin, ik_solver)


    
    print("Center piece placed successfully")
    

def place_at_right(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws):
    pick_up() # Replace this with pickup later
    tuck()

    block_height = 0.05

    #this might still be bit off so I added print statment to verify 
    
    tuck()
    pick_up()


    place_position = [
        drop_tag_pos[0],
        
        drop_tag_pos[1]+block_height,
        
        -0.01
    ]
    
    print("Attempting to place first block to the right of center")
    tuck()
    
    grab_block(tag_pos, right_gripper, args, limb, kin, ik_solver, 1, yaws)
    
    print("Block grabbed successfully.")
    
    place_position = np.array(place_position)
    
    pick_up() # Replace this with pickup later
    tuck()
    rotate_block(drop_tag_pos,right_gripper,args,limb,kin,ik_solver)
    
    place_block(place_position, right_gripper, args, limb, kin, ik_solver)

    #might have to add pick_up here so it doesnt hit other blocks 
    
    
    print("right piece placed successfully")
    

def place_at_left(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws):
    tuck()
    pick_up()
    
    
    block_height = 0.06
    
    place_position = [
        drop_tag_pos[0],
        
        drop_tag_pos[1]  - block_height,
        
        -0.01
    ]
    
    print("Attempting to place first block to the right of center")

    
    grab_block(tag_pos, right_gripper, args, limb, kin, ik_solver, 2, yaws)
    
    print("Block grabbed successfully.")
    
    place_position = np.array(place_position)
    
    pick_up() # Replace this with pickup later-0.145
    tuck()

    rotate_block(drop_tag_pos,right_gripper,args,limb,kin,ik_solver)
    
    place_block(place_position, right_gripper, args, limb, kin, ik_solver)
    
    
    print("left piece placed successfully")
    

def place_skew_right(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws):
    
    tuck()
    pick_up()
    block_height = 0.05
    base_z_level = -0.145
    
    place_position = [
        drop_tag_pos[0],
        
        drop_tag_pos[1]  + (block_height ) / 2,
        
        0.04  #i think this should be -0.145 plus or minus block height 
    ]
    
    print("Attempting to place the block at the higher level")
    
    grab_block(tag_pos, right_gripper, args, limb, kin, ik_solver, 5, yaws)
    
    pick_up()

    
    rotate_block(drop_tag_pos,right_gripper,args,limb,kin,ik_solver)
    
    # place_block(place_position, right_gripper, args, limb, kin, ik_solver)
    place_block(place_position, right_gripper, args, limb, kin, ik_solver)
    
    print("Placed successfully")
    

def place_skew_left(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws):
    
    tuck()
    pick_up()
    block_height = 0.05
    base_z_level = -0.145
    
    place_position = [
        drop_tag_pos[0] ,
        
        drop_tag_pos[1] - (block_height) / 2,
        
       0.04
    ]
    
    print("Attempting to place the block at the higher level")
    
    grab_block(tag_pos, right_gripper, args, limb, kin, ik_solver, 7, yaws)
    
    pick_up()

    rotate_block(drop_tag_pos,right_gripper,args,limb,kin,ik_solver)
    
    # place_block(place_position, right_gripper, args, limb, kin, ik_solver)
    place_block(place_position, right_gripper, args, limb, kin, ik_solver)

    
    print("Placed successfully")
    

def level1_horizontal_stack(drop_tag_pos,tag_pos,right_gripper,args,limb,kin,ik_solver, marker_ids, yaws):
    
    place_at_center(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws)
    
    rospy.sleep(0.5)
    
    place_at_right(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws)
    
    rospy.sleep(0.5)
    
    place_at_left(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws)
    
    rospy.sleep(0.5)
    
def level2_horizontal_stack(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws):
    
    place_skew_right(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws)
    
    rospy.sleep(0.5)
    
    place_skew_left(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws)
    
    rospy.sleep(0.5)
    
    
def final_stack(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws):
    pick_up()
    block_height = 0.05
    
    place_position = [
        
        drop_tag_pos[0],
        
        drop_tag_pos[1],
        
        -0.01 +2*block_height
    ]
    
    grab_block(tag_pos, right_gripper, args, limb, kin, ik_solver, 8, yaws)
    
    pick_up()

    rotate_block(drop_tag_pos,right_gripper,args,limb,kin,ik_solver)
    
    place_block(place_position, right_gripper, args, limb, kin, ik_solver)
    
    
    print("Pyramid complete")

def vertical(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws):
    i = 0
    for marker in tag_pos.keys():
        tuck()
        pick_up()
        block_height = 0.05
        base_z_level = -0.145
        
        place_position = [
            drop_tag_pos[0],
            
            drop_tag_pos[1],
            
            -0.01 + block_height*i  #i think this should be -0.145 plus or minus block height 
        ]
        
        print("Attempting to place the block at the higher level")
        
        grab_block(tag_pos, right_gripper, args, limb, kin, ik_solver, marker, yaws)
        
        pick_up()

        
        rotate_block(drop_tag_pos,right_gripper,args,limb,kin,ik_solver)
        
        # place_block(place_position, right_gripper, args, limb, kin, ik_solver)
        place_block(place_position, right_gripper, args, limb, kin, ik_solver)
        i+=1
        print("Placed successfully")    
    
    




def main():
    """
    Examples of how to run me:
    python scripts/main.py --help <------This prints out all the help messages
    and describes what each parameter is
    python scripts/main.py -t line -ar_marker 3 -c torque --log
 
    You can also change the rate, timeout if you want
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='line', help=
        'Options: li6ne, circle.  Default: line'
    )

    parser.add_argument('-stack', '-s', type=str, default='py', help=
        'Options: py, v.  Default: py'
    )

    #change this to be able to get more than one input here
    parser.add_argument('-ar_marker', '-ar', nargs='+', help=
        'Which AR marker to use.  Default: 1'
    )

    parser.add_argument('-controller_name', '-c', type=str, default='moveit', 
        help='Options: moveit, open_loop, pid.  Default: moveit'
    )
    parser.add_argument('-rate', type=int, default=200, help="""
        This specifies how many ms between loops.  It is important to use a rate
        and not a regular while loop because you want the loop to refresh at a
        constant rate, otherwise you would have to tune your PD parameters if 
        the loop runs slower / faster.  Default: 200"""
    )
    parser.add_argument('-timeout', type=int, default=None, help=
        """after how many seconds should the controller terminate if it hasn\'t already.  
        Default: None"""
    )
    parser.add_argument('-num_way', type=int, default=50, help=
        'How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 300'
    )
    parser.add_argument('--log', action='store_true', help='plots controller performance')
    args = parser.parse_args()


    rospy.init_node('moveit_node')
#step 1: TUCK to view all items on table     
    tuck()

    AR_tag_pos = {}
    # always the Center
    drop_tag_pos = lookup_tag(4,AR_tag_pos)
    #drop_tag_pos = [0.6512452527405076, 0.15575391244005002 + (0.05) / 2, 0.04]
    drop_tag_pos[2] = -0.01
    print("tag for drop:",drop_tag_pos)


    #calibrate the gripper
    right_gripper = robot_gripper.Gripper('right_gripper')
    print('Calibrating...')
    right_gripper.calibrate()
    right_gripper.open()
    
    # this is used for sending commands (velocity, torque, etc) to the robot "right_gripper_tip"stp_022312TP99620_tip_1
    ik_solver = IK("base", "right_gripper_tip")
    limb = intera_interface.Limb("right")
    kin = sawyer_kinematics("right")

    

    # tag_pos = {marker_id: lookup_tag(marker_id) for marker_id in marker_ids}
    tag_pos,yaws = scan(marker_ids)
    print(tag_pos)
    print(yaws)
       

    
    level1_tag_pos = {
        0: tag_pos.get(0),
        
        1: tag_pos.get(1),
        
        2: tag_pos.get(2)
    }
    
    level2_tag_pos = {
        
        5: tag_pos.get(5),
        
        7: tag_pos.get(7)
    }
    
    final_level = {
        
        8: tag_pos.get(8)
    }
    
    # Gets the yaws of all the relevant markers.

    if args.stack == "py":
        level1_horizontal_stack(drop_tag_pos, level1_tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws)
        
        level2_horizontal_stack(drop_tag_pos, level2_tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws)
        
        final_stack(drop_tag_pos, final_level, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws)
    if args.stack == "v":
        vertical(drop_tag_pos, tag_pos, right_gripper, args, limb, kin, ik_solver, marker_ids, yaws)

if __name__ == "__main__":
    main()


 