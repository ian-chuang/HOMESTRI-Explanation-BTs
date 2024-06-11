import rospy
import moveit_commander
from xbt_planning_interface.srv import SimplePlan, SimplePlanRequest, SimplePlanResponse
import moveit_msgs.msg
import geometry_msgs.msg
import math
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from scipy.spatial.transform import Rotation as R

def generate_circle(current_pose, center_point, target_pose, rotation_step_size=0.02):
    current_position = np.array([
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z
    ])
    current_orientation = np.array([
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    ])
    center = np.array([
        center_point.x,
        center_point.y,
        center_point.z
    ])
    target_position = np.array([
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z
    ])  

    # Calculate vectors
    start_vector = np.array(current_position) - np.array(center)
    end_vector = np.array(target_position) - np.array(center)

    # Calculate the rotation axis
    rotation_axis = np.cross(start_vector, end_vector)
    if np.linalg.norm(rotation_axis) == 0:
        # if the rotation axis is zero, the start_vector and end_vector are collinear
        rospy.logerr("The start_vector and end_vector are collinear")
        return None
    rotation_axis /= np.linalg.norm(rotation_axis)

    # given rotation_axis, start_vector, end_vector, calculate the shortest angle
    rotation_step_size = 0.05
    angle = np.arccos(np.dot(start_vector, end_vector) / (np.linalg.norm(start_vector) * np.linalg.norm(end_vector)))

    poses = []
    for rot in np.flip(np.arange(angle, 0, -rotation_step_size)):
        # Create rotation matrix
        rot_matrix = R.from_rotvec(rotation_axis * rot).as_matrix()
        # Apply rotation matrix to start_vector
        rotated_vector = np.dot(rot_matrix, start_vector)

        # rotate starting_pose quat around rotation_axis
        starting_rot = R.from_quat(current_orientation)
        rotated_rot = R.from_rotvec(rotation_axis * rot) * starting_rot

        # Create pose
        pose = np.concatenate([center + rotated_vector, rotated_rot.as_quat()])
        pose_msg = geometry_msgs.msg.Pose()
        pose_msg.position.x = pose[0]
        pose_msg.position.y = pose[1]
        pose_msg.position.z = pose[2]
        pose_msg.orientation.x = pose[3]
        pose_msg.orientation.y = pose[4]
        pose_msg.orientation.z = pose[5]
        pose_msg.orientation.w = pose[6]
        poses.append(pose_msg)

    return poses


class PlanningInterface:
    def __init__(self):
        # Retrieve the arm and gripper move group names from the parameter server
        name = rospy.get_name()
        move_group_name = rospy.get_param(name + '/move_group')
        self.default_vel_scaling = rospy.get_param(name + '/default_vel_scaling')
        self.default_acc_scaling = rospy.get_param(name + '/default_acc_scaling')

        self.move_group = moveit_commander.MoveGroupCommander(move_group_name, wait_for_servers=30.0)

        self.move_group.set_pose_reference_frame("world")
        constraints = moveit_msgs.msg.Constraints()
        constraints.joint_constraints = []
        self.joint_constraints = rospy.get_param(name + '/joint_constraints')
        print("param name: " + name + '/joint_constraints')
        print(self.joint_constraints)

        for joint_name, limits in self.joint_constraints.items():
            min = limits['min']
            max = limits['max']
            middle = (min + max) / 2
            constraint = moveit_msgs.msg.JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = middle
            constraint.tolerance_above = max - middle
            constraint.tolerance_below = middle - min
            constraint.weight = 10
            constraints.joint_constraints.append(constraint)

        # self.move_group.set_path_constraints(constraints)


        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.plan_srv = rospy.Service(name + '/simple_plan', SimplePlan, self.simple_plan_cb)

    def simple_plan_cb(self, req):
        print(req)

        mode = req.mode
        joints = req.joints
        target = req.target
        pose = req.pose
        center = req.center

        vel_scaling = req.vel_scaling
        acc_scaling = req.acc_scaling

        # set start state to current state
        self.move_group.set_start_state_to_current_state()

        # set velocity and acceleration scaling factors
        if vel_scaling == 0:
            vel_scaling = self.default_vel_scaling
        if acc_scaling == 0:
            acc_scaling = self.default_acc_scaling
        self.move_group.set_max_velocity_scaling_factor(vel_scaling)
        self.move_group.set_max_acceleration_scaling_factor(acc_scaling)


        # Transform the pose to the "world" frame
        if mode == SimplePlanRequest.POSE or mode == SimplePlanRequest.POSE_LINE:
            try:
                pose = self.tfBuffer.transform(pose, "world", timeout=rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr("Error transforming pose: " + str(e))
                res = SimplePlanResponse()
                res.success = False
                return res
            
        if mode == SimplePlanRequest.CIRCLE:
            try:
                center = self.tfBuffer.transform(center, "world", timeout=rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr("Error transforming center: " + str(e))
                res = SimplePlanResponse()
                res.success = False
                return res

        if mode == SimplePlanRequest.POSE:
            self.move_group.set_planning_pipeline_id("ompl")
            self.move_group.set_planner_id("RRTConnect")
            self.move_group.set_pose_target(pose)
        elif mode == SimplePlanRequest.POSE_LINE:
            self.move_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
            self.move_group.set_planner_id("LIN")
            self.move_group.set_pose_target(pose)
        elif mode == SimplePlanRequest.TARGET:
            self.move_group.set_planning_pipeline_id("chomp")
            self.move_group.set_planner_id("CHOMP")
            self.move_group.set_named_target(target)
        elif mode == SimplePlanRequest.JOINT:
            self.move_group.set_planning_pipeline_id("ompl")
            self.move_group.set_planner_id("RRTConnect")
            self.move_group.set_joint_value_target(joints)
        elif mode == SimplePlanRequest.CIRCLE:

            current_pose = self.move_group.get_current_pose().pose
            center_point = center.point
            target_pose = pose.pose
            
            waypoints = generate_circle(current_pose, center_point, target_pose, rotation_step_size=0.02)

            if waypoints is None:
                rospy.logerr("Unable to compute path")
                res = SimplePlanResponse()
                res.success = False
                return res

            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,  # List of waypoints
                0.01,        # Step size (max distance between waypoints)
                5.0,         # Jump threshold
                avoid_collisions=True,  # Avoid collisions (set to False if not needed)
                path_constraints=None  # Can add a constraint to the path if needed
            )

            if fraction != 1.0:
                rospy.logerr("Unable to compute path")
                res = SimplePlanResponse()
                res.success = False
                return res

            plan = self.move_group.retime_trajectory(
                self.move_group.get_current_state(),
                plan,
                velocity_scaling_factor = vel_scaling,
                acceleration_scaling_factor = acc_scaling,
                algorithm = "iterative_time_parameterization" 
            ) 		

            res = SimplePlanResponse()        
            res.joint_trajectory = plan.joint_trajectory
            res.success = True
            return res

        else:
            rospy.logerr("Unknown planning mode: " + str(mode))
            res = SimplePlanResponse()
            res.success = False
            return res
        
        success, robot_traj, time, error_code = self.move_group.plan()

        if not success:
            rospy.logerr("Planning failed with error code: " + str(error_code))
            res = SimplePlanResponse()
            res.success = False
            return res
        
        res = SimplePlanResponse()        
        res.joint_trajectory = robot_traj.joint_trajectory
        res.success = True
        
        return res
    