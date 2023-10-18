import rospy
import moveit_commander
from xbt_planning_interface.srv import SimplePlan, SimplePlanRequest, SimplePlanResponse
import moveit_msgs.msg
import math
import tf2_ros
import tf2_geometry_msgs

class PlanningInterface:
    def __init__(self):
        # Retrieve the arm and gripper move group names from the parameter server
        name = rospy.get_name()
        move_group_name = rospy.get_param(name + '/move_group')
        self.default_vel_scaling = rospy.get_param(name + '/default_vel_scaling')
        self.default_acc_scaling = rospy.get_param(name + '/default_acc_scaling')

        self.move_group = moveit_commander.MoveGroupCommander(move_group_name, wait_for_servers=30.0)

        self.move_group.set_planning_time(5)
        self.move_group.allow_replanning(True)
        self.move_group.allow_looking(True)

        constraints = moveit_msgs.msg.Constraints()
        constraints.joint_constraints = []

        joint_constraint = moveit_msgs.msg.JointConstraint()
        joint_constraint.joint_name = 'shoulder_pan_joint'
        joint_constraint.position = 0
        joint_constraint.tolerance_above = math.pi
        joint_constraint.tolerance_below = math.pi
        joint_constraint.weight = 10
        constraints.joint_constraints.append(joint_constraint)

        joint_constraint = moveit_msgs.msg.JointConstraint()
        joint_constraint.joint_name = 'elbow_joint'
        joint_constraint.position = math.pi/2
        joint_constraint.tolerance_above = math.pi/2
        joint_constraint.tolerance_below = math.pi/2
        joint_constraint.weight = 10
        constraints.joint_constraints.append(joint_constraint)

        joint_constraint = moveit_msgs.msg.JointConstraint()
        joint_constraint.joint_name = 'wrist_1_joint'
        joint_constraint.position = -math.pi
        joint_constraint.tolerance_above = math.pi
        joint_constraint.tolerance_below = math.pi/2
        joint_constraint.weight = 10
        constraints.joint_constraints.append(joint_constraint)

        joint_constraint = moveit_msgs.msg.JointConstraint()
        joint_constraint.joint_name = 'wrist_3_joint'
        joint_constraint.position = -math.pi
        joint_constraint.tolerance_above = 4
        joint_constraint.tolerance_below = math.pi
        joint_constraint.weight = 10
        constraints.joint_constraints.append(joint_constraint)

        self.move_group.set_path_constraints(constraints)


        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.plan_srv = rospy.Service(name + '/simple_plan', SimplePlan, self.simple_plan_cb)

    def simple_plan_cb(self, req):
        print(req)

        mode = req.mode
        joints = req.joints
        target = req.target
        pose = req.pose
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
            self.move_group.set_planning_pipeline_id("chomp")
            self.move_group.set_planner_id("CHOMP")
            self.move_group.set_joint_value_target(joints)
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
    