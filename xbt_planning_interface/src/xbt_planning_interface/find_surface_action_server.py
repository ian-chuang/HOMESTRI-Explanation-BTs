import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import WrenchStamped
from xbt_planning_interface.msg import FindSurfaceAction, FindSurfaceResult
from actionlib_msgs.msg import GoalStatus
import numpy as np
import threading

BASE_LINK = "base_link"
TIP_LINK = "tcp_link"
    
class FindSurfaceActionServer:
    def __init__(self):  
        # get name of node
        name = rospy.get_name()   

        self.tip_link = TIP_LINK 
        self.base_link = BASE_LINK

        # Create a TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize the wrench publisher
        self.wrench_publisher = rospy.Publisher(name + "/target_wrench", WrenchStamped, queue_size=1)

        # Initialize the current wrench subscriber
        self.wrench_subscriber = rospy.Subscriber(name + "/wrench", WrenchStamped, self.wrench_callback, queue_size=1)
        # current wrench 
        self.current_wrench = None
        # current wrench mutex
        self.current_wrench_mutex = threading.Lock()

        # Initialize the action server
        self.find_surface_action_server = actionlib.SimpleActionServer(
            name + '/find_surface',
            FindSurfaceAction,
            execute_cb=self.execute_cb,
            auto_start=False)
        self.find_surface_action_server.start()

    def wrench_callback(self, wrench_stamped):
        with self.current_wrench_mutex:
            self.current_wrench = wrench_stamped

    def execute_cb(self, goal):
        # retrieve values
        wrench_stamped = goal.wrench
        force_threshold = goal.force_threshold
        maximum_distance = goal.maximum_distance
        timeout = goal.timeout.to_sec()

        # set any torques to zero
        wrench_stamped.wrench.torque.x = 0
        wrench_stamped.wrench.torque.y = 0
        wrench_stamped.wrench.torque.z = 0

        # if timeout is 0 then set it to infinity
        if timeout == 0:
            timeout = float('inf')

        # get the starting pose
        start_pos, start_quat, ok = self.get_current_pose()
        if not ok:
            self.reset_target_wrench()
            find_surface_result = FindSurfaceResult()
            find_surface_result.error_code = FindSurfaceResult.TF_LOOKUP_ERROR
            self.find_surface_action_server.set_aborted(find_surface_result)
            return

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if self.find_surface_action_server.is_preempt_requested():
                self.reset_target_wrench()
                self.find_surface_action_server.set_preempted()
                return

            # publish wrench
            self.wrench_publisher.publish(wrench_stamped)

            # get current pose
            current_pos, current_quat, ok = self.get_current_pose()
            if not ok:
                self.reset_target_wrench()
                find_surface_result = FindSurfaceResult()
                find_surface_result.error_code = FindSurfaceResult.TF_LOOKUP_ERROR
                self.find_surface_action_server.set_aborted(find_surface_result)
                return

            # calculate distance error
            distance_error = np.linalg.norm(current_pos - start_pos)

            # check if max distance has been exceeded
            if distance_error > maximum_distance:
                self.reset_target_wrench()
                find_surface_result = FindSurfaceResult()
                find_surface_result.error_code = FindSurfaceResult.MAXIMUM_DISTANCE_EXCEEDED
                self.find_surface_action_server.set_aborted(find_surface_result)
                return

            # check if force threshold has been exceeded
            with self.current_wrench_mutex:
                current_wrench = self.current_wrench

            if current_wrench is not None:
                force = np.array([current_wrench.wrench.force.x, current_wrench.wrench.force.y, current_wrench.wrench.force.z])
                force_norm = np.linalg.norm(force)
                if force_norm > force_threshold:
                    self.reset_target_wrench()
                    find_surface_result = FindSurfaceResult()
                    find_surface_result.error_code = FindSurfaceResult.FORCE_THRESHOLD_EXCEEDED
                    self.find_surface_action_server.set_succeeded(find_surface_result)
                    return

        # timeout exceeded
        rospy.logerr("Timeout exceeded")
        self.reset_target_wrench()
        find_surface_result = FindSurfaceResult()
        find_surface_result.error_code = FindSurfaceResult.TIMEOUT_EXCEEDED
        self.find_surface_action_server.set_aborted(find_surface_result)

    def reset_target_wrench(self):
        # Create a WrenchStamped message
        wrench_stamped = WrenchStamped()
        wrench_stamped.header.stamp = rospy.Time.now()
        wrench_stamped.header.frame_id = self.tip_link
        wrench_stamped.wrench.force.x = 0.0
        wrench_stamped.wrench.force.y = 0.0
        wrench_stamped.wrench.force.z = 0.0
        wrench_stamped.wrench.torque.x = 0.0
        wrench_stamped.wrench.torque.y = 0.0
        wrench_stamped.wrench.torque.z = 0.0

        # Publish the WrenchStamped message
        self.wrench_publisher.publish(wrench_stamped)

    def get_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.base_link, self.tip_link, rospy.Time(0), timeout=rospy.Duration(1.0))
            # Extract position and quaternion from the transform
            position = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
            quaternion = np.array([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
            return position, quaternion, True
        except Exception as e:
            rospy.logerr(f"TF lookup failed: {e}")
            return None, None, False  # Handle lookup failure gracefully
