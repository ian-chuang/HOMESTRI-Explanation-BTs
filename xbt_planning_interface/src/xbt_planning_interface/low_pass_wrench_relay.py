import rospy
from geometry_msgs.msg import WrenchStamped

class LowPassWrenchRelay:
    def __init__(self):
        # Get input and output topic names from the private parameter server
        self.input_topic = rospy.get_param('~input_topic')
        self.output_topic = rospy.get_param('~output_topic')

        # Define filter parameters (adjust these as needed)
        # the higher the value the more smoothing
        self.alpha = rospy.get_param('~alpha', 0.1)
        
        # Initialize time variables
        self.last_time = None

        # Create a subscriber to the input wrench topic
        self.input_sub = rospy.Subscriber(self.input_topic, WrenchStamped, self.input_callback)

        # Create a publisher for the output wrench topic
        self.output_pub = rospy.Publisher(self.output_topic, WrenchStamped, queue_size=10)

        # Initialize the filtered_wrench with zero values
        self.filtered_wrench = WrenchStamped()
        self.filtered_wrench.wrench.force.x = 0.0
        self.filtered_wrench.wrench.force.y = 0.0
        self.filtered_wrench.wrench.force.z = 0.0
        self.filtered_wrench.wrench.torque.x = 0.0
        self.filtered_wrench.wrench.torque.y = 0.0
        self.filtered_wrench.wrench.torque.z = 0.0

    def input_callback(self, msg):
        current_time = rospy.Time.now()

        if self.last_time is not None:
            time_interval = (current_time - self.last_time).to_sec()

            # Calculate the exponential smoothing factor based on time
            alpha_time = 1 - pow(0.5, time_interval / self.alpha)

            # Update the filtered_wrench using the time-based exponential low-pass filter
            self.filtered_wrench.wrench.force.x = (1 - alpha_time) * self.filtered_wrench.wrench.force.x + alpha_time * msg.wrench.force.x
            self.filtered_wrench.wrench.force.y = (1 - alpha_time) * self.filtered_wrench.wrench.force.y + alpha_time * msg.wrench.force.y
            self.filtered_wrench.wrench.force.z = (1 - alpha_time) * self.filtered_wrench.wrench.force.z + alpha_time * msg.wrench.force.z
            self.filtered_wrench.wrench.torque.x = (1 - alpha_time) * self.filtered_wrench.wrench.torque.x + alpha_time * msg.wrench.torque.x
            self.filtered_wrench.wrench.torque.y = (1 - alpha_time) * self.filtered_wrench.wrench.torque.y + alpha_time * msg.wrench.torque.y
            self.filtered_wrench.wrench.torque.z = (1 - alpha_time) * self.filtered_wrench.wrench.torque.z + alpha_time * msg.wrench.torque.z

        # Publish the filtered wrench
        self.output_pub.publish(self.filtered_wrench)

        # Update the last_time
        self.last_time = current_time
