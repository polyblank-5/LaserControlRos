import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
'''
This node gets the positions from the weed list
It checks if one of the weed is in the laser area
if so its position is getting published 
checks laser status to see if weed is killed 

'''
class LaserDesiredPublisher(Node):
    def __init__(self):
        super().__init__('laser_desired_publisher')
        self.publisher_ = self.create_publisher(Float64, 'desired_laser', 10)
        self.subscription = self.create_subscription(
            Float64,
            'control_input',
            self.control_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.timer = self.create_timer(1.0, self.publish_desired_value)

    def publish_desired_value(self):
        msg = Float64()
        msg.data = 10.0  # Replace with your logic
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Desired Value: {msg.data}')

    def control_callback(self, msg):
        self.get_logger().info(f'Received Control Input: {msg.data}')
        # Process received data

def main(args=None):
    rclpy.init(args=args)
    node = LaserDesiredPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
