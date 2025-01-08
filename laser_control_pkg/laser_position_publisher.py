import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from laser_control_pkg.laser_measured_publisher import LaserMeasuredPublisher
'''
This node gets the positions from the weed list
It checks if one of the weed is in the laser area
if so its position is getting published 
checks laser status to see if weed is killed 

'''
class LaserPositionPublisher(Node):
    def __init__(self):
        super().__init__('laser_position_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'position_laser', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'weed_position_laser_area',
            self.control_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.timer = self.create_timer(1.0, self.publish_desired_value)
        self.x_weed:float = -1.0
        self.y_weed:float = -1.0

    def publish_desired_value(self):
        msg = Float32MultiArray()
        msg.data = 10.0  # Replace with your logic
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Desired Value: {msg.data}')

    def control_callback(self, msg:Float32MultiArray):
        self.get_logger().info(f'Received Control Input: {msg.data}')
        # Process received data

def main(args=None):
    rclpy.init(args=args)
    node = LaserPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
