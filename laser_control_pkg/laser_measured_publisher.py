import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
'''
This node gets the weed to kill position and moves the laser in its direction
if the laser is over the target the laser is switched on for 100 ms
after 100 ms laser is switched of 
'''
class LaserMeasuredPublisher(Node):
    def __init__(self):
        super().__init__('laser_measured_publisher')
        self.publisher_ = self.create_publisher(Float64, 'measured_laser', 10)
        self.subscription = self.create_subscription(
            Float64,
            'sensor_data',
            self.sensor_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.timer = self.create_timer(1.0, self.publish_measured_value)

    def publish_measured_value(self):
        msg = Float64()
        msg.data = 20.0  # Replace with your logic
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Measured Value: {msg.data}')

    def sensor_callback(self, msg):
        self.get_logger().info(f'Received Sensor Data: {msg.data}')
        # Process received data

def main(args=None):
    rclpy.init(args=args)
    node = LaserMeasuredPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
