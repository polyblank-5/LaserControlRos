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
        self.timer_interval = 1.0
        self.timer = self.create_timer(self.timer_interval, self.publish_desired_value)
        self.x_weed:float = -1.0
        self.y_weed:float = -1.0

        self.x_laser:float = -1.0
        self.y_laser:float = -1.0

        self.x_speed:float = 0.0
        self.y_speed:float = 0.0

    def publish_desired_value(self):
        msg = Float32MultiArray()
        x_diff:float = self.x_weed-self.x_laser
        y_diff:float = self.y_weed-self.y_laser
        if x_diff > 0:
            x_next:float = x_diff - self.x_speed*self.timer_interval
        else:
            x_next = x_diff + self.x_speed*self.timer_interval
        if y_diff > 0:
            y_next:float = y_diff - self.y_speed*self.timer_interval
        else:
            y_next = y_diff + self.y_speed*self.timer_interval
        msg.data = [x_next,y_next]  # Replace with your logic
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Desired Value: {msg.data}')

    def control_callback(self, msg:Float32MultiArray):
        #self.get_logger().info(f'Received Control Input: {msg.data}')
        self.x_weed,self.y_weed  = LaserMeasuredPublisher.transform_position(msg.data[0],msg.data[1])



def main(args=None):
    rclpy.init(args=args)
    node = LaserPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
