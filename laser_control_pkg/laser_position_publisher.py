import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from laser_measured_publisher import LaserMeasuredPublisher
import math
'''
This node gets the positions from the weed list
It checks if one of the weed is in the laser area
if so its position is getting published 
checks laser status to see if weed is killed 

'''
class LaserPositionPublisher(Node):
    def __init__(self):
        super().__init__('laser_position_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'laser_position_publisher', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'weed_position_laser_area',
            self.control_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.timer_interval = 0.1
        self.timer = self.create_timer(self.timer_interval, self.publish_desired_value)
        self.x_weed:float = 202.0
        self.y_weed:float = 202.0

        self.x_laser:float = 100.0
        self.y_laser:float = 100.0

        self.x_speed:float = 50.0
        self.y_speed:float = 50.0

    def publish_desired_value(self):
        self.get_logger().info(f'Weed position: {self.x_weed,self.y_weed}')
        msg = Float32MultiArray()
        x_diff:float = self.x_weed-self.x_laser
        y_diff:float = self.y_weed-self.y_laser
        self.get_logger().info(f'position differences: {x_diff,y_diff}')
        if x_diff > 0: # TODO ersetzen durch sign funktion
            x_next:float = self.control_saturation(x_diff,- self.x_speed,self.x_weed,self.x_laser) 
        else:
            x_next = self.control_saturation(x_diff, self.x_speed,self.x_weed,self.x_laser)
        if y_diff > 0:
            y_next:float = self.control_saturation(y_diff,- self.y_speed,self.y_weed,self.y_laser)
        else:
            y_next = self.control_saturation(y_diff, self.y_speed,self.y_weed,self.y_laser)
        msg.data = [x_next,y_next]  # Replace with your logic
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Desired Value: {msg.data}')

    def control_saturation(self,difference:float,speed:float, weed_position:float,laser_position:float) -> float:
        result: float = 0.0
        if abs(difference)< abs(speed*self.timer_interval):
            result = weed_position
        else:
            result = laser_position + math.copysign(1,speed) * difference + speed*self.timer_interval
        return result

    def control_callback(self, msg:Float32MultiArray):
        #self.get_logger().info(f'Received Control Input: {msg.data}')
        self.x_weed,self.y_weed  = LaserMeasuredPublisher.transform_position(msg.data[0],msg.data[1])



def main(args=None):
    rclpy.init(args=args)
    node = LaserPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
