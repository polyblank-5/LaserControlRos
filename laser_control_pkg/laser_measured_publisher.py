import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray
from std_msgs.msg import Bool
from typing import List, Tuple
import math
import time 
from inverse_coordinate_transformation import Constants, InverseCoordinateTransformation
'''
This node gets the weed to kill position and moves the laser in its direction
if the laser is over the target the laser is switched on for 100 ms
after 100 ms laser is switched of 
'''
class LaserMeasuredPublisher(Node):
    def __init__(self):
        super().__init__('laser_measured_publisher')
        constants_inverse= Constants('src/laser_control_pkg/config/config.yaml')
        self.inverse_transform = InverseCoordinateTransformation(constants_inverse)
        self.publisher_weed_trans_pos = self.create_publisher(Float32MultiArray, 'weed_position_laser_area', 10)
        self.publisher_laser_activity = self.create_publisher(Bool, 'laser_activation', 10)
        self.subscription_weed_position = self.create_subscription(
            Float32MultiArray,
            'plant_laser_data_publisher',
            self.plant_measured_position_callback,
            10)
        self.subscription_weed_position  # Prevent unused variable warning
        self.subscription_laser_position = self.create_subscription(
            Float32MultiArray,
            "laser_position_publisher",
            self.laser_position_callback,
            10
        )
        self.subscription_laser_position
        self.laser_timer = None 
        self.timer_active: bool = False
        #self.timer = self.create_timer(1.0, self.publish_measured_value)
        self.x_weed_transformed:float = -1.0
        self.y_weed_transformed:float = -1.0

        self.id:int = -1

    def laser_working_publisher(self):
        self.timer_active = True
        msg = Bool()
        msg.data = True
        self.get_logger().info(f'Publishing Data to {self.publisher_laser_activity.topic_name}: Laser {msg.data}')
        self.publisher_laser_activity.publish(msg)
        if self.laser_timer == None:
            self.laser_timer = self.create_timer(0.5,self.laser_finished_weed_timer)
        else:
            self.laser_timer.reset()
        #time.sleep(2) # TODO this is propably the problem
        #msg.data = False
        #self.get_logger().info(f'Publishing Data to {self.publisher_laser_activity.topic_name}: Laser {msg.data}')
        #self.publisher_laser_activity.publish(msg)
    
    def laser_finished_weed_timer(self):
        msg = Bool()
        msg.data = False
        self.get_logger().info(f'Turning Laser off {self.publisher_laser_activity.topic_name}: Laser {msg.data}')
        self.publisher_laser_activity.publish(msg)
        self.laser_timer.cancel()
        self.timer_active = False

    
    def laser_position_callback(self, msg:Float32MultiArray):
        """
        Get position of laser
        If laser over weed start lasering
        transform position too robot coordiantes to perform the distance check
        """
        self.get_logger().info(f'Getting Data from {self.subscription_laser_position.topic_name}: Laser {msg.data}')
        x_laser, y_laser = LaserMeasuredPublisher.transform_position(msg.data[0],msg.data[1])
        if math.sqrt((self.x_weed_transformed - x_laser) ** 2 + (self.y_weed_transformed - y_laser) ** 2) <= 0.5 and not self.timer_active:
            self.laser_working_publisher()
    
    def plant_measured_position_callback(self, msg:Float32MultiArray):
        """
        Publish transformed plant position
        """
        self.x_weed_transformed, self.y_weed_transformed = LaserMeasuredPublisher.transform_position(float(msg.data[0]),float(msg.data[1]))
        data_pub = Float32MultiArray()
        data_pub.data = [self.x_weed_transformed, self.y_weed_transformed]
        self.publisher_weed_trans_pos.publish(data_pub)
        #self.get_logger().info(f'Received {self.publisher_weed_trans_pos.topic_name}: {data_pub.data}')
        #if self.id != msg.data[2]:
        #    self.id = msg.data[2]
        #    self.laser_working_publisher()
        # Process received data
        '''
        Get data from desired position
        move to desired position with defined speed
        publish new position

        when meassuered postion next to desired -> laser on for 100ms
            laser off 
        
        new position will be published after a weed is grilled


        #TODO do i have to publish actions to the arm controller
        
        
        '''
    def transformed_weed_position_publisher(self):
        pass
    
    @staticmethod
    def transform_position(x:float,y:float)-> List[float]:
        return [x,y]

    @staticmethod
    def retransform_position(x:float,y:float)-> List[float]:
        return [x,y]

def main(args=None):
    rclpy.init(args=args)
    node = LaserMeasuredPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
