import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray
from std_msgs.msg import Bool
import time 
'''
This node gets the weed to kill position and moves the laser in its direction
if the laser is over the target the laser is switched on for 100 ms
after 100 ms laser is switched of 
'''
class LaserMeasuredPublisher(Node):
    def __init__(self):
        print('running')
        super().__init__('laser_measured_publisher')
        self.publisher_desired_pos = self.create_publisher(Float64, 'measured_laser', 10)
        self.publisher_laser_activity = self.create_publisher(Bool, 'laser_activation', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'plant_laser_data_publisher',
            self.laser_measured_position_callback,
            10)
        self.subscription  # Prevent unused variable warning
        #self.timer = self.create_timer(1.0, self.publish_measured_value)

        self.id:int = -1

    def laser_working_publisher(self):
        msg = Bool()
        msg.data = True
        self.get_logger().info(f'Publishing Data to {self.publisher_laser_activity.topic_name}: Laser {msg.data}')
        self.publisher_laser_activity.publish(msg)
        time.sleep(1/25)
        msg.data = False
        self.get_logger().info(f'Publishing Data to {self.publisher_laser_activity.topic_name}: Laser {msg.data}')
        self.publisher_laser_activity.publish(msg)


    def publish_measured_value(self): # timer callback
        msg = Float64()
        msg.data = 20.0  # Replace with your logic
        self.publisher_desired_pos.publish(msg)
        self.get_logger().info(f'Publishing Measured Value: {msg.data}')

    def laser_measured_position_callback(self, msg):
        self.get_logger().info(f'Received {self.subscription.topic_name}: {msg.data}')
        if self.id != msg.data[2]:
            self.id = msg.data[2]
            self.laser_working_publisher
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

def main(args=None):
    rclpy.init(args=args)
    node = LaserMeasuredPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
