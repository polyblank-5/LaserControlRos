import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger,Trigger_Request,Trigger_Response

class LaserControlService(Node):
    def __init__(self):
        super().__init__('laser_control_service')

        # Create services
        self.activate_service = self.create_service(
            Trigger,
            '/laser_control/activate_laser',
            self.activate_laser_callback
        )
        self.deactivate_service = self.create_service(
            Trigger,
            '/laser_control/deactivate_laser',
            self.deactivate_laser_callback
        )

        self.laser_state = False  # Internal state of the laser

        self.get_logger().info('Laser Control Services Ready')

    def activate_laser_callback(self, request:Trigger_Request, response:Trigger_Response):
        """Handle laser activation requests."""
        if not self.laser_state:
            self.laser_state = True
            response.success = True
            response.message = "Laser activated successfully."
        else:
            response.success = False
            response.message = "Laser already activated."
        self.get_logger().info(response.message)
        return response

    def deactivate_laser_callback(self, request:Trigger_Request, response:Trigger_Response):
        """Handle laser deactivation requests."""
        if self.laser_state:
            self.laser_state = False
            response.success = True
            response.message = "Laser deactivated successfully."
        else:
            response.success = False
            response.message = "Laser already deactivated."

        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)

    node = LaserControlService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Service node shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
