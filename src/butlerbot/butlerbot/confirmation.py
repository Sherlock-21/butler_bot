import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
import signal

class Confirmation(Node):
    def __init__(self):
        super().__init__('confirmation')
        self.get_logger().info('Confirmation Node is up and running!')

        # Service for kitchen confirmation
        self.kitchen_service = self.create_service(SetBool, 'kitchen_confirmation_service', self.handle_kitchen_confirmation)

        # Service for table confirmation
        self.table_service = self.create_service(SetBool, 'table_confirmation_service', self.handle_table_confirmation)

        # Service for order cancellation
        self.order_cancellation = self.create_service(SetBool, 'cancel_order', self.cancel_order)

    def handle_kitchen_confirmation(self, request, response):
        """Handle the kitchen confirmation request."""
        print('\nReceived request for KITCHEN confirmation.')

        confirmation = self.get_confirmation("\nKITCHEN : Confirm the order (1 or 0): ",11)
        if confirmation == 1:
            response.success = True
            response.message = "Kitchen confirmation successful."
            print('Kitchen confirmation sent.')
        else:
            response.success = False
            response.message = "Kitchen DENIED the order."
            print('Kitchen confirmation failed.')

        return response
    
    def handle_table_confirmation(self, request, response):
        """Handle the table confirmation request."""
        print('\nReceived request for TABLE confirmation.')

        confirmation = self.get_confirmation("\nTABLE : Confirm the order (1 or 0): ",11)
        if confirmation == 1:
            response.success = True
            response.message = "Table confirmation successful."
            print('Table confirmation sent.')
        else:
            response.success = False
            response.message = "Table DENIED the order."
            print('Table confirmation failed.')

        return response
    
    def cancel_order(self, request, response):
        """Order Cancellation Request"""
        confirmation = self.get_confirmation("\nPRESS 1 To CANCEL the ORDER: ",6)
        if confirmation == 1:
            response.success = True
            response.message = "ORDER CANCELLED"
        else:
            response.success = False
            response.message = "ORDER PASSED"
        
        return response

    def get_confirmation(self, prompt,timeout):
        
        def timeout_handler(signum, frame):
            raise TimeoutError

        # Set the signal handler for SIGALRM
        signal.signal(signal.SIGALRM, timeout_handler)
        signal.alarm(timeout)  

        try:
            # Get user input
            confirmation = int(input(prompt))
        except TimeoutError:
            print()
            confirmation = 0
        except ValueError:
            print()
            confirmation = 0
        finally:

            signal.alarm(0)

        return confirmation

def main(args=None):
    rclpy.init(args=args)
    confirmation = Confirmation()

    rclpy.spin(confirmation)

    # Clean up
    confirmation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
