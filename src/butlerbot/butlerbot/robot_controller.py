import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        print('Robot Controller Node is up and running!')
        self.callback_group_1=MutuallyExclusiveCallbackGroup()
        self.callback_group_2=MutuallyExclusiveCallbackGroup()
        self.callback_group_3=MutuallyExclusiveCallbackGroup()
        self.callback_group_4=MutuallyExclusiveCallbackGroup()

        # Subscribe to the order queue topic
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'order_queue',
            self.order_callback,
            10,
            callback_group=self.callback_group_1
        )

        self.kitchen_service_client = self.create_client(SetBool, 'kitchen_confirmation_service',callback_group=self.callback_group_2)
        
        self.table_service_client = self.create_client(SetBool, 'table_confirmation_service',callback_group=self.callback_group_3)

        self.order_cancel = self.create_client(SetBool, 'cancel_order',callback_group=self.callback_group_4)

        while not self.kitchen_service_client.wait_for_service(timeout_sec=1.0):
            print('Service not available, waiting...')
        
        while not self.table_service_client.wait_for_service(timeout_sec=1.0):
            print('Service not available, waiting...')


        # Placeholders for the current order and confirmation flags
        self.current_order = None
        self.kitchen_confirmed = 0
        self.table_confirmed = 0
        self.order_completed = False
        self.ret_kitch = 0

    def order_callback(self, msg1):
        """Callback function for receiving order numbers."""
        if not msg1.data:
            print('Order queue is empty. No orders to process.')
            return

        # Process each order in the queue
        number_of_order = len(msg1.data)
        if number_of_order==1:
            for order_number in msg1.data:
                print(f'\n Received order number: {order_number}')
                self.process_order([msg1.data[0]])
        else:
            print(f'\n Received group order : {msg1.data}')
            self.process_order(list(msg1.data))

    def process_order(self, order_numbers):
        """Simulate robot movement for a given order."""
        
        self.kitchen_confirmed = 0
        self.table_confirmed = 0

        # Simulate movement to kitchen

        
     
        self.simulate_movement('\n\tMoving to Kitchen... -->')
        if not self.order_cancellation() or not self.request_kitchen_confirmation() :
            print()          
        
        
        else:
                
        

        

            for order_number in order_numbers:
                
                self.process_single_order(order_number)
            
            if self.ret_kitch==1:
                self.simulate_movement('\n\tMoving back to Kitchen... -->')
                    


        
        self.simulate_movement('\n\tReturning to Home State... -->')
        
            
             
        
        
    def process_single_order(self, order_number):
        """Simulate robot movement for a order."""
        
        
        self.simulate_movement(f'\n\tMoving to Table {order_number}... -->')
        
        if not self.order_cancellation() or not self.request_table_confirmation(order_number):
            self.ret_kitch = 1
            return

        
        print(f'\n Order {order_number} processed SUCCESSFULLY!')
                
        return 
            



    def simulate_movement(self, message):
        """Simulate the movement by typing out the message character by character."""
        for char in message:
            print(char, end='', flush=True)
            time.sleep(0.1)
        print() 



    def order_cancellation(self):
        """Cancellation time period"""

        # Create a request object
        request = SetBool.Request()
        request.data = True  # Send True to request confirmation

        # Call the service asynchronously
        future = self.order_cancel.call_async(request)

        # Wait for the service response with a timeout
        timeout_sec = 5.0
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)


        if future.done():
            try:
                response = future.result()
                if response.success:
                    print('\n \t\t\t ORDER CANCELLED')
                    return False
                    
                else:
                    
                    return True
                    
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
                
        else:
            return True

        
        


    def request_kitchen_confirmation(self):
        """Request confirmation from the kitchen via a service call."""
        print(' \nRequesting confirmation from Kitchen...')

        # Create a request object
        request = SetBool.Request()
        request.data = True  # Send True to request confirmation

        # Call the service asynchronously
        future = self.kitchen_service_client.call_async(request)

        # Wait for the service response with a timeout
        timeout_sec = 10.0
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)


        if future.done():
            try:
                response = future.result()
                if response.success:
                    print(' Received confirmation from Kitchen.')
                    return True
                    
                else:
                    print(f'\n Kitchen confirmation failed: {response.message}')
                    return False
                    
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
                
        else:
            print('\n\t\t No one attended the robot - Kitchen confirmation timed out!')
            return False

    def request_table_confirmation(self,order_number):
        """Request confirmation from the table via a service call."""
        print(f'\n Requesting confirmation from Table {order_number}')

        # Create a request object
        req = SetBool.Request()
        req.data = True  # Send True to request confirmation

        # Call the service asynchronously
        future = self.table_service_client.call_async(req)

        # Wait for the service response with a timeout
        timeout_sec = 10.0
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.done():
            try:
                response = future.result()
                if response.success:
                    print(f' Received confirmation from Table {order_number}.')
                    return True
                    
                else:
                    print(f' Table {order_number} confirmation failed: {response.message}')
                    return False
                    
            except Exception as e:
                self.get_logger().error(f'\n Service call failed: {e}')
                
        else:
            print('\n\t\t No one attended the robot - Table confirmation timed out!')
            return False
            

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    executor = MultiThreadedExecutor(5)
    executor.add_node(robot_controller)

    try:
        while 1 :
            executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
