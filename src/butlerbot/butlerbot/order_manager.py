import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32


class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')
        self.get_logger().info('Order Manager Node is up and running!')

        # Placeholder for orders list
        self.orders = []

        # Publisher for the list of order numbers
        self.order_publisher = self.create_publisher(Int32MultiArray, 'order_queue', 10)

      
        

    def add_order(self, order_number):
        """Function to add a new order."""
        order = f'Order {order_number}'
        self.orders.append(order)
        print(f'\t\t\tNew order added: {order}')

        # Publish the updated order list
        

    def remove_order(self, order_number):
        """Function to remove an order."""
        order_to_remove = f'Order {order_number}'
        if order_to_remove in self.orders:
            self.orders.remove(order_to_remove)
            print(f'\t\t\tOrder removed: {order_to_remove}')
            # Publish the updated order list
            
        else:
            self.get_logger().info(f'\t\tOrder {order_number} not found in queue.')

    def publish_order_queue(self):
        """Publish the current order queue."""
        comorder=[int(order.split(' ')[-1]) for order in self.orders]
        order_msg = Int32MultiArray(data=comorder)
        self.order_publisher.publish(order_msg)



def main(args=None):
    rclpy.init(args=args)
    order_manager = OrderManager()

    while rclpy.ok():
        rclpy.spin_once(order_manager, timeout_sec=0.1)
        # Get the order action from the user
        user_input = input("Enter 'a<number1> <number2> ...' to add or 'r<number1> <number2> ...' to remove orders: ")

        # Split the input into individual commands
        commands = user_input.split()

        for command in commands:
            # Determine the action (add or remove) and the order number
            if command.startswith('a'):
                order_number = command[1:]
                order_manager.add_order(order_number)
            elif command.startswith('r'):
                order_number = command[1:]
                order_manager.remove_order(order_number)
            else:
                print(f"Invalid command '{command}'. Please enter 'a<number>' to add or 'r<number>' to remove an order.")

        order_manager.publish_order_queue()
        order_manager.orders = []

        
        

    # Clean up
    order_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

