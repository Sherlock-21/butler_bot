import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')    #Node Initialization
        self.get_logger().info('Order Manager Node is up and running!')

        # Placeholder for orders list
        self.orders = []          #order_queue varialbe

        # Publisher for the list of order numbers
        self.order_publisher = self.create_publisher(Int32MultiArray, 'order_queue', 10) # pub creation

      
        

    def add_order(self, order_number):      #function for adding order
        """Function to add a new order."""
        order = f'Order {order_number}'
        self.orders.append(order)
        print(f'\t\t\tNew order added: {order}')

       

    def publish_order_queue(self):     # publish order_queue
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
        user_input = input("Enter 'a<number1> <number2> ...' to add orders: ")

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
        order_manager.orders = []   # resetting order_queue

        
        

    # Clean up
    order_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

