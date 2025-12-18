import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleVelocitySubscriber(Node):

    def __init__(self):
        # 1. Initialize the Node with a unique name
        super().__init__('simple_velocity_subscriber')
        
        # 2. Create a Subscriber:
        #    - Topic Type: Twist (to receive velocity commands)
        #    - Topic Name: '/cmd_vel' (matching the publisher)
        #    - Queue Size: 10
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        self.get_logger().info('Velocity Subscriber Node Initialized. Listening to /cmd_vel.')

    def listener_callback(self, msg):
        # 3. Define the callback function that executes every time a message is received
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Log the received command. In a real robot, this is where you'd send
        # instructions to the motor drivers based on linear_x and angular_z.
        self.get_logger().info(f'Received command: Linear X={linear_x:.2f} m/s, Angular Z={angular_z:.2f} rad/s')

def main(args=None):
    # Standard boilerplate for ROS 2 Python
    rclpy.init(args=args)
    subscriber_node = SimpleVelocitySubscriber()
    
    try:
        # Keep the node running so it can continuously listen for messages
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
