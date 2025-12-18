
---
sidebar_position: 102
---

# Chapter 2: Python Agents & rclpy Control

In the previous chapter, we learned about the core concepts of ROS 2 and how it acts as the nervous system for our robots. Now, let's get our hands dirty and see how we can use Python to interact with a ROS 2 system.

## What is rclpy?

`rclpy` is the official Python client library for ROS 2. It allows you to write Python scripts (which we'll call "agents") that can create nodes, publish and subscribe to topics, call services, and interact with actions.

With `rclpy`, you can write Python code that controls your robot, reads sensor data, and performs complex tasks. It's a powerful tool that makes it easy to get started with ROS 2 programming.

## A Simple Python Agent

Let's start with a simple example of a Python agent that creates a ROS 2 node and publishes a "Hello, World!" message to a topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldPublisher(Node):

    def __init__(self):
        super().__init__('hello_world_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_world', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, World!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    hello_world_publisher = HelloWorldPublisher()
    rclpy.spin(hello_world_publisher)
    hello_world_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Let's break down this code:

1.  **`import rclpy`**: This line imports the `rclpy` library.
2.  **`from rclpy.node import Node`**: This line imports the `Node` class, which is the base class for all ROS 2 nodes.
3.  **`from std_msgs.msg import String`**: This line imports the `String` message type, which is used to send simple text messages.
4.  **`class HelloWorldPublisher(Node):`**: This line defines a new class called `HelloWorldPublisher` that inherits from the `Node` class.
5.  **`super().__init__('hello_world_publisher')`**: This line calls the constructor of the `Node` class and gives our node a name: "hello_world_publisher".
6.  **`self.publisher_ = self.create_publisher(String, 'hello_world', 10)`**: This line creates a publisher that publishes messages of type `String` to the `hello_world` topic. The `10` is the queue size, which is the number of messages to buffer before they are sent.
7.  **`self.timer = self.create_timer(timer_period, self.timer_callback)`**: This line creates a timer that calls the `timer_callback` method every `timer_period` seconds.
8.  **`def timer_callback(self):`**: This method is called by the timer. It creates a new `String` message, sets its data to "Hello, World!", and publishes it to the `hello_world` topic.
9.  **`def main(args=None):`**: This is the main function of our script.
10. **`rclpy.init(args=args)`**: This line initializes the `rclpy` library.
11. **`hello_world_publisher = HelloWorldPublisher()`**: This line creates an instance of our `HelloWorldPublisher` class.
12. **`rclpy.spin(hello_world_publisher)`**: This line starts the ROS 2 event loop. It will block until the node is shut down.
13. **`hello_world_publisher.destroy_node()`**: This line destroys the node.
14. **`rclpy.shutdown()`**: This line shuts down the `rclpy` library.

### Running the Agent

To run this agent, you would save it as a Python file (e.g., `hello_world.py`) and then run it from the command line:

```bash
python3 hello_world.py
```

You would then see the following output:

```
[INFO] [hello_world_publisher]: Publishing: "Hello, World!"
[INFO] [hello_world_publisher]: Publishing: "Hello, World!"
...
```

You could also use the `ros2` command-line tool to inspect the topic:

```bash
ros2 topic echo /hello_world
```

This would show you the messages being published to the topic.

## Subscribing to a Topic

Now, let's create another Python agent that subscribes to the `hello_world` topic and prints the messages it receives.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldSubscriber(Node):

    def __init__(self):
        super().__init__('hello_world_subscriber')
        self.subscription = self.create_subscription(
            String,
            'hello_world',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    hello_world_subscriber = HelloWorldSubscriber()
    rclpy.spin(hello_world_subscriber)
    hello_world_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This code is very similar to the publisher code. The main difference is that it creates a subscription instead of a publisher. The `create_subscription` method takes the message type, the topic name, a callback function, and the queue size as arguments. The callback function is called whenever a new message is received on the topic.

## Services and Actions in rclpy

You can also use `rclpy` to create service clients and servers, and action clients and servers. The process is very similar to creating publishers and subscribers. You would use the `create_service` and `create_client` methods for services, and the `create_action_server` and `create_action_client` methods for actions.

## Exercises

1.  What is `rclpy`?
2.  Write a Python agent that subscribes to the `/scan` topic (of type `sensor_msgs/LaserScan`) and prints the minimum and maximum range values.
3.  Write a Python agent that provides a service called `/add_two_ints` that takes two integers as input and returns their sum.
4.  Write a Python agent that is an action client for the `/move_to_goal` action. It should send a goal to the action server and print the feedback and result.
5.  Modify the "Hello, World!" publisher to publish your name instead of "Hello, World!".
