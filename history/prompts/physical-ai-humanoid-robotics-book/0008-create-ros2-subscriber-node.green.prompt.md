--- 
id: 0008
title: Create ROS 2 Subscriber Node in Local Workspace
stage: green
date: 2025-12-07
surface: agent
model: gemini-1.5-flash-latest
feature: physical-ai-humanoid-robotics-book
branch: main
user: user
command: |-
  File Structure
  Save the code below in the file:

  ~/ros2_ws/src/my_robot_control/my_robot_control/simple_subscriber.py

  B. Python Code (simple_subscriber.py)
  This node will subscribe to the /cmd_vel topic and simply log the received velocity commands.

  Python

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
labels: ["ros2", "python", "subscriber", "colcon"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- ros2_ws_local/src/my_robot_control/my_robot_control/simple_subscriber.py
tests:
null
---

## Prompt

File Structure
Save the code below in the file:

~/ros2_ws/src/my_robot_control/my_robot_control/simple_subscriber.py

B. Python Code (simple_subscriber.py)
This node will subscribe to the /cmd_vel topic and simply log the received velocity commands.

Python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleVelocitySubscriber(Node):

    def __init__(self):
        # 1. Initialize the Node with a unique name
        super().__init__(\'simple_velocity_subscriber\')
        
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

## Response snapshot

Successfully created and wrote to new file: C:\Users\Hira Ali\OneDrive\Desktop\hack_yt\ros2_ws_local\src\my_robot_control\my_robot_control\simple_subscriber.py.

## Outcome

- ✅ Impact: A ROS 2 subscriber node was created in a local workspace, allowing for further development and building within the project's constraints.
- 🧪 Tests: No automated tests were run.
- 📁 Files: One Python file was created.
- 🔁 Next prompts: The user will likely want to build this new ROS 2 package.
- 🧠 Reflection: The user's request to create a file outside the workspace was handled by clarifying the limitation and proposing a viable alternative. After confirmation, the file was created successfully. This shows the importance of communication when encountering environmental constraints.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
