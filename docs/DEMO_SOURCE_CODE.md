# Demo Code & Setup Files

This file contains the full source code and configuration for all parts of the ROS 2 demo.

---

### **File 1: `ros2_ws/src/humanoid_control/package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_control</name>
  <version>0.0.1</version>
  <description>ROS 2 package for Physical AI humanoid demo</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

### **File 2: `ros2_ws/src/humanoid_control/setup.py`**

```python
from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package for Physical AI humanoid demo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_input_node = humanoid_control.voice_input_node:main',
            'llm_action_node = humanoid_control.llm_action_node:main',
        ],
    },
)
```

---

### **File 3: `ros2_ws/src/humanoid_control/launch/demo.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the core ROS 2 nodes for the voice-controlled humanoid demo.
    Note: The Isaac Sim controller node is launched from within the Isaac Sim application, not here.
    """
    return LaunchDescription([
        Node(
            package='humanoid_control',
            executable='voice_input_node',
            name='voice_input_node',
            output='screen'
        ),
        Node(
            package='humanoid_control',
            executable='llm_action_node',
            name='llm_action_node',
            output='screen'
        ),
    ])
```

---

### **File 4: `ros2_ws/src/humanoid_control/humanoid_control/voice_input_node.py`**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import os

# Set your OpenAI API key as an environment variable
# export OPENAI_API_KEY='your_key_here'

class VoiceInputNode(Node):
    """
    Captures audio from the microphone, transcribes it using OpenAI's Whisper API,
    and publishes the result to a ROS 2 topic.
    """
    def __init__(self):
        super().__init__('voice_input_node')
        self.publisher_ = self.create_publisher(String, '/voice_command', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        self.get_logger().info("Voice Input Node is ready. Calibrating microphone...")
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
        self.get_logger().info("Microphone calibrated. Listening for command...")
        self.listen_for_command()

    def listen_for_command(self):
        try:
            with self.microphone as source:
                audio = self.recognizer.listen(source)
            self.get_logger().info("Processing audio...")

            # Recognize speech using OpenAI Whisper API
            text = self.recognizer.recognize_whisper_api(audio)
            self.get_logger().info(f"Whisper API transcribed: '{text}'")

            # Publish the transcribed text
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            self.get_logger().info("Published voice command.")

        except sr.RequestError as e:
            self.get_logger().error(f"Could not request results from Whisper API; {e}")
        except sr.UnknownValueError:
            self.get_logger().warn("Whisper API could not understand audio")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")
        finally:
            # Listen again for the next command
            self.get_logger().info("Listening for next command...")
            self.listen_for_command()


def main(args=None):
    rclpy.init(args=args)
    voice_input_node = VoiceInputNode()
    rclpy.spin(voice_input_node)
    voice_input_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

---

### **File 5: `ros2_ws/src/humanoid_control/humanoid_control/llm_action_node.py`**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json
import os

# Make sure your OpenAI API key is set as an environment variable
# export OPENAI_API_KEY='your_key_here'

class LLMActionNode(Node):
    """
    Subscribes to transcribed voice commands, sends them to GPT-4o for interpretation,
    and publishes a structured action plan.
    """
    def __init__(self):
        super().__init__('llm_action_node')
        self.subscription = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10)
        self.action_publisher_ = self.create_publisher(String, '/action_plan', 10)
        self.client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.get_logger().info("LLM Action Node is ready.")

    def command_callback(self, msg):
        command_text = msg.data
        self.get_logger().info(f"Received voice command: '{command_text}'")
        
        try:
            system_prompt = """
            You are a robotics AI assistant. Your job is to translate a natural language command
            into a structured, step-by-step JSON plan that a robot can execute.
            The available actions are: NAVIGATE, PICKUP, PLACE, and SAY.
            - NAVIGATE: Requires a 'target' (e.g., "table", "kitchen_counter").
            - PICKUP: Requires a 'target' (e.g., "red_cup", "blue_box").
            - PLACE: Requires a 'target' location (e.g., "my_location", "table").
            - SAY: Requires a 'text' string for the robot to speak.

            The scene contains: 'red_cup', 'blue_box', 'table', 'kitchen_counter'.
            The user's location is 'user_location'.

            Based on the user's command, generate a JSON array of action objects.
            For example, for the command "bring me the red cup", the plan should be:
            [
                {"action": "NAVIGATE", "target": "table"},
                {"action": "PICKUP", "target": "red_cup"},
                {"action": "NAVIGATE", "target": "user_location"},
                {"action": "SAY", "text": "Here is your cup."}
            ]
            """
            
            response = self.client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": command_text}
                ],
                response_format={"type": "json_object"}
            )
            
            action_plan_str = response.choices[0].message.content
            self.get_logger().info(f"Generated action plan: {action_plan_str}")

            # Validate that the output is valid JSON
            json.loads(action_plan_str)

            action_msg = String()
            action_msg.data = action_plan_str
            self.action_publisher_.publish(action_msg)
            self.get_logger().info("Published action plan.")

        except openai.APIError as e:
            self.get_logger().error(f"OpenAI API error: {e}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode JSON from LLM response: {action_plan_str}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")


def main(args=None):
    rclpy.init(args=args)
    llm_action_node = LLMActionNode()
    rclpy.spin(llm_action_node)
    llm_action_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### **File 6: `isaac_sim_controller.py` (To be run inside Isaac Sim)**

*This script should be saved outside the ROS 2 workspace, in a location accessible to Isaac Sim's script editor.*

```python
import asyncio
import json
import carb

from omni.isaac.kit import SimulationApp

# The configuration for the Isaac Sim application
CONFIG = {
    "width": 1280,
    "height": 720,
    "headless": False,
    "renderer": "RayTracedLighting",
}

# Start the simulation
kit = SimulationApp(CONFIG)

from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import open_stage, add_reference_to_stage
from omni.isaac.core_nodes.scripts.core_nodes import CoreNodes
import omni.isaac.ros2_bridge.scripts.ros2_bridge as ros2_bridge

# --- ROS 2 Specific Imports ---
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread

# Global dictionary to store object references
SCENE_OBJECTS = {}

class IsaacControllerNode(Node):
    """
    The ROS 2 node that lives inside Isaac Sim. It subscribes to the action plan
    and controls the robot and objects in the simulation.
    """
    def __init__(self, world):
        super().__init__('isaac_controller_node')
        self.subscription = self.create_subscription(String, '/action_plan', self.plan_callback, 10)
        self._world = world
        self.get_logger().info("Isaac Controller Node is ready and listening for action plans.")

    def plan_callback(self, msg):
        self.get_logger().info(f"Received action plan: {msg.data}")
        try:
            # The plan is inside the 'plan' key if the JSON object is structured that way
            plan_data = json.loads(msg.data)
            plan = plan_data.get("plan", []) # Default to empty list if key not found
            if not plan:
                 self.get_logger().warn("Plan is empty or 'plan' key not found in JSON. Checking root.")
                 plan = plan_data # Assume the root is the list of actions
            
            # Run the plan asynchronously in the simulation context
            asyncio.ensure_future(self.execute_plan(plan))
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode action plan JSON.")
        except Exception as e:
            self.get_logger().error(f"Error processing plan: {e}")

    async def execute_plan(self, plan):
        self.get_logger().info(f"Executing plan with {len(plan)} steps.")
        humanoid_robot = self._world.scene.get_object("humanoid")
        if not humanoid_robot:
            self.get_logger().error("Humanoid robot not found in the scene!")
            return

        for step in plan:
            action = step.get("action")
            target = step.get("target")
            self.get_logger().info(f"Executing: {action} -> {target}")

            # This is a placeholder for actual robot control logic.
            # In a real scenario, this would call navigation and manipulation APIs.
            if action == "NAVIGATE":
                target_obj = SCENE_OBJECTS.get(target)
                if target_obj:
                    # Dummy navigation: just move the robot near the object
                    pos, _ = target_obj.get_world_pose()
                    humanoid_robot.set_world_pose(position=pos - [1.5, 0, 0])
                    self.get_logger().info(f"Navigating robot to {target}.")
                else:
                    self.get_logger().warn(f"Navigation target '{target}' not found.")
            
            elif action == "PICKUP":
                target_obj = SCENE_OBJECTS.get(target)
                if target_obj:
                    # Dummy pickup: parent the object to the robot's "hand"
                    # In a real scenario, you'd use a manipulator controller.
                    target_obj.set_parent(humanoid_robot.prim_path)
                    target_obj.set_local_pose(translation=[0.5, 0.5, 1.0]) # Position in hand
                    self.get_logger().info(f"Picking up {target}.")
                else:
                    self.get_logger().warn(f"Pickup target '{target}' not found.")
            
            elif action == "SAY":
                self.get_logger().info(f"ROBOT SAYS: '{step.get('text')}'")

            await self._world.next_simulation_step_async()
            await asyncio.sleep(2.0) # Pause between steps to visualize

        self.get_logger().info("Plan execution complete.")


def setup_scene(world):
    """
    Sets up the simulation environment, robot, and objects.
    """
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder.")
        return

    # Add a ground plane
    world.scene.add_default_ground_plane()

    # Add a humanoid robot
    # IMPORTANT: Replace with the actual path to your humanoid's USD file
    # This example uses a standard asset, but a humanoid like Unitree H1 would be better.
    humanoid_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
    add_reference_to_stage(usd_path=humanoid_asset_path, prim_path="/World/humanoid")
    
    # Add a table
    SCENE_OBJECTS["table"] = world.scene.add(
        VisualCuboid(
            prim_path="/World/table",
            name="table",
            position=[2.0, 0, 0.4],
            scale=[1.5, 2.0, 0.8],
            color=[0.5, 0.4, 0.3]
        )
    )

    # Add the red cup
    SCENE_OBJECTS["red_cup"] = world.scene.add(
        VisualCuboid(
            prim_path="/World/red_cup",
            name="red_cup",
            position=[2.0, -0.2, 0.85], # On the table
            scale=[0.1, 0.1, 0.2],
            color=[1.0, 0, 0]
        )
    )

    # Add the blue box
    SCENE_OBJECTS["blue_box"] = world.scene.add(
        VisualCuboid(
            prim_path="/World/blue_box",
            name="blue_box",
            position=[2.0, 0.2, 0.85], # On the table
            scale=[0.1, 0.1, 0.1],
            color=[0, 0, 1.0]
        )
    )

    # Define a user location
    SCENE_OBJECTS["user_location"] = world.scene.add(
        VisualCuboid(
            prim_path="/World/user_location",
            name="user_location",
            position=[-1.0, 0, 0],
            visible=False
        )
    )
    
    kit.update()


async def run_simulation():
    """The main asynchronous loop for the simulation."""
    world = World()
    setup_scene(world)
    
    # Start the ROS 2 bridge
    ros_bridge = ros2_bridge.Ros2Bridge()

    # Start the ROS 2 node in a separate thread
    rclpy.init()
    isaac_controller = IsaacControllerNode(world)
    ros_thread = Thread(target=rclpy.spin, args=(isaac_controller,), daemon=True)
    ros_thread.start()

    # Required for the world to be initialized
    await world.reset_async()

    # Main simulation loop
    while kit.is_running():
        # This is the main simulation step.
        await world.next_simulation_step_async()

    # Cleanup
    isaac_controller.destroy_node()
    rclpy.shutdown()
    kit.close()


if __name__ == "__main__":
    try:
        asyncio.run(run_simulation())
    except Exception as e:
        carb.log_error(f"Main simulation loop error: {e}")
        kit.close()
```
