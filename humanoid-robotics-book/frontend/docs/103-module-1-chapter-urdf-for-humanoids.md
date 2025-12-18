
---
sidebar_position: 103
---

# Chapter 3: URDF for Humanoid Robots

In the previous chapters, we learned about the basics of ROS 2 and how to control a ROS 2 system using Python. Now, it's time to give our robots a body. In this chapter, we'll learn how to use the Unified Robot Description Format (URDF) to model humanoid robots.

## What is URDF?

URDF is an XML format for representing a robot model. It allows you to define the robot's links, joints, sensors, and other properties in a structured way. URDF is the most common way to describe a robot's physical structure in ROS.

A URDF file is composed of a set of XML tags that describe the robot's geometry, kinematics, and dynamics. The main tags are `<robot>`, `<link>`, and `<joint>`.

### The `<robot>` Tag

The `<robot>` tag is the root element of a URDF file. It has a single attribute, `name`, which is the name of the robot.

```xml
<robot name="my_humanoid">
  ...
</robot>
```

### The `<link>` Tag

The `<link>` tag describes a rigid part of the robot. It has a `name` attribute and can contain three optional tags: `<inertial>`, `<visual>`, and `<collision>`.

-   `<inertial>`: Describes the link's inertial properties (mass, center of mass, and inertia tensor).
-   `<visual>`: Describes the link's appearance (geometry, material, and color).
-   `<collision>`: Describes the link's collision geometry, which is used for collision detection.

Here's an example of a simple link:

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.2 0.3 0.5" />
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.3 0.5" />
    </geometry>
  </collision>
</link>
```

### The `<joint>` Tag

The `<joint>` tag describes the connection between two links. It has a `name` attribute and a `type` attribute, which can be one of the following:

-   `revolute`: A hinge joint that rotates along an axis.
-   `continuous`: A continuous joint that rotates along an axis.
-   `prismatic`: A sliding joint that moves along an axis.
-   `fixed`: A joint that does not allow any motion.
-   `floating`: A joint that allows motion in all 6 degrees of freedom.
-   `planar`: A joint that allows motion in a plane.

The `<joint>` tag also has two required child tags: `<parent>` and `<child>`, which specify the names of the two links that are connected by the joint.

Here's an example of a simple joint:

```xml
<joint name="torso_to_right_arm" type="revolute">
  <parent link="torso" />
  <child link="right_upper_arm" />
  <axis xyz="0 1 0" />
  <limit effort="1000.0" lower="-2.0" upper="2.0" velocity="0.5" />
</joint>
```

## Modeling a Humanoid Robot

Now, let's see how we can use these tags to model a simple humanoid robot. Our robot will have a torso, a head, two arms, and two legs.

Here's an ASCII diagram of our robot:

```
      +-------+
      |  Head |
      +-------+
          |
      +-------+
      | Torso |
      +-------+
      |   |   |
+-----+   |   +-----+
| L Arm | | | R Arm |
+-----+   |   +-----+
          |
      +---+---+
      | L Leg | R Leg |
      +---+---+
```

We can represent this robot in URDF as follows:

```xml
<robot name="my_humanoid">

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.3 0.5" />
      </geometry>
    </visual>
  </link>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </visual>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0 0 0.3" />
  </joint>

  <!-- Arms -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
    </visual>
  </link>

  <joint name="torso_to_right_upper_arm" type="revolute">
    <parent link="torso" />
    <child link="right_upper_arm" />
    <origin xyz="0 -0.2 0.2" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" />
  </joint>

  <!-- ... and so on for the other links and joints -->

</robot>
```

This is a very simple example, but it shows how you can use URDF to create a complete model of a humanoid robot. You can then use this model in ROS 2 for visualization, simulation, and control.

## Visualizing a URDF Model

You can use a tool like RViz to visualize a URDF model. RViz is a 3D visualization tool for ROS that can display robot models, sensor data, and other information.

To visualize your URDF model in RViz, you would need to do the following:

1.  Launch the `robot_state_publisher` node, which reads the URDF file and publishes the robot's state to the `/robot_description` topic.
2.  Launch RViz and add a "RobotModel" display.
3.  Set the "Robot Description" topic to `/robot_description`.

You would then see your robot model in the RViz window.

## Exercises

1.  What is URDF?
2.  What are the three main tags in a URDF file?
3.  Write the URDF code for the left arm of our simple humanoid robot. It should have an upper arm and a lower arm, connected by a revolute joint.
4.  How can you visualize a URDF model?
5.  Find a URDF model of a real humanoid robot online (e.g., on GitHub) and try to understand its structure.
