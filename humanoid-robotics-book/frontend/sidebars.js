// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'Book-Index',
    },
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'Physical_AI_and_Humanoid_Robotics_BOOK',
        'Physical_AI_and_Humanoid_Robotics_Course_Package',
        'PHYSICAL_AI_HUMANOID_ROBOTICS_COMPLETE_BOOK',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-chapter-ros-2-nervous-system',
        'module-1-chapter-python-agents-rclpy',
        'module-1-chapter-urdf-for-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo and Unity)',
      items: [
        'module-2-chapter-digital-twins-gazebo',
        'module-2-chapter-high-fidelity-unity',
        'module-2-chapter-sensor-simulation-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3-chapter-ai-perception-isaac-sim',
        'module-3-chapter-vslam-isaac-ros',
        'module-3-chapter-path-planning-nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-chapter-vla-foundations',
        'module-4-chapter-voice-commands-llms',
        'module-4-chapter-autonomous-humanoid-capstone',
      ],
    },
    {
      type: 'doc',
      id: 'capstone-assembling-the-full-stack',
    },
    {
        type: 'doc',
        id: 'weekly-breakdown',
    }
  ],
};

module.exports = sidebars;

