import React, { useState } from 'react';
import SyllabusCard from '../SyllabusCard';
import styles from './SyllabusGrid.module.css';

// Data is now updated with correct slugs from sidebars.js to ensure valid links
const syllabusData = [
    {
      "id": 1,
      "title": "Module 1: The Robotic Nervous System (ROS 2)",
      "focus": "Middleware for robot control. Covers ROS 2 Nodes, Topics, Services, rclpy, and URDF for humanoids.",
      "chapters": [
        { "title": "ROS 2 Nervous System", "slug": "module-1-chapter-ros-2-nervous-system" },
        { "title": "Python Agents with rclpy", "slug": "module-1-chapter-python-agents-rclpy" },
        { "title": "URDF for Humanoids", "slug": "module-1-chapter-urdf-for-humanoids" }
      ]
    },
    {
      "id": 2,
      "title": "Module 2: The Digital Twin (Gazebo & Unity)",
      "focus": "Physics simulation and environment building. Covers Gazebo physics, Unity rendering, and sensors like LiDAR, depth cameras, and IMUs.",
      "chapters": [
        { "title": "Digital Twins with Gazebo", "slug": "module-2-chapter-digital-twins-gazebo" },
        { "title": "High-Fidelity Rendering in Unity", "slug": "module-2-chapter-high-fidelity-unity" },
        { "title": "Advanced Sensor Simulation", "slug": "module-2-chapter-sensor-simulation-humanoids" }
      ]
    },
    {
      "id": 3,
      "title": "Module 3: The AI-Robot Brain (NVIDIA Isaac™)",
      "focus": "Advanced perception and training. Covers Isaac Sim, synthetic data generation, Isaac ROS, VSLAM, and Nav2.",
       "chapters": [
        { "title": "AI Perception with Isaac Sim", "slug": "module-3-chapter-ai-perception-isaac-sim" },
        { "title": "VSLAM with Isaac ROS", "slug": "module-3-chapter-vslam-isaac-ros" },
        { "title": "Path Planning with Nav2", "slug": "module-3-chapter-path-planning-nav2" }
      ]
    },
    {
      "id": 4,
      "title": "Module 4: Vision-Language-Action (VLA)",
      "focus": "LLMs + Robotics. Covers OpenAI Whisper for voice commands, LLM-based planning, and an autonomous humanoid capstone project.",
      "chapters": [
        { "title": "Foundations of VLA", "slug": "module-4-chapter-vla-foundations" },
        { "title": "Voice Commands with LLMs", "slug": "module-4-chapter-voice-commands-llms" },
        { "title": "Autonomous Humanoid Capstone", "slug": "module-4-chapter-autonomous-humanoid-capstone" }
      ]
    }
];

const SyllabusGrid = () => {
  // State to track the currently open module
  const [openModule, setOpenModule] = useState(null);

  // Function to toggle a module's open/closed state
  const toggleModule = (id) => {
    setOpenModule(openModule === id ? null : id);
  };

  return (
    <div id="syllabus-section" className={styles.syllabusGrid}>
      {syllabusData.map((module) => (
        <SyllabusCard
          key={module.id}
          id={module.id}
          title={module.title}
          focus={module.focus}
          description={module.description} // This might be undefined now, will be cleaned up
          chapters={module.chapters}
          toggleModule={() => toggleModule(module.id)}
          isOpen={openModule === module.id}
        />
      ))}
    </div>
  );
};

export default SyllabusGrid;
