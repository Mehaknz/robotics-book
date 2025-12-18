import React from 'react';
import styles from './style.module.css';

const HumanoidRoboticsComponent: React.FC = () => {
  const part7Title = 'Chapter 7: Advanced Navigation with Nav2';
  const part7Content = 'Configure the Nav2 stack for dynamic environments, including costmap plugins, behavior trees, and waypoint following for humanoid navigation.';

  return (
    <div className={styles.container}>
      {/* Div 1: Introduction Section */}
      <div className={`${styles.section} ${styles.introSection}`}>
        <h1 className={styles.mainTitle}>Physical AI & Humanoid Robotics</h1>
        <p className={styles.introText}>
          Welcome to the cutting edge of artificial intelligence. This book is your comprehensive guide to understanding, building, and deploying intelligent humanoid robots that can perceive, interact, and operate in the physical world. Bridge the gap between digital code and embodied action.
        </p>
      </div>

      {/* Div 2: Part 7 Content */}
      <div className={styles.section}>
        <h2 className={styles.sectionTitle}>{part7Title}</h2>
        <p className={styles.sectionContent}>{part7Content}</p>
      </div>

      {/* Div 3: Part 7 Content (Again) */}
      <div className={styles.section}>
        <h2 className={styles.sectionTitle}>{part7Title}</h2>
        <p className={styles.sectionContent}>{part7Content}</p>
      </div>
    </div>
  );
};

export default HumanoidRoboticsComponent;
