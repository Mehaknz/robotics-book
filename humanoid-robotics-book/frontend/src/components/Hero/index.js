import React from 'react';
import styles from './Hero.module.css';

const Hero = () => {
  // This function will force a browser navigation to the book index page.
  const handleStartLearning = () => {
    window.location.href = '/docs/Book-Index';
  };

  return (
    <div className={styles.heroContainer}>
      <div className={styles.heroContent}>
        <h1 className={styles.heroTitle}>Physical AI and Humanoid Robotics</h1>
        <p className={styles.heroSubtitle}>
          An advanced course on the future of intelligent robotics.
        </p>
        <a href="/docs/Book-Index" className={styles.ctaButton}>
          Start Learning
        </a>
      </div>
    </div>
  );
};

export default Hero;
