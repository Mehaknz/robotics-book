import React from 'react';
import styles from './CustomFooter.module.css';
import modulesData from '../../data/modules.json';
import syllabusSectionsData from '../../data/syllabusSections.json';
import Link from '@docusaurus/Link';

const CustomFooter = () => {
  return (
    <footer className={styles.customFooter}>
      <div className={styles.footerContent}>
        <div className={styles.footerSection}>
          <h3 className={styles.sectionTitle}>Main Modules</h3>
          <ul className={styles.linksList}>
            {modulesData.map((module) => (
              <li key={module.id}>
                <Link to={`/docs/${module.slug}`}>{module.title}</Link>
              </li>
            ))}
          </ul>
        </div>

        <div className={styles.footerSection}>
          <h3 className={styles.sectionTitle}>Syllabus Sections</h3>
          <ul className={styles.linksList}>
            {syllabusSectionsData.map((section) => (
              <li key={section.id}>
                <Link to={`/docs/${section.chapters[0].slug}`}>{section.title}</Link>
              </li>
            ))}
          </ul>
        </div>

        <div className={styles.footerSection}>
          <h3 className={styles.sectionTitle}>Connect</h3>
          <ul className={styles.linksList}>
            <li><a href="https://github.com/your-repo" target="_blank" rel="noopener noreferrer">GitHub</a></li>
            <li><a href="https://twitter.com/your-twitter" target="_blank" rel="noopener noreferrer">Twitter</a></li>
            <li><a href="mailto:info@example.com">Contact Us</a></li>
          </ul>
        </div>
      </div>
      <div className={styles.footerBottom}>
        <p className={styles.copyright}>
          Copyright © {new Date().getFullYear()} Physical AI & Humanoid Robotics text book
        </p>
      </div>
    </footer>
  );
};

export default CustomFooter;
