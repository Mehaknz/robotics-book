import React, { useState } from 'react';
import styles from './SyllabusFolder.module.css';
import syllabusSections from '../../data/syllabusSections.json'; // Adjusted path
import Link from '@docusaurus/Link';

const SyllabusFolder = () => {
  const [openSection, setOpenSection] = useState(null);

  const toggleSection = (id) => {
    setOpenSection(openSection === id ? null : id);
  };

  return (
    <div className={styles.folderContainer}>
      <h2 className={styles.folderTitle}>Syllabus Section</h2>
      <div className={styles.slidesContainer}>
        {syllabusSections.map((section) => (
          <div
            key={section.id}
            className={`${styles.slideCard} ${
              openSection === section.id ? styles.open : ''
            }`}
            onClick={() => toggleSection(section.id)}
          >
            <div className={styles.slideHeader}>
              <i className={`fas ${section.icon} ${styles.slideIcon}`}></i>
              <h3 className={styles.slideTitle}>{section.title}</h3>
            </div>
            <p className={styles.slideDescription}>{section.description}</p>
            {openSection === section.id && (
              <div className={styles.chaptersContainer}>
                <h4 className={styles.chaptersTitle}>Chapters</h4>
                <ul className={styles.chaptersList}>
                  {section.chapters.map((chapter) => (
                    <li key={chapter.slug} className={styles.chapterItem}>
                      <Link to={`/docs/${chapter.slug}`}>{chapter.title}</Link>
                    </li>
                  ))}
                </ul>
              </div>
            )}
          </div>
        ))}
      </div>
    </div>
  );
};

export default SyllabusFolder;
