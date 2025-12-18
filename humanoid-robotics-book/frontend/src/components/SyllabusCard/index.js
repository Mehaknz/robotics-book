import React from 'react';
import Link from '@docusaurus/Link';
import styles from './SyllabusCard.module.css';

const SyllabusCard = ({ id, title, focus, chapters, toggleModule, isOpen }) => {
  return (
    // The entire card is now a clickable element to toggle the accordion
    <div className={`${styles.moduleCard} ${isOpen ? styles.open : ''}`} onClick={toggleModule}>
      <div className={styles.cardHeader}>
        <h2 className={styles.moduleTitle}>{title}</h2>
        {/* The chevron icon provides a visual indicator for the accordion state */}
        <span className={`${styles.chevronIcon} ${isOpen ? styles.chevronOpen : ''}`}></span>
      </div>
      <p className={styles.moduleFocus}>{focus}</p>

      {/* The chapters container with the smooth max-height transition */}
      <div className={`${styles.chaptersWrapper} ${isOpen ? styles.chaptersWrapperOpen : ''}`}>
        <div className={styles.chaptersContent}>
          <h3 className={styles.chaptersTitle}>Book Chapters</h3>
          <ul className={styles.chaptersList}>
            {chapters.map((chapter, index) => (
              <li key={index}>
                <Link to={`/docs/${chapter.slug}`} onClick={(e) => e.stopPropagation()}>
                  {chapter.title}
                </Link>
              </li>
            ))}
          </ul>
        </div>
      </div>
    </div>
  );
};

export default SyllabusCard;
