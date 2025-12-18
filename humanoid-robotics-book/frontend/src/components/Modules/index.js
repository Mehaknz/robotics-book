import React, { useState } from 'react';
import styles from './Modules.module.css';
import modules from '../../data/modules.json';
import Link from '@docusaurus/Link';

const Modules = () => {
  const [openModule, setOpenModule] = useState(null);

  const toggleModule = (id) => {
    setOpenModule(openModule === id ? null : id);
  };

  return (
    <div className={styles.modulesContainer}>
      <div className={styles.slider}>
        {modules.map((module) => (
          <div
            key={module.id}
            className={`${styles.moduleCard} ${
              openModule === module.id ? styles.open : ''
            }`}
            onClick={() => toggleModule(module.id)}
          >
            <div className={styles.moduleHeader}>
              <i className={`fas ${module.icon} ${styles.moduleIcon}`}></i>
              <h3 className={styles.moduleTitle}>{module.title}</h3>
            </div>
            <p className={styles.moduleDescription}>{module.description}</p>
            {openModule === module.id && (
              <div className={styles.chaptersContainer}>
                <h4 className={styles.chaptersTitle}>Chapters</h4>
                <ul className={styles.chaptersList}>
                  {module.chapters.map((chapter) => (
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

export default Modules;
