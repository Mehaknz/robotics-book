import React from 'react';
import styles from '../components/style.module.css';

const Page = () => {
  return (
    <div className={styles.container}>
      <h1 className={styles.title}>Welcome to the Page</h1>
      <button className={styles.button}>Click Me</button>
    </div>
  );
};

export default Page;
