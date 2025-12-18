import React, { useState, useEffect } from 'react';
import styles from '../css/modal.module.css';

const AuthModals = () => {
  const [showSignIn, setShowSignIn] = useState(false);
  const [showSignUp, setShowSignUp] = useState(false);

  useEffect(() => {
    const handleOpenSignIn = () => setShowSignIn(true);
    const handleOpenSignUp = () => setShowSignUp(true);

    document.addEventListener('openSignInModal', handleOpenSignIn);
    document.addEventListener('openSignUpModal', handleOpenSignUp);

    return () => {
      document.removeEventListener('openSignInModal', handleOpenSignIn);
      document.removeEventListener('openSignUpModal', handleOpenSignUp);
    };
  }, []);

  const handleCloseModal = () => {
    setShowSignIn(false);
    setShowSignUp(false);
  };

  return (
    <>
      {showSignIn && (
        <div className={styles.modal} onClick={handleCloseModal}>
          <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
            <span className={styles.closeButton} onClick={handleCloseModal}>&times;</span>
            <h2>Sign In</h2>
            <form>
              <input type="email" placeholder="Email" required />
              <input type="password" placeholder="Password" required />
              <button type="submit">Sign In</button>
            </form>
          </div>
        </div>
      )}

      {showSignUp && (
        <div className={styles.modal} onClick={handleCloseModal}>
          <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
            <span className={styles.closeButton} onClick={handleCloseModal}>&times;</span>
            <h2>Sign Up</h2>
            <form>
              <input type="text" placeholder="Username" required />
              <input type="email" placeholder="Email" required />
              <input type="password" placeholder="Password" required />
              <button type="submit">Sign Up</button>
            </form>
          </div>
        </div>
      )}
    </>
  );
};

export default AuthModals;