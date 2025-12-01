/**
 * Auth Button Component
 * Shows Login button or User menu with Logout
 */

import React, { useState, useEffect } from 'react';
import { logout, getStoredUser, type User } from '../services/authService';
import { AuthModal } from './AuthModal';
import styles from './AuthButton.module.css';

export function AuthButton() {
  const [user, setUser] = useState<User | null>(null);
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [showUserMenu, setShowUserMenu] = useState(false);

  useEffect(() => {
    // Check for stored user on mount
    setUser(getStoredUser());
  }, []);

  const handleLogout = async () => {
    await logout();
    setUser(null);
    setShowUserMenu(false);
    // Note: Chat history is preserved in backend per user
    // No page reload needed - chat continues for anonymous user
  };

  const handleAuthSuccess = () => {
    setUser(getStoredUser());
    setShowAuthModal(false);
  };

  if (user) {
    // User is logged in - show user menu
    return (
      <div className={styles.userMenu}>
        <button
          className={styles.userButton}
          onClick={() => setShowUserMenu(!showUserMenu)}
        >
          <span className={styles.userIcon}>👤</span>
          <span className={styles.userName}>{user.name}</span>
        </button>

        {showUserMenu && (
          <div className={styles.dropdown}>
            <div className={styles.dropdownItem}>
              <span className={styles.userEmail}>{user.email}</span>
            </div>
            <button
              className={styles.dropdownButton}
              onClick={handleLogout}
            >
              Logout
            </button>
          </div>
        )}
      </div>
    );
  }

  // User not logged in - show login button
  return (
    <>
      <button
        className={styles.loginButton}
        onClick={() => setShowAuthModal(true)}
      >
        Login
      </button>

      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        onSuccess={handleAuthSuccess}
      />
    </>
  );
}
