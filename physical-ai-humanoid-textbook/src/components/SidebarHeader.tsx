/**
 * Sidebar Header Component
 * Displays above the table of contents with collapse button
 */

import React, { useState, useEffect } from 'react';
import { Menu, X } from 'lucide-react';
import styles from './SidebarHeader.module.css';

export function SidebarHeader(): JSX.Element {
  const [isCollapsed, setIsCollapsed] = useState(false);

  const toggleSidebar = () => {
    const newState = !isCollapsed;
    setIsCollapsed(newState);

    if (newState) {
      document.body.classList.add('sidebar-collapsed');
    } else {
      document.body.classList.remove('sidebar-collapsed');
    }

    localStorage.setItem('sidebar-collapsed', String(newState));
  };

  useEffect(() => {
    const saved = localStorage.getItem('sidebar-collapsed');
    const collapsed = saved === 'true';
    setIsCollapsed(collapsed);
    if (collapsed) {
      document.body.classList.add('sidebar-collapsed');
    } else {
      document.body.classList.remove('sidebar-collapsed');
    }
  }, []);

  return (
    <div className={styles.sidebarHeader}>
      <div className={styles.headerContent}>
        <h3 className={styles.title}>Table of Contents</h3>
      </div>
    </div>
  );
}

export default SidebarHeader;
