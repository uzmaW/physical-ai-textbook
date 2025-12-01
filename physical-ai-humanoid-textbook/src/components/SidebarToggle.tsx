/**
 * Sidebar Toggle Component
 * Allows collapsing/expanding the left sidebar (table of contents)
 */

import React, { useState, useEffect } from 'react';
import { Menu, X } from 'lucide-react';
import { JSX } from 'react/jsx-runtime';

export function SidebarToggle(): JSX.Element {
  const [isCollapsed, setIsCollapsed] = useState(false);
  const [sidebarElement, setSidebarElement] = useState<HTMLElement | null>(null);

  const findSidebar = (): HTMLElement | null => {
    // Try multiple selectors to find the sidebar
    const selectors = [
      '.theme-doc-sidebar-container',
      '[class*="docSidebarContainer"]',
      '[class*="sidebar"]',
      'aside[class*="sidebar"]',
      '.docs-sidebar',
      'nav.menu'
    ];
    
    for (const selector of selectors) {
      const element = document.querySelector(selector) as HTMLElement;
      if (element && element.offsetParent !== null) {
        console.log('Found sidebar with selector:', selector);
        return element;
      }
    }
    
    console.warn('Sidebar not found');
    return null;
  };

  const applyCollapsedState = (collapsed: boolean, sidebar?: HTMLElement | null) => {
    const targetSidebar = sidebar || sidebarElement;
    
    if (targetSidebar) {
      if (collapsed) {
        targetSidebar.style.marginLeft = '-280px';
        targetSidebar.style.transition = 'margin-left 0.3s ease';
      } else {
        targetSidebar.style.marginLeft = '0';
        targetSidebar.style.transition = 'margin-left 0.3s ease';
      }
    }
    
    // Update body class for global styling
    if (collapsed) {
      document.body.classList.add('sidebar-collapsed');
    } else {
      document.body.classList.remove('sidebar-collapsed');
    }
  };

  const toggleSidebar = () => {
    const newState = !isCollapsed;
    setIsCollapsed(newState);
    applyCollapsedState(newState);
    
    // Save state to localStorage
    localStorage.setItem('sidebar-collapsed', String(newState));
  };

  useEffect(() => {
    // Find sidebar on mount
    const sidebar = findSidebar();
    if (sidebar) {
      setSidebarElement(sidebar);
      
      // Load saved state from localStorage (default is expanded)
      const saved = localStorage.getItem('sidebar-collapsed');
      if (saved === 'true') {
        setIsCollapsed(true);
        applyCollapsedState(true, sidebar);
      } else {
        // Ensure sidebar is expanded by default
        setIsCollapsed(false);
        applyCollapsedState(false, sidebar);
      }
    }

    // Add keyboard shortcut (Ctrl/Cmd + B)
    const handleKeyDown = (e: KeyboardEvent) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 'b') {
        e.preventDefault();
        toggleSidebar();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, []);

  // Re-apply state when sidebar element changes
  useEffect(() => {
    if (sidebarElement && isCollapsed) {
      applyCollapsedState(true);
    }
  }, [sidebarElement]);

  return (
    <button
      className={`sidebar-toggle ${isCollapsed ? 'collapsed' : ''}`}
      onClick={toggleSidebar}
      title={isCollapsed ? 'Show sidebar' : 'Hide sidebar'}
      aria-label={isCollapsed ? 'Show sidebar' : 'Hide sidebar'}
    >
      {isCollapsed ? <Menu /> : <X />}
    </button>
  );
}

export default SidebarToggle;
