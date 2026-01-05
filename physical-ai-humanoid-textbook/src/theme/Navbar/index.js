/**
 * Custom Navbar with theme toggle and sidebar expand button
 */
import React from 'react';
import NavbarOriginal from '@theme-original/Navbar';
import { Menu, Sun, Moon } from 'lucide-react';
import { useColorMode } from '@docusaurus/theme-common';

export default function Navbar(props) {
  const { colorMode, setColorMode } = useColorMode();
  const isDarkMode = colorMode === 'dark';
  const [isSidebarCollapsed, setIsSidebarCollapsed] = React.useState(false);

  // Check sidebar state on mount
  React.useEffect(() => {
    const saved = localStorage.getItem('sidebar-collapsed');
    const collapsed = saved === 'true';
    setIsSidebarCollapsed(collapsed);
    if (collapsed) {
      document.body.classList.add('sidebar-collapsed');
    } else {
      document.body.classList.remove('sidebar-collapsed');
    }
  }, []);

  const toggleTheme = () => {
    setColorMode(isDarkMode ? 'light' : 'dark');
  };

  const toggleSidebar = () => {
    const newCollapsedState = !isSidebarCollapsed;
    setIsSidebarCollapsed(newCollapsedState);

    if (newCollapsedState) {
      document.body.classList.add('sidebar-collapsed');
    } else {
      document.body.classList.remove('sidebar-collapsed');
    }
    localStorage.setItem('sidebar-collapsed', String(newCollapsedState));
  };

  const floatingButtonStyle = {
    position: 'fixed',
    top: '5rem',
    width: '40px',
    height: '40px',
    borderRadius: '8px',
    border: '1px solid var(--ifm-color-emphasis-300)',
    background: 'var(--ifm-background-color)',
    color: 'var(--ifm-color-content)',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    cursor: 'pointer',
    boxShadow: '0 2px 8px rgba(0, 0, 0, 0.1)',
    transition: 'all 0.2s ease',
    zIndex: 1000,
  };

  const headerStyle = {
    position: 'fixed',
    top: 0,
    left: 0,
    right: 0,
    height: '60px',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: '0 1rem',
    background: 'var(--ifm-background-color)',
    borderBottom: '1px solid var(--ifm-toc-border-color)',
    zIndex: 1200,
  };

  const headerLeft = { display: 'flex', alignItems: 'center', gap: '0.5rem' };
  const headerRight = { display: 'flex', alignItems: 'center', gap: '0.5rem' };

  return (
    <>
      {/* Fixed Top Header */}
      {/* Invisible placeholder nav to satisfy Docusaurus hooks expecting .navbar */}
      <nav
        className="navbar"
        aria-hidden="true"
        style={{
          position: 'fixed',
          top: 0,
          left: 0,
          right: 0,
          height: '60px',
          background: 'transparent',
          visibility: 'hidden',
          pointerEvents: 'none',
          zIndex: 1000,
        }}
      />
      <div style={headerStyle}>
        <div style={headerLeft}>
          <button
            onClick={toggleSidebar}
            title={isSidebarCollapsed ? 'Open sidebar' : 'Close sidebar'}
            aria-label={isSidebarCollapsed ? 'Open sidebar' : 'Close sidebar'}
            aria-pressed={isSidebarCollapsed}
            style={{
              width: '40px',
              height: '40px',
              borderRadius: '8px',
              border: 'none',
              background: 'var(--ifm-color-primary)',
              color: '#fff',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              cursor: 'pointer',
              boxShadow: '0 2px 8px rgba(0, 0, 0, 0.15)',
              transition: 'all 0.2s ease',
              marginRight: '0.75rem',
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.transform = 'translateY(-1px)';
              e.currentTarget.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.2)';
              e.currentTarget.style.background = 'var(--ifm-color-primary-dark)';
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.transform = 'translateY(0)';
              e.currentTarget.style.boxShadow = '0 2px 8px rgba(0, 0, 0, 0.15)';
              e.currentTarget.style.background = 'var(--ifm-color-primary)';
            }}
          >
            <Menu size={18} />
          </button>
          <img src="/img/logo.svg" alt="Physical AI Logo" style={{ width: 28, height: 28, borderRadius: 6 }} />
          <strong>Physical AI Humanoid Textbook</strong>
        </div>
        <div style={headerRight}>
          <button
            className="navbar-theme-toggle"
            onClick={toggleTheme}
            title={`Switch to ${isDarkMode ? 'light' : 'dark'} mode`}
            aria-label={`Switch to ${isDarkMode ? 'light' : 'dark'} mode`}
            aria-pressed={isDarkMode}
            style={{
              width: '36px',
              height: '36px',
              borderRadius: '8px',
              border: '1px solid var(--ifm-color-emphasis-300)',
              background: 'var(--ifm-background-color)',
              color: 'var(--ifm-color-content)',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              cursor: 'pointer',
              boxShadow: '0 2px 8px rgba(0, 0, 0, 0.1)',
              transition: 'all 0.2s ease',
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.transform = 'translateY(-1px)';
              e.currentTarget.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.15)';
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.transform = 'translateY(0)';
              e.currentTarget.style.boxShadow = '0 2px 8px rgba(0, 0, 0, 0.1)';
            }}
          >
            {isDarkMode ? <Sun size={18} /> : <Moon size={18} />}
          </button>
        </div>
      </div>


      {/* Sidebar Expand Button removed to avoid duplicates; use SidebarHeader control */}

      {/* Theme toggle moved into fixed header above */}
    </>
  );
}