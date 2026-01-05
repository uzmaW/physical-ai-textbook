import React, { useState, useEffect } from 'react';
import { Menu } from 'lucide-react';

// Root wrapper with expand button for collapsed sidebar
export default function Root({children}) {
  const [isCollapsed, setIsCollapsed] = useState(false);

  useEffect(() => {
    const checkCollapsed = () => {
      setIsCollapsed(document.body.classList.contains('sidebar-collapsed'));
    };
    
    checkCollapsed();
    
    // Watch for changes
    const observer = new MutationObserver(checkCollapsed);
    observer.observe(document.body, { attributes: true, attributeFilter: ['class'] });
    
    return () => observer.disconnect();
  }, []);

  const expandSidebar = () => {
    document.body.classList.remove('sidebar-collapsed');
    localStorage.setItem('sidebar-collapsed', 'false');
  };

  return (
    <>
      {children}
    </>
  );
}
