import React, { useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

export default function Root({ children }) {
  const location = useLocation();

  useEffect(() => {
    // Scroll to top when navigating to a new page (not when using hash links)
    if (!location.hash) {
      window.scrollTo(0, 0);
    }
  }, [location.pathname]); // Only trigger on pathname change, not hash

  return <>{children}</>;
}
