/**
 * DocSidebar Wrapper
 * Adds header with collapse button above the sidebar
 */

import React from 'react';
import {useLocation} from '@docusaurus/router';
import DocSidebarOriginal from '@theme-original/DocSidebar';
// Removed SidebarHeader to avoid extra header inside sidebar
import type { Props } from '@theme/DocSidebar';

export default function DocSidebar(props: Props) {
  const scrollContainerRef = React.useRef<HTMLDivElement | null>(null);
  const location = useLocation();

  React.useEffect(() => {
    // After route changes, scroll active sidebar item into view smoothly
    const container = scrollContainerRef.current;
    if (!container) return;
    try {
      const active = container.querySelector('.menu__link--active, .menu__list-item--active');
      if (active && container) {
        const el = active as HTMLElement | null;
        const containerHeight = container?.clientHeight || 0;
        const elOffsetTop = el?.offsetTop || 0;
        const elHeight = el?.clientHeight || 0;
        const targetTop = elOffsetTop - containerHeight / 2 + elHeight / 2;
        container.scrollTo({ top: Math.max(0, targetTop), behavior: 'smooth' });
      }
    } catch (e) {
      // Prevent runtime errors from breaking rendering
      // console.debug('Sidebar auto-scroll skipped:', e);
    }
  }, [location.pathname]);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100%' }}>
      <div ref={scrollContainerRef} style={{ flex: 1, overflow: 'auto' }}>
        <DocSidebarOriginal {...props} />
      </div>
    </div>
  );
}
