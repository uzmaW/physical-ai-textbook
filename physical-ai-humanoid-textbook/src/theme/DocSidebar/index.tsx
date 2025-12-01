/**
 * DocSidebar Wrapper
 * Adds header with collapse button above the sidebar
 */

import React from 'react';
import DocSidebarOriginal from '@theme-original/DocSidebar';
import { SidebarHeader } from '@site/src/components/SidebarHeader';
import type { Props } from '@theme/DocSidebar';

export default function DocSidebar(props: Props) {
  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100%' }}>
      <SidebarHeader />
      <div style={{ flex: 1, overflow: 'auto' }}>
        <DocSidebarOriginal {...props} />
      </div>
    </div>
  );
}
