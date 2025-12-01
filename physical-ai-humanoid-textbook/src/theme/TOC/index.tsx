/**
 * TOC Component - Replaced with RAGChatWidget
 */

import React from 'react';
import { RAGChatWidget } from '../../components/RAGChatWidget';

export default function TOC() {
  const [isChatCollapsed, setIsChatCollapsed] = React.useState(false);

  React.useEffect(() => {
    // Listen for chat collapse state changes
    const checkCollapsed = () => {
      const expandButton = document.querySelector('.expandChatButton');
      setIsChatCollapsed(!!expandButton);
    };
    
    const observer = new MutationObserver(checkCollapsed);
    observer.observe(document.body, { childList: true, subtree: true });
    
    return () => observer.disconnect();
  }, []);

  if (isChatCollapsed) {
    return null; // Hide the TOC container when chat is collapsed
  }

  return (
    <div
      className="custom-toc-wrapper"
      style={{
        position: 'sticky',
        top: 0,
        height: '100vh',
        width: '370px',
        maxWidth: '370px',
        minWidth: '370px',
        overflow: 'hidden',
        padding: 0,
        display: 'flex',
        flexDirection: 'column',
      }}
    >
      <RAGChatWidget />
    </div>
  );
}
