/**
 * Chat Sidebar Component for Docusaurus Integration
 * Now uses RAGChatWidget with modern beautiful design
 */

import React from 'react';
import { RAGChatWidget } from '@site/src/components/RAGChatWidget';

function ChatSidebar() {
  React.useEffect(() => {
    // Mark chat as visible so layout can reserve space
    document.body.classList.add('chat-visible');
    // Ensure not collapsed at start influences layout; rely on widget for state
    return () => {
      document.body.classList.remove('chat-visible');
    };
  }, []);

  return (
    <div className="chat-sidebar-container">
      <RAGChatWidget />
    </div>
  );
}

export default ChatSidebar;