/**
 * Chat Sidebar Component for Docusaurus Integration
 * Now uses RAGChatWidget with modern beautiful design
 */

import React from 'react';
import { RAGChatWidget } from '@site/src/components/RAGChatWidget';

function ChatSidebar() {
  return <RAGChatWidget />;
}

export default ChatSidebar;