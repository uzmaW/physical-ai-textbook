/**
 * Chat Sidebar Component for Docusaurus Integration
 * Integrates the FastAPI chat system with Docusaurus book layout
 */

import React, { useState, useRef, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import { useDocusaurusContext } from '@docusaurus/core';
import clsx from 'clsx';
import {
  Send,
  Sparkles,
  MessageSquare,
  Plus,
  Settings,
  Trash2,
  ExternalLink,
  X,
  ChevronLeft,
  ChevronRight
} from 'lucide-react';
import ReactMarkdown from 'react-markdown';
import styles from './styles.module.css';

// Chat service - adapted from our previous implementation
import { chatService } from '../../services/chatService';

function ChatSidebar() {
  const { siteConfig } = useDocusaurusContext();
  const location = useLocation();
  
  // Chat state
  const [messages, setMessages] = useState([
    {
      id: 'welcome',
      role: 'assistant',
      content: "Hello! I'm your robotics education assistant. I can help you understand concepts, work through labs, and answer questions about this chapter. How can I assist you?",
      timestamp: Date.now()
    }
  ]);
  
  const [input, setInput] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const [conversations, setConversations] = useState([]);
  const [currentConversationId, setCurrentConversationId] = useState(null);
  const [showConversations, setShowConversations] = useState(false);
  const [showSettings, setShowSettings] = useState(false);
  const [isCollapsed, setIsCollapsed] = useState(false);
  
  // Refs
  const messagesEndRef = useRef(null);
  const textAreaRef = useRef(null);
  
  // Auto-scroll to bottom
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isTyping]);

  // Load conversations on mount
  useEffect(() => {
    loadConversations();
  }, []);

  // Context awareness based on current page
  const getCurrentPageContext = () => {
    const path = location.pathname;
    let weekNumber = null;
    let chapterType = 'general';
    let difficulty = 'beginner';
    
    // Extract week number
    const weekMatch = path.match(/week-?(\d+)/i);
    if (weekMatch) {
      weekNumber = parseInt(weekMatch[1]);
    }
    
    // Determine chapter type
    if (path.includes('lab')) {
      chapterType = 'lab';
      difficulty = 'intermediate';
    } else if (path.includes('theory') || path.includes('concept')) {
      chapterType = 'theory';
    } else if (path.includes('advanced')) {
      difficulty = 'advanced';
    }
    
    return {
      week_filter: weekNumber,
      chunk_types: chapterType === 'lab' ? ['lab', 'code'] : ['chapter', 'paragraph'],
      difficulty_level: difficulty,
      robotics_topics: extractTopicsFromPath(path)
    };
  };

  const extractTopicsFromPath = (path) => {
    const topics = [];
    if (path.includes('perception') || path.includes('vision') || path.includes('sensor')) {
      topics.push('perception');
    }
    if (path.includes('navigation') || path.includes('path')) {
      topics.push('navigation');
    }
    if (path.includes('manipulation') || path.includes('arm') || path.includes('gripper')) {
      topics.push('manipulation');
    }
    if (path.includes('control')) {
      topics.push('control');
    }
    if (path.includes('simulation') || path.includes('gazebo') || path.includes('isaac')) {
      topics.push('simulation');
    }
    return topics;
  };

  const sendMessage = async () => {
    if (!input.trim() || isTyping) return;

    const userMessage = {
      id: Date.now().toString(),
      role: 'user',
      content: input.trim(),
      timestamp: Date.now()
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsTyping(true);

    const aiMessageId = (Date.now() + 1).toString();
    const aiMessage = {
      id: aiMessageId,
      role: 'assistant',
      content: '',
      isStreaming: true,
      timestamp: Date.now()
    };

    setMessages(prev => [...prev, aiMessage]);

    try {
      const contextFilters = getCurrentPageContext();
      
      const chatRequest = {
        message: input.trim(),
        conversation_id: currentConversationId,
        context_filters: contextFilters,
        max_context_chunks: 5,
        include_citations: true
      };

      let accumulatedContent = '';
      let messageMetadata = {};

      const stream = chatService.sendMessage(chatRequest);
      
      for await (const chunk of stream) {
        if (chunk.startsWith('__METADATA__')) {
          const metadataStr = chunk.substring(12);
          try {
            messageMetadata = JSON.parse(metadataStr);
            setCurrentConversationId(messageMetadata.conversation_id);
          } catch (e) {
            console.warn('Failed to parse metadata:', e);
          }
        } else {
          accumulatedContent += chunk;
          setMessages(prev => prev.map(msg =>
            msg.id === aiMessageId 
              ? { ...msg, content: accumulatedContent, metadata: messageMetadata }
              : msg
          ));
        }
      }
    } catch (error) {
      console.error('Chat error:', error);
      setMessages(prev => prev.map(msg =>
        msg.id === aiMessageId 
          ? { 
              ...msg, 
              content: "I apologize, but I'm having trouble connecting to the chat service. Please check your connection settings.",
              isStreaming: false 
            }
          : msg
      ));
    } finally {
      setIsTyping(false);
      setMessages(prev => prev.map(msg =>
        msg.id === aiMessageId ? { ...msg, isStreaming: false } : msg
      ));
    }
  };

  const loadConversations = async () => {
    try {
      const conversations = await chatService.getConversations();
      setConversations(conversations);
    } catch (error) {
      console.error('Failed to load conversations:', error);
    }
  };

  const newConversation = () => {
    chatService.newConversation();
    setCurrentConversationId(null);
    setMessages([
      {
        id: 'welcome',
        role: 'assistant',
        content: "Hello! I'm your robotics education assistant. How can I help you with this chapter?",
        timestamp: Date.now()
      }
    ]);
    setShowConversations(false);
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  if (isCollapsed) {
    return (
      <div className={clsx(styles.chatSidebar, styles.collapsed)}>
        <button
          className={styles.expandButton}
          onClick={() => setIsCollapsed(false)}
          title="Open Chat"
        >
          <ChevronLeft size={20} />
          <Sparkles size={16} />
        </button>
      </div>
    );
  }

  return (
    <div className={styles.chatSidebar}>
      {/* Header */}
      <div className={styles.chatHeader}>
        <div className={styles.headerLeft}>
          <Sparkles size={16} className={styles.headerIcon} />
          <span className={styles.headerTitle}>AI Assistant</span>
        </div>
        
        <div className={styles.headerControls}>
          <button
            onClick={() => setShowConversations(!showConversations)}
            className={styles.controlButton}
            title="Conversations"
          >
            <MessageSquare size={14} />
          </button>
          
          <button
            onClick={newConversation}
            className={styles.controlButton}
            title="New conversation"
          >
            <Plus size={14} />
          </button>
          
          <button
            onClick={() => setShowSettings(true)}
            className={styles.controlButton}
            title="Settings"
          >
            <Settings size={14} />
          </button>
          
          <button
            onClick={() => setIsCollapsed(true)}
            className={styles.controlButton}
            title="Collapse chat"
          >
            <ChevronRight size={14} />
          </button>
        </div>
      </div>

      {/* Conversations List */}
      {showConversations && (
        <div className={styles.conversationsList}>
          <div className={styles.conversationsHeader}>
            <span>Recent Conversations</span>
          </div>
          <div className={styles.conversationsContent}>
            {conversations.map((conv) => (
              <div
                key={conv.conversation_id}
                className={clsx(styles.conversationItem, {
                  [styles.active]: currentConversationId === conv.conversation_id
                })}
                onClick={async () => {
                  try {
                    const messages = await chatService.getConversation(conv.conversation_id);
                    setMessages(messages);
                    setCurrentConversationId(conv.conversation_id);
                    setShowConversations(false);
                  } catch (error) {
                    console.error('Failed to load conversation:', error);
                  }
                }}
              >
                <div className={styles.conversationInfo}>
                  <div className={styles.conversationTitle}>{conv.title}</div>
                  <div className={styles.conversationMeta}>
                    {conv.message_count} messages
                  </div>
                </div>
                <button
                  onClick={async (e) => {
                    e.stopPropagation();
                    try {
                      await chatService.deleteConversation(conv.conversation_id);
                      setConversations(prev => 
                        prev.filter(c => c.conversation_id !== conv.conversation_id)
                      );
                      if (currentConversationId === conv.conversation_id) {
                        newConversation();
                      }
                    } catch (error) {
                      console.error('Failed to delete conversation:', error);
                    }
                  }}
                  className={styles.deleteButton}
                >
                  <Trash2 size={12} />
                </button>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* Messages */}
      <div className={styles.messagesContainer}>
        {messages.map((msg) => (
          <div
            key={msg.id}
            className={clsx(styles.message, {
              [styles.userMessage]: msg.role === 'user',
              [styles.assistantMessage]: msg.role === 'assistant'
            })}
          >
            <div className={styles.messageContent}>
              <ReactMarkdown className={styles.markdown}>
                {msg.content}
              </ReactMarkdown>
              {msg.isStreaming && (
                <span className={styles.streamingIndicator}>▋</span>
              )}
            </div>
            
            {/* Citations for assistant messages */}
            {msg.role === 'assistant' && msg.metadata?.retrieved_contexts && (
              <div className={styles.citations}>
                <div className={styles.citationsLabel}>Sources:</div>
                <div className={styles.citationsList}>
                  {msg.metadata.retrieved_contexts.slice(0, 3).map((context, idx) => (
                    <div key={idx} className={styles.citation}>
                      <ExternalLink size={10} />
                      <span>{context.section_title || context.source_file}</span>
                    </div>
                  ))}
                </div>
              </div>
            )}
          </div>
        ))}
        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <div className={styles.inputContainer}>
        <div className={styles.inputWrapper}>
          <textarea
            ref={textAreaRef}
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder="Ask about this chapter..."
            className={styles.messageInput}
            rows={1}
            disabled={isTyping}
          />
          <button
            onClick={sendMessage}
            disabled={!input.trim() || isTyping}
            className={styles.sendButton}
          >
            <Send size={16} />
          </button>
        </div>
      </div>
    </div>
  );
}

export default ChatSidebar;