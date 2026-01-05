/**
 * RAG Chat Widget for right sidebar
 * Enhanced with selected-text support and backend integration
 * Modern design matching docusaurus-integration theme
 */

import React, { useState, useEffect, useRef } from 'react';
import { useUserStore } from '@site/src/store/userStore';
import { sendChatMessage, type Citation } from '@site/src/services/apiService';
import { AuthButton } from './AuthButton';
import { AuthModal } from './AuthModal';
import ReactMarkdown from 'react-markdown';
import {
  Sparkles,
  Plus,
  Trash2,
  Maximize2,
  Minimize2,
  X,
  Send,
  Loader2,
  ChevronLeft,
  MessageSquare,
  Settings,
  ExternalLink,
  ChevronRight,
  History,
  User
} from 'lucide-react';
import styles from './ChatUI.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
}

export function RAGChatWidget() {
  const { userProfile } = useUserStore();

  // State
  const [messages, setMessages] = useState<Message[]>([
    {
      id: 'welcome',
      role: 'assistant',
      content: "Hello! I'm your AI Tutor. I can help you understand concepts, work through examples, and answer questions about the textbook. How can I assist you?",
    },
  ]);
  const [input, setInput] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isExpanded, setIsExpanded] = useState<boolean>(() => {
    if (typeof window === 'undefined') return false;
    return localStorage.getItem('chat-expanded') === 'true';
  });
  const [isCollapsed, setIsCollapsed] = useState<boolean>(() => {
    if (typeof window === 'undefined') return false;
    return localStorage.getItem('chat-collapsed') === 'true';
  });
  const [showHistory, setShowHistory] = useState(false);
  const [showSettings, setShowSettings] = useState(false);
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [conversations, setConversations] = useState<any[]>([]);
  const [conversationId, setConversationId] = useState<string | null>(null);

  // Refs
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const messagesContainerRef = useRef<HTMLDivElement>(null);
  const textAreaRef = useRef<HTMLTextAreaElement>(null);
  const chatContainerRef = useRef<HTMLDivElement>(null);

  // Sync body classes and persist state
  useEffect(() => {
    if (typeof document === 'undefined') return;
    document.body.classList.toggle('chat-expanded', isExpanded && !isCollapsed);
    if (isExpanded && !isCollapsed) {
      localStorage.setItem('chat-expanded', 'true');
    } else {
      localStorage.setItem('chat-expanded', 'false');
    }
  }, [isExpanded, isCollapsed]);

  useEffect(() => {
    if (typeof document === 'undefined') return;
    document.body.classList.toggle('chat-collapsed', isCollapsed);
    localStorage.setItem('chat-collapsed', isCollapsed ? 'true' : 'false');
    if (isCollapsed) {
      // Ensure expanded is false when collapsed
      setIsExpanded(false);
    }
  }, [isCollapsed]);

  // Auto-scroll to bottom
  const scrollToBottom = () => {
    const container = messagesContainerRef.current;
    if (container) {
      container.scrollTo({ top: container.scrollHeight, behavior: 'smooth' });
    } else {
      messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isLoading]);

  // Detect text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim() || '';
      if (text && text.length > 10 && text.length < 500) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const sendMessage = async (messageText?: string, context?: string) => {
    const textToSend = messageText || input;
    if (!textToSend.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: textToSend,
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      // Get current page context
      const currentPath = typeof window !== 'undefined' ? window.location.pathname : '/';

      // Use the API service for proper backend communication
      const data = await sendChatMessage({
        message: textToSend,
        conversationId: conversationId || undefined,
        userEmail: userProfile?.email || undefined,
        selectedText: context || selectedText,
        userLevel: userProfile?.preferences?.difficulty_preference || 'intermediate',
        language: userProfile?.preferences?.language || 'en',
      });

      // Update conversation ID from response (if this is first message)
      if (!conversationId) {
        setConversationId(data.conversation_id);
      }

      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: data.answer,
        citations: data.citations,
      };

      setMessages(prev => [...prev, assistantMessage]);
      setSelectedText(''); // Clear selection after use
    } catch (error) {
      console.error('Chat error:', error);

      // Error message (apiService already provides a user-friendly fallback)
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: error instanceof Error ? error.message : '❌ An unexpected error occurred. Please check that the backend is running.',
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleNewConversation = () => {
    setMessages([
      {
        id: 'welcome',
        role: 'assistant',
        content: "Hello! I'm your AI Tutor. I can help you understand concepts, work through examples, and answer questions about the textbook. How can I assist you?",
      },
    ]);
    setConversationId(null); // Start new conversation with next message
    setShowHistory(false);
  };

  const handleClearChat = () => {
    if (window.confirm('Clear this conversation? (History is saved in the database)')) {
      handleNewConversation();
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const handleLoginClick = () => {
    setShowAuthModal(true);
  };

  const handleAuthSuccess = () => {
    setShowAuthModal(false);
  };

  if (isCollapsed) {
    return (
      <button
        className={styles.collapsedButton}
        onClick={() => {
          setIsCollapsed(false);
          // Keep previous expanded pref
        }}
        title="Open Chat"
        style={{
          position: 'fixed',
          top: '5rem',
          right: '1rem',
          zIndex: 1000,
          width: '48px',
          height: '48px',
          borderRadius: '8px',
          border: '1px solid rgb(209, 213, 219)',
          background: 'rgb(238, 246, 255)',
          color: 'rgb(59, 130, 246)',
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          gap: '4px',
          cursor: 'pointer',
          boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
          transition: 'all 0.2s ease',
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.transform = 'translateY(-2px)';
          e.currentTarget.style.boxShadow = '0 6px 16px rgba(0, 0, 0, 0.2)';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.transform = 'translateY(0)';
          e.currentTarget.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.15)';
        }}
      >
        <ChevronLeft size={20} />
        <Sparkles size={16} />
      </button>
    );
  }

  return (
    <div className={`${styles.chatContainer} ${isExpanded ? styles.chatContainerExpanded : ''}`}>
      {/* Header */}
      <div className={styles.chatHeader}>
        <div className={styles.headerTop}>
          <div className={styles.headerLeft}>
            <Sparkles size={18} className={styles.headerIcon} />
            <span className={styles.headerTitle}>AI Assistant</span>
         
          </div>

          <div className={styles.headerControls}>
            <button
              onClick={() => setShowHistory(!showHistory)}
              className={styles.iconButton}
              title="Conversation history"
            >
              <History size={14} />
            </button>

            <button
              onClick={handleNewConversation}
              className={styles.iconButton}
              title="New conversation"
            >
              <Plus size={14} />
            </button>

            <button
              onClick={() => {
                console.log('Settings clicked, current state:', showSettings);
                setShowSettings(prev => !prev);
              }}
              className={styles.iconButton}
              title="Settings"
            >
              <Settings size={14} />
            </button>

            <button
              onClick={handleLoginClick}
              className={styles.iconButton}
              title={userProfile ? 'Account' : 'Login'}
            >
              <User size={14} />
            </button>

            <button
              onClick={(e) => {
                e.stopPropagation();
                const newExpandedState = !isExpanded;
                setIsCollapsed(false);
                setIsExpanded(newExpandedState);
              }}
              className={styles.iconButton}
              title={isExpanded ? 'Normal width' : 'Expand width'}
            >
              {isExpanded ? <Minimize2 size={14} /> : <Maximize2 size={14} />}
            </button>
               <button
              onClick={() => {
                setIsCollapsed(true);
                setIsExpanded(false);
              }}
              className={styles.iconButton}
              title="Collapse chat"
            >
              <ChevronRight size={14} />
            </button>
          </div>
        </div>

      </div>

      {/* Auth Modal */}
      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        onSuccess={handleAuthSuccess}
      />

      {/* History Panel */}
      {showHistory && (
        <div className={styles.historyPanel}>
          <div className={styles.historyHeader}>
            <span className={styles.historyTitle}>Conversation History</span>
            <button
              onClick={() => setShowHistory(false)}
              className={styles.closeButton}
            >
              <X size={16} />
            </button>
          </div>
          <div className={styles.historyContent}>
            {conversations.length === 0 ? (
              <div className={styles.emptyState}>
                <MessageSquare size={32} opacity={0.3} />
                <p>No conversations yet</p>
              </div>
            ) : (
              conversations.map((conv) => (
                <div
                  key={conv.id}
                  className={styles.conversationItem}
                  onClick={() => {
                    // Load conversation logic here
                    setShowHistory(false);
                  }}
                >
                  <div className={styles.conversationInfo}>
                    <div className={styles.conversationTitle}>{conv.title}</div>
                    <div className={styles.conversationMeta}>{conv.messages} messages</div>
                  </div>
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      setConversations(prev => prev.filter(c => c.id !== conv.id));
                    }}
                    className={styles.deleteButton}
                  >
                    <Trash2 size={12} />
                  </button>
                </div>
              ))
            )}
          </div>
        </div>
      )}

      {/* Settings Modal */}
      {showSettings && (
        <>
          <div
            className={styles.settingsOverlay}
            onClick={() => setShowSettings(false)}
          />
          <div className={styles.settingsPanel}>
            <div className={styles.settingsHeader}>
              <span className={styles.settingsTitle}>Settings</span>
              <button
                onClick={() => setShowSettings(false)}
                className={styles.closeButton}
              >
                <X size={16} />
              </button>
            </div>
            <div className={styles.settingsContent}>
              <div className={styles.settingItem}>
                <label className={styles.settingLabel}>Difficulty Level</label>
                <select className={styles.settingSelect}>
                  <option value="beginner">Beginner</option>
                  <option value="intermediate">Intermediate</option>
                  <option value="advanced">Advanced</option>
                </select>
              </div>
              <div className={styles.settingItem}>
                <label className={styles.settingLabel}>Language</label>
                <select className={styles.settingSelect}>
                  <option value="en">English</option>
                  <option value="es">Spanish</option>
                  <option value="fr">French</option>
                </select>
              </div>
              <div className={styles.settingItem}>
                <label className={styles.settingLabel}>Theme</label>
                <select className={styles.settingSelect}>
                  <option value="light">Light</option>
                  <option value="dark">Dark</option>
                  <option value="auto">Auto</option>
                </select>
              </div>
            </div>
          </div>
        </>
      )}

      {/* Messages */}
      <div className={styles.messagesContainer} ref={messagesContainerRef}>
        {messages.map((msg) => (
          <div
            key={msg.id}
            className={`${styles.message} ${msg.role === 'user' ? styles.messageUser : styles.messageAssistant}`}
          >
            <div className={styles.messageBubble}>
              <div className={styles.messageContent}>
                <ReactMarkdown>{msg.content}</ReactMarkdown>
              </div>

              {/* Citations */}
              {msg.citations && msg.citations.length > 0 && (
                <div className={styles.citations}>
                  <div className={styles.citationsLabel}>Sources:</div>
                  <div className={styles.citationsList}>
                    {msg.citations.map((cite, idx) => (
                      <div key={idx} className={styles.citation}>
                        <ExternalLink size={12} />
                        <a
                          href={cite.url}
                          target="_blank"
                          rel="noopener noreferrer"
                        >
                          {cite.chapter}
                        </a>
                      </div>
                    ))}
                  </div>
                </div>
              )}
            </div>
          </div>
        ))}

        {isLoading && (
          <div className={styles.loadingContainer}>
            <div className={styles.loadingBubble}>
              <div className={styles.loadingDot}></div>
              <div className={styles.loadingDot}></div>
              <div className={styles.loadingDot}></div>
            </div>
          </div>
        )}

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
            disabled={isLoading}
          />
          <button
            onClick={() => sendMessage()}
            disabled={isLoading || !input.trim()}
            className={styles.sendButton}
            title="Send message (Enter)"
          >
            {isLoading ? <Loader2 size={16} className={styles.spinIcon} /> : <Send size={16} />}
          </button>
        </div>
      </div>
    </div>
  );
}
