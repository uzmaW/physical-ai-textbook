import { create } from 'zustand';
import { DocPage, ChatMessage, DocSection } from '../types';
import { MOCK_DOCS } from '../constants';
import { streamChatMessage } from '../services/geminiService';

interface AppState {
  // Theme
  theme: 'light' | 'dark';
  toggleTheme: () => void;

  // Navigation
  activePage: DocPage;
  setActivePage: (page: DocPage) => void;
  isLeftSidebarOpen: boolean;
  toggleLeftSidebar: () => void;
  closeLeftSidebar: () => void;

  // Mobile Chat State
  isMobileChatOpen: boolean;
  toggleMobileChat: () => void;
  closeMobileChat: () => void;

  // Chat Logic
  messages: ChatMessage[];
  isChatTyping: boolean;
  addMessage: (message: ChatMessage) => void;
  setTyping: (isTyping: boolean) => void;
  sendUserMessage: (text: string) => Promise<void>;
}

export const useAppStore = create<AppState>((set, get) => ({
  // Default to dark mode ("NinjaAI" style)
  theme: 'dark',
  toggleTheme: () => set((state) => ({ theme: state.theme === 'dark' ? 'light' : 'dark' })),

  activePage: MOCK_DOCS[0].pages[0],
  setActivePage: (page) => set({ activePage: page }),
  
  isLeftSidebarOpen: false,
  toggleLeftSidebar: () => set((state) => ({ isLeftSidebarOpen: !state.isLeftSidebarOpen })),
  closeLeftSidebar: () => set({ isLeftSidebarOpen: false }),

  isMobileChatOpen: false,
  toggleMobileChat: () => set((state) => ({ isMobileChatOpen: !state.isMobileChatOpen })),
  closeMobileChat: () => set({ isMobileChatOpen: false }),

  messages: [
    {
      id: 'welcome',
      role: 'model',
      text: "Hello. I'm your documentation assistant. How can I help you with this page?",
      timestamp: Date.now()
    }
  ],
  isChatTyping: false,
  addMessage: (msg) => set((state) => ({ messages: [...state.messages, msg] })),
  setTyping: (typing) => set({ isChatTyping: typing }),

  sendUserMessage: async (text) => {
    const { activePage, addMessage, setTyping } = get();
    if (!text.trim()) return;

    const userMsg: ChatMessage = {
      id: Date.now().toString(),
      role: 'user',
      text: text.trim(),
      timestamp: Date.now()
    };
    addMessage(userMsg);
    setTyping(true);

    const aiMsgId = (Date.now() + 1).toString();
    const aiMsg: ChatMessage = {
      id: aiMsgId,
      role: 'model',
      text: '',
      isStreaming: true,
      timestamp: Date.now()
    };
    addMessage(aiMsg);

    let accumulatedText = "";
    try {
        const stream = streamChatMessage(text, activePage.content);
        for await (const chunk of stream) {
            accumulatedText += chunk;
            set((state) => ({
                messages: state.messages.map(m =>
                    m.id === aiMsgId ? { ...m, text: accumulatedText } : m
                )
            }));
        }
    } catch(e) {
        console.error(e);
    } finally {
        setTyping(false);
        set((state) => ({
            messages: state.messages.map(m =>
                m.id === aiMsgId ? { ...m, isStreaming: false } : m
            )
        }));
    }
  }
}));