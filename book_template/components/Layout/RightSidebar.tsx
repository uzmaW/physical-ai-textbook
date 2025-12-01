import React, { useRef, useEffect, useState } from 'react';
import { Send, Sparkles, StopCircle, X } from 'lucide-react';
import ReactMarkdown from 'react-markdown';
import { useAppStore } from '../../store/useStore';

export const RightSidebar: React.FC = () => {
  const { messages, isChatTyping, sendUserMessage, closeMobileChat } = useAppStore();
  const [input, setInput] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const textAreaRef = useRef<HTMLTextAreaElement>(null);

  // Auto-scroll to bottom
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isChatTyping]);

  const handleSend = () => {
    if (!input.trim() || isChatTyping) return;
    sendUserMessage(input);
    setInput('');
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <aside className="flex flex-col h-full w-full font-sans bg-transparent relative">
      {/* Header */}
      <div className="px-5 py-4 border-b border-slate-200 dark:border-slate-800 bg-white/80 dark:bg-slate-950/80 backdrop-blur-sm flex items-center justify-between sticky top-0 z-10 shrink-0 transition-colors duration-300">
        <div className="flex items-center gap-2 text-slate-800 dark:text-slate-200 font-medium">
          <Sparkles size={16} className="text-emerald-500 dark:text-emerald-400" />
          <span className="text-sm">Assistant</span>
        </div>
        <div className="flex items-center gap-3">
          <div className="flex items-center gap-1.5">
            <span className="w-1.5 h-1.5 rounded-full bg-emerald-500 animate-pulse"></span>
            <span className="text-[10px] font-semibold text-slate-500 uppercase tracking-wider">Online</span>
          </div>
          {/* Close button for mobile only */}
          <button 
            onClick={closeMobileChat}
            className="lg:hidden text-slate-400 hover:text-slate-600 dark:hover:text-slate-200 p-1"
          >
            <X size={18} />
          </button>
        </div>
      </div>

      {/* Messages Area */}
      <div className="flex-1 overflow-y-auto p-5 space-y-6 scroll-smooth bg-white dark:bg-transparent transition-colors duration-300">
        {messages.map((msg) => (
          <div 
            key={msg.id} 
            className={`flex flex-col gap-1.5 animate-in fade-in slide-in-from-bottom-2 duration-300 ${msg.role === 'user' ? 'items-end' : 'items-start'}`}
          >
            {/* Bubble */}
            <div className={`
              max-w-[90%] rounded-2xl px-4 py-3 text-sm shadow-sm leading-relaxed
              ${msg.role === 'user' 
                ? 'bg-emerald-600 text-white rounded-tr-sm' 
                : 'bg-slate-50 dark:bg-slate-900 border border-slate-200 dark:border-slate-800 text-slate-700 dark:text-slate-300 rounded-tl-sm'}
            `}>
              <div className="prose prose-sm dark:prose-invert max-w-none prose-p:leading-relaxed prose-p:my-1 text-inherit">
                <ReactMarkdown>{msg.text}</ReactMarkdown>
              </div>
              {msg.isStreaming && (
                 <span className="inline-block w-1 h-3 ml-1 align-middle bg-emerald-400 animate-pulse rounded-full"></span>
              )}
            </div>
            
            {/* Timestamp / Caption */}
            <span className="text-[10px] text-slate-400 dark:text-slate-600 px-1">
              {msg.role === 'user' ? 'You' : 'AI'}
            </span>
          </div>
        ))}
        <div ref={messagesEndRef} />
      </div>

      {/* Input Area */}
      <div className="p-4 bg-white dark:bg-slate-950 border-t border-slate-200 dark:border-slate-800 shrink-0 transition-colors duration-300">
        <div className="relative group">
          <textarea
            ref={textAreaRef}
            rows={1}
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder="Ask anything..."
            className="w-full pl-4 pr-12 py-3.5 bg-slate-50 dark:bg-slate-900 border border-slate-200 dark:border-slate-800 rounded-xl focus:outline-none focus:bg-white dark:focus:bg-slate-800 focus:ring-2 focus:ring-emerald-500/20 focus:border-emerald-500/50 transition-all resize-none text-sm text-slate-800 dark:text-slate-200 placeholder-slate-400 dark:placeholder-slate-500"
            style={{ minHeight: '50px', maxHeight: '150px' }}
          />
          <button
            onClick={handleSend}
            disabled={!input.trim() || isChatTyping}
            className={`
              absolute right-2.5 bottom-2.5 p-1.5 rounded-lg transition-all duration-200
              ${input.trim() && !isChatTyping
                ? 'bg-emerald-600 text-white hover:bg-emerald-500 shadow-sm shadow-emerald-900/20' 
                : 'text-slate-400 dark:text-slate-600 cursor-not-allowed hover:bg-slate-200 dark:hover:bg-slate-800'}
            `}
          >
           {isChatTyping ? <StopCircle size={18} className="animate-pulse text-emerald-500 dark:text-emerald-400" /> : <Send size={18} />}
          </button>
        </div>
      </div>
    </aside>
  );
};