import React, { useEffect } from 'react';
import { Navbar } from './components/Layout/Navbar';
import { LeftSidebar } from './components/Layout/LeftSidebar';
import { RightSidebar } from './components/Layout/RightSidebar';
import { MainContent } from './components/Layout/MainContent';
import { useAppStore } from './store/useStore';

const App: React.FC = () => {
  const { isMobileChatOpen, closeMobileChat, theme } = useAppStore();

  // Sync theme with DOM
  useEffect(() => {
    if (theme === 'dark') {
      document.documentElement.classList.add('dark');
    } else {
      document.documentElement.classList.remove('dark');
    }
  }, [theme]);

  return (
    <div className="min-h-screen bg-white dark:bg-slate-950 flex flex-col font-sans text-slate-900 dark:text-slate-200 transition-colors duration-300">
      <Navbar />

      <div className="flex flex-1 relative max-w-[1920px] mx-auto w-full">
        {/* Left Navigation Sidebar - Fixed on Desktop, Drawer on Mobile */}
        <LeftSidebar />

        {/* Center Content */}
        <MainContent />

        {/* Right Chat Sidebar - Sticky Column on Desktop */}
        <div className="hidden lg:flex shrink-0 w-80 xl:w-96 flex-col border-l border-slate-200 dark:border-slate-800 h-[calc(100vh-3.5rem)] sticky top-14 bg-white dark:bg-slate-950 z-10 transition-colors duration-300">
          <RightSidebar />
        </div>
      </div>

      {/* Mobile Chat Drawer Overlay */}
      <div className={`lg:hidden fixed inset-0 z-50 pointer-events-none ${isMobileChatOpen ? 'pointer-events-auto' : ''}`}>
         {/* Backdrop */}
         <div 
            className={`absolute inset-0 bg-black/60 backdrop-blur-sm transition-opacity duration-300 ${isMobileChatOpen ? 'opacity-100' : 'opacity-0'}`}
            onClick={closeMobileChat}
         />
         {/* Drawer Panel */}
         <div className={`
            absolute top-0 right-0 h-full w-[85vw] max-w-sm bg-white dark:bg-slate-900 border-l border-slate-200 dark:border-slate-800 shadow-2xl flex flex-col
            transition-transform duration-300 ease-in-out
            ${isMobileChatOpen ? 'translate-x-0' : 'translate-x-full'}
         `}>
            <RightSidebar />
         </div>
      </div>

    </div>
  );
};

export default App;