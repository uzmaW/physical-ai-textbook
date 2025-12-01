import React from 'react';
import { Book, Menu, MessageSquareText, Moon, Sun } from 'lucide-react';
import { APP_NAME } from '../../constants';
import { useAppStore } from '../../store/useStore';

export const Navbar: React.FC = () => {
  const { toggleLeftSidebar, toggleMobileChat, theme, toggleTheme } = useAppStore();

  return (
    <nav className="h-14 border-b border-slate-200 dark:border-slate-800 bg-white/80 dark:bg-slate-950/80 backdrop-blur-md sticky top-0 z-50 flex items-center justify-between px-4 lg:px-6 transition-colors duration-300">
      <div className="flex items-center gap-4">
        <button 
          onClick={toggleLeftSidebar}
          className="lg:hidden p-2 -ml-2 text-slate-600 dark:text-slate-400 hover:bg-slate-100 dark:hover:bg-slate-800 hover:text-slate-900 dark:hover:text-slate-100 rounded-md transition-colors"
          aria-label="Toggle Navigation"
        >
          <Menu size={20} />
        </button>
        
        <div className="flex items-center gap-2 font-bold text-lg text-emerald-600 dark:text-emerald-500 cursor-default select-none">
          <Book size={24} />
          <span>{APP_NAME}</span>
        </div>
      </div>

      <div className="flex items-center gap-2">
        <button
          onClick={toggleTheme}
          className="p-2 text-slate-500 hover:text-emerald-600 dark:text-slate-400 dark:hover:text-emerald-400 hover:bg-slate-100 dark:hover:bg-slate-800 rounded-md transition-colors"
          aria-label="Toggle Theme"
        >
          {theme === 'dark' ? <Sun size={20} /> : <Moon size={20} />}
        </button>

        <div className="lg:hidden border-l border-slate-200 dark:border-slate-800 pl-2 ml-1">
          <button
            onClick={toggleMobileChat}
            className="p-2 -mr-2 text-slate-600 dark:text-slate-400 hover:text-emerald-600 dark:hover:text-emerald-400 hover:bg-emerald-50 dark:hover:bg-emerald-500/10 rounded-md transition-colors"
            aria-label="Toggle Assistant"
          >
            <MessageSquareText size={20} />
          </button>
        </div>
      </div>
    </nav>
  );
};