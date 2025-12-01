import React from 'react';
import { ChevronRight } from 'lucide-react';
import { useAppStore } from '../../store/useStore';
import { MOCK_DOCS } from '../../constants';

export const LeftSidebar: React.FC = () => {
  const { activePage, setActivePage, isLeftSidebarOpen, closeLeftSidebar } = useAppStore();
  const sections = MOCK_DOCS;

  return (
    <>
      {/* Mobile Backdrop */}
      {isLeftSidebarOpen && (
        <div 
          className="fixed inset-0 bg-black/60 backdrop-blur-sm z-40 lg:hidden"
          onClick={closeLeftSidebar}
        />
      )}

      {/* Sidebar Container */}
      <aside className={`
        fixed lg:sticky top-14 left-0 h-[calc(100vh-3.5rem)]
        w-72 bg-white dark:bg-slate-950 border-r border-slate-200 dark:border-slate-800 overflow-y-auto
        transition-transform duration-300 ease-in-out z-50
        ${isLeftSidebarOpen ? 'translate-x-0' : '-translate-x-full lg:translate-x-0'}
      `}>
        <div className="p-4 lg:py-6">
          {sections.map((section, idx) => (
            <div key={idx} className="mb-6">
              <h3 className="px-3 mb-2 text-xs font-bold text-slate-500 uppercase tracking-wider">
                {section.title}
              </h3>
              <ul className="space-y-0.5">
                {section.pages.map((page) => {
                  const isActive = activePage.id === page.id;
                  return (
                    <li key={page.id}>
                      <button
                        onClick={() => {
                          setActivePage(page);
                          if (window.innerWidth < 1024) closeLeftSidebar();
                        }}
                        className={`
                          w-full flex items-center justify-between px-3 py-2 rounded-md text-sm transition-colors text-left
                          ${isActive 
                            ? 'bg-emerald-50 text-emerald-700 dark:bg-emerald-500/10 dark:text-emerald-400 font-medium' 
                            : 'text-slate-600 dark:text-slate-400 hover:bg-slate-100 dark:hover:bg-slate-900 hover:text-slate-900 dark:hover:text-slate-200'}
                        `}
                      >
                        <span>{page.title}</span>
                        {isActive && <ChevronRight size={14} className="text-emerald-500" />}
                      </button>
                    </li>
                  );
                })}
              </ul>
            </div>
          ))}
          
          <div className="mt-8 px-3">
             <div className="bg-slate-50 dark:bg-slate-900/50 rounded-lg p-4 border border-slate-200 dark:border-slate-800">
               <p className="text-xs text-slate-500 dark:text-slate-400 font-medium mb-2">Need Help?</p>
               <p className="text-xs text-slate-600 dark:text-slate-500">Ask the AI assistant on the right panel for instant answers.</p>
             </div>
          </div>
        </div>
      </aside>
    </>
  );
};