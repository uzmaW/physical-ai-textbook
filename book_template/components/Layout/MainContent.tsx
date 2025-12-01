import React from 'react';
import ReactMarkdown from 'react-markdown';
import { useAppStore } from '../../store/useStore';

export const MainContent: React.FC = () => {
  const { activePage } = useAppStore();

  return (
    <main className="flex-1 min-w-0 py-8 px-6 lg:px-12 xl:px-16 pb-24 bg-white dark:bg-slate-950 transition-colors duration-300">
      <div className="max-w-3xl mx-auto">
        <div className="mb-6">
          <div className="text-sm font-medium text-emerald-600 dark:text-emerald-500 mb-2 flex items-center gap-2">
            Documentation <span className="text-slate-400 dark:text-slate-600">/</span> <span className="text-slate-500 dark:text-slate-400">{activePage.title}</span>
          </div>
        </div>
        
        <article className="prose prose-slate dark:prose-invert lg:prose-lg max-w-none 
          prose-headings:font-bold prose-headings:tracking-tight 
          prose-a:text-emerald-600 dark:prose-a:text-emerald-400 hover:prose-a:text-emerald-500 dark:hover:prose-a:text-emerald-300 
          prose-img:rounded-xl 
          prose-pre:bg-slate-100 dark:prose-pre:bg-slate-900 prose-pre:border prose-pre:border-slate-200 dark:prose-pre:border-slate-800 
          prose-code:text-emerald-700 dark:prose-code:text-emerald-300 prose-code:bg-emerald-50 dark:prose-code:bg-emerald-950/30 prose-code:px-1 prose-code:py-0.5 prose-code:rounded-md prose-code:before:content-none prose-code:after:content-none 
          font-normal text-slate-700 dark:text-slate-300 transition-colors duration-300">
          <ReactMarkdown>{activePage.content}</ReactMarkdown>
        </article>
        
        <div className="mt-16 pt-8 border-t border-slate-200 dark:border-slate-800 flex justify-between text-sm font-medium text-slate-500">
          <button className="hover:text-emerald-600 dark:hover:text-emerald-400 transition-colors">
            ← Previous Page
          </button>
          <button className="hover:text-emerald-600 dark:hover:text-emerald-400 transition-colors">
            Next Page →
          </button>
        </div>
      </div>
    </main>
  );
};