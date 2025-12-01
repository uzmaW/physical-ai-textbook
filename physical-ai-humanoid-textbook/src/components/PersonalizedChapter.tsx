/**
 * PersonalizedChapter Component
 * Wraps chapter content with personalization and translation features
 */

import React, { useState, useEffect, useRef } from 'react';
import { useUserStore } from '@site/src/store/userStore';
import styles from './PersonalizedChapter.module.css';

interface PersonalizedChapterProps {
  id: string;
  children: React.ReactNode;
}

export function PersonalizedChapter({ id, children }: PersonalizedChapterProps): JSX.Element {
  const { userProfile, updatePreferences } = useUserStore();
  const chapterContentRef = useRef<HTMLDivElement>(null);

  const [difficulty, setDifficulty] = useState(
    userProfile?.preferences?.difficulty_preference || 'intermediate'
  );
  const [isUrdu, setIsUrdu] = useState(false);
  const [urduContent, setUrduContent] = useState<string | null>(null);
  const [isTranslating, setIsTranslating] = useState(false);

  // Cycle through difficulty levels
  const handlePersonalize = () => {
    const levels: Array<'beginner' | 'intermediate' | 'advanced'> = ['beginner', 'intermediate', 'advanced'];
    const currentIndex = levels.indexOf(difficulty);
    const nextLevel = levels[(currentIndex + 1) % 3];

    setDifficulty(nextLevel);
    updatePreferences({ difficulty_preference: nextLevel });
  };

  // Get API base URL (support both dev and prod)
  const getApiBaseUrl = (): string => {
    if (typeof window === 'undefined') {
      return '/api';
    }

    // In development, use localhost:8000 directly
    // In production, use /api which will be handled by the backend
    const isDev = window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1';

    if (isDev) {
      return 'http://localhost:8000/api';
    }

    // Production: use relative path
    return '/api';
  };

  // Toggle Urdu translation
  const handleTranslate = async () => {
    if (isUrdu) {
      setIsUrdu(false);
      return;
    }

    setIsTranslating(true);

    try {
      const userId = userProfile?.id || 'guest';
      const apiBase = getApiBaseUrl();
      const textContent = extractTextContent();

      console.log(`[Translation] Extracted content length: ${textContent.length} chars`);
      if (textContent.length > 0) {
        console.log(`[Translation] First 300 chars: "${textContent.substring(0, 300)}..."`);
      }

      if (!textContent || textContent.trim().length === 0) {
        console.error('[Translation] ✗ Failed to extract content from chapter');
        console.error('[Translation] Chapter ref available?', !!chapterContentRef.current);
        console.error('[Translation] DOM elements:', {
          'prose': !!document.querySelector('.chapter-content .prose'),
          'chapter-content': !!document.querySelector('.chapter-content'),
          'article': !!document.querySelector('article'),
          'main': !!document.querySelector('main')
        });
        alert('Error: No content to translate. The chapter content could not be extracted. Please try refreshing the page.');
        setIsTranslating(false);
        return;
      }

      console.log(`[Translation] Starting translation for chapter: ${id}`);
      console.log(`[Translation] API Base: ${apiBase}`);
      console.log(`[Translation] Content length: ${textContent.length} chars`);

      // Check cache first
      const cacheUrl = `${apiBase}/translate/cached?userId=${userId}&chapterId=${id}`;
      console.log(`[Translation] Checking cache: ${cacheUrl}`);
      console.log(`[Translation] Full URL will be:`, window.location.origin + cacheUrl);

      const cacheResponse = await fetch(cacheUrl);

      if (cacheResponse.ok) {
        const cached = await cacheResponse.json();
        console.log(`[Translation] ✓ Found cached translation (indexed: ${cached.indexed})`);
        setUrduContent(cached.content);
        setIsUrdu(true);
      } else {
        console.log(`[Translation] Cache miss (${cacheResponse.status}), fetching new translation...`);
        
        // Translate and cache
        const translateUrl = `${apiBase}/translate/`;
        console.log(`[Translation] POST to ${translateUrl}`);
        console.log(`[Translation] Full URL will be:`, window.location.origin + translateUrl);

        const translateResponse = await fetch(translateUrl, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            userId,
            chapterId: id,
            content: textContent,
            targetLang: 'ur'
          })
        });

        if (translateResponse.ok) {
          const result = await translateResponse.json();
          console.log(`[Translation] ✓ Translation success (indexed: ${result.indexed})`);
          setUrduContent(result.translatedContent);
          setIsUrdu(true);
        } else {
          const errorText = await translateResponse.text();
          console.error(`[Translation] ✗ Failed with status ${translateResponse.status}:`, errorText);
          alert(`Translation failed: ${translateResponse.status} ${translateResponse.statusText}`);
        }
      }
    } catch (error) {
      console.error('[Translation] ✗ Exception:', error);
      alert('Translation service unavailable. Check console for details.');
    } finally {
      setIsTranslating(false);
    }
  };

  // Extract text content from chapter DOM
  const extractTextContent = (): string => {
    let text = '';
    
    // Method 1: Use ref to chapter content div
    if (chapterContentRef.current) {
      const element = chapterContentRef.current;
      const clone = element.cloneNode(true) as HTMLElement;
      
      // Remove interactive elements and buttons
      clone.querySelectorAll('button, [role="button"], a.anchor, .translate-btn, .personalize-btn').forEach(el => {
        el.remove();
      });
      
      text = clone.textContent || '';
      console.log(`[Translation] Extracted from ref: ${text.length} chars`);
    }
    
    // Method 2: Fallback to DOM selector if ref not available
    if (!text || text.trim().length === 0) {
      const targetElement = document.querySelector('.chapter-content .prose') || 
                           document.querySelector('.chapter-content');
      
      if (targetElement) {
        const clone = targetElement.cloneNode(true) as HTMLElement;
        clone.querySelectorAll('button, [role="button"], a.anchor').forEach(el => el.remove());
        text = clone.textContent || '';
        console.log(`[Translation] Extracted from DOM: ${text.length} chars`);
      }
    }
    
    // Method 3: Last resort - try to extract from Docusaurus main article
    if (!text || text.trim().length === 0) {
      const articleElement = document.querySelector('article') || 
                            document.querySelector('main');
      
      if (articleElement) {
        const clone = articleElement.cloneNode(true) as HTMLElement;
        // Remove header, nav, buttons
        clone.querySelectorAll('header, nav, button, .navbar, .pagination-nav').forEach(el => {
          el.remove();
        });
        text = clone.textContent || '';
        console.log(`[Translation] Extracted from article: ${text.length} chars`);
      }
    }
    
    // Clean up extra whitespace and normalize
    const cleaned = text
      .replace(/\s+/g, ' ')  // Collapse multiple spaces
      .replace(/\n\s*\n/g, '\n')  // Remove extra blank lines
      .trim();
    
    console.log(`[Translation] Final cleaned content: ${cleaned.length} chars`);
    return cleaned;
  };

  return (
    <div className={styles.personalizedChapter}>
      {/* Chapter Header with Controls */}
      <div className={styles.chapterHeader}>
        <div>
          <span
            className={`${styles.difficultyBadge} ${
              difficulty === 'beginner'
                ? styles.beginner
                : difficulty === 'intermediate'
                ? styles.intermediate
                : styles.advanced
            }`}
          >
            {difficulty.charAt(0).toUpperCase() + difficulty.slice(1)}
          </span>
        </div>

        <div className={styles.controlButtons}>
          <button
            onClick={handlePersonalize}
            className={`${styles.controlButton} ${styles.personalizeButton}`}
            title="Cycle through difficulty levels: Beginner → Intermediate → Advanced"
          >
            🎯 Personalize
          </button>

          <button
            onClick={handleTranslate}
            className={`${styles.controlButton} ${styles.translateButton}`}
            disabled={isTranslating}
            title={isUrdu ? 'Switch back to English' : 'Translate to Urdu'}
          >
            {isTranslating ? '⏳ Translating...' : isUrdu ? 'English' : 'ترجمہ'}
          </button>
        </div>
      </div>

      {/* Chapter Content */}
      <div
        ref={chapterContentRef}
        className={styles.chapterContent}
        data-difficulty={difficulty}
      >
        {isUrdu && urduContent ? (
          <div
            className={`prose dark:prose-invert max-w-none ${styles.urduContent}`}
            dangerouslySetInnerHTML={{ __html: urduContent }}
          />
        ) : (
          <div className="prose dark:prose-invert max-w-none">
            {children}
          </div>
        )}
      </div>
    </div>
  );
}
