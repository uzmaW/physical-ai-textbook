/**
 * Custom Docusaurus Layout with Integrated Chat UI
 * Layout: Cover -> TOC Left | Main Content Center | Chat Right
 */

import React from 'react';
import clsx from 'clsx';
import ErrorBoundary from '@docusaurus/ErrorBoundary';
import {PageMetadata} from '@docusaurus/theme-common';
import {useKeyboardNavigation} from '@docusaurus/theme-common/internal';
import SkipToContent from '@theme/SkipToContent';
import AnnouncementBar from '@theme/AnnouncementBar';
import Navbar from '@theme/Navbar';
import Footer from '@theme/Footer';
import LayoutProvider from '@theme/Layout/Provider';
import ErrorPageContent from '@theme/ErrorPageContent';
import {ThemeClassNames} from '@docusaurus/theme-common';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import BookCover from '../BookCover';
import ChatSidebar from '../ChatSidebar';
import TableOfContents from '../TableOfContents';

import styles from './styles.module.css';

function Layout(props) {
  const {
    children,
    noFooter,
    wrapperClassName,
    pageClassName,
  } = props;

  const {siteConfig} = useDocusaurusContext();
  const {
    title: siteTitle,
    tagline,
    customFields: {
      bookConfig = {},
    } = {},
  } = siteConfig;

  useKeyboardNavigation();

  return (
    <LayoutProvider>
      <PageMetadata {...props.pageMetadata} />
      <SkipToContent />
      <AnnouncementBar />
      {!props.isHomePage && <Navbar />}

     
        
        {!props.isHomePage && (
           <div
        className={clsx(
          ThemeClassNames.wrapper.main,
          styles.wrapper,
          wrapperClassName,
        )}>
        {/* Book Header with Logo and Title (Home only) */}
      
        
            <div className={styles.contentLayout}>
              {/* Let Docusaurus render the default DocSidebar on the left and the doc content */}
              <main className={styles.mainContent}>
                <div
                  className={clsx(
                    ThemeClassNames.wrapper.main,
                    styles.mainWrapper,
                    pageClassName,
                  )}>
                  <ErrorBoundary fallback={(params) => <ErrorPageContent {...params} />}>
                    {children}
                  </ErrorBoundary>
                </div>
              </main>

              {/* Right Sidebar - Chat UI */}
              <aside className={styles.rightSidebar}>
                <ChatSidebar />
              </aside>
            </div>
            {!noFooter && <Footer />}
            </div>
          )}
          
          {/* Book Cover Page */}
          {props.isHomePage && (
            <>
         
            <BookCover 
              title={siteTitle}
              subtitle={tagline}
              bookStyle={styles.bookLogo}
              logoImage={bookConfig.logo}
              coverImage={bookConfig.coverImage}
              authors={bookConfig.authors}
              onStartReading={() => {
                // Navigate to the first chapter (Introduction)
                if (typeof window !== 'undefined') {
                  const target = '/chapters/intro';
                  // Prefer SPA navigation if available
                  try {
                    // Docusaurus injects a client-side router; fall back to full reload if not available
                    const nav = window?.docusaurus?.router?.history;
                    if (nav && typeof nav.push === 'function') {
                      nav.push(target);
                    } else {
                      window.location.assign(target);
                    }
                  } catch {
                    window.location.assign(target);
                  }
                }
              }}
            />
            </>
          )}

    </LayoutProvider>
  );
}

export default Layout;
