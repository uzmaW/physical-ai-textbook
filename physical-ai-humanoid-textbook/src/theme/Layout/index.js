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
import {useDocusaurusContext} from '@docusaurus/core';
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

      <div
        className={clsx(
          ThemeClassNames.wrapper.main,
          styles.wrapper,
          wrapperClassName,
        )}>
        
        {/* Book Header with Logo and Title */}
        <header className={styles.bookHeader}>
          <div className={styles.headerContent}>
            {bookConfig.logo && (
              <img 
                src={bookConfig.logo} 
                alt={`${siteTitle} Logo`}
                className={styles.bookLogo}
              />
            )}
            <div className={styles.bookTitleSection}>
              <h1 className={styles.bookTitle}>{siteTitle}</h1>
              {tagline && <p className={styles.bookSubtitle}>{tagline}</p>}
            </div>
          </div>
        </header>

        {/* Main Book Layout */}
        <div className={styles.bookContainer}>
          
          {/* Book Cover Page */}
          {props.isHomePage && (
            <BookCover 
              title={siteTitle}
              subtitle={tagline}
              coverImage={bookConfig.coverImage}
              authors={bookConfig.authors}
              onStartReading={() => {
                // Scroll to content or navigate to first chapter
                window.scrollTo({ 
                  top: window.innerHeight, 
                  behavior: 'smooth' 
                });
              }}
            />
          )}

          {/* Three-Column Layout for Content Pages */}
          {!props.isHomePage && (
            <div className={styles.contentLayout}>
              
              {/* Left Sidebar - Table of Contents */}
              <aside className={styles.leftSidebar}>
                <div className={styles.tocContainer}>
                  <TableOfContents />
                </div>
              </aside>

              {/* Main Content */}
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
          )}
        </div>

        {!noFooter && <Footer />}
      </div>
    </LayoutProvider>
  );
}

export default Layout;
