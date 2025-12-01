/**
 * Book Cover Component for Docusaurus
 * Displays cover image, title, authors and read button
 */

import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

function BookCover({
  title,
  subtitle,
  coverImage,
  authors = [],
  onStartReading,
  className
}) {
  return (
    <div className={clsx(styles.bookCover, className)}>
      <div className={styles.coverContent}>
        {coverImage && (
          <img 
            src={coverImage} 
            alt={`${title} Cover`}
            className={styles.coverImage}
          />
        )}
        
        <h1 className={styles.coverTitle}>
          {title}
        </h1>
        
        {subtitle && (
          <p className={styles.coverSubtitle}>
            {subtitle}
          </p>
        )}
        
        {authors.length > 0 && (
          <div className={styles.coverAuthors}>
            <span>by </span>
            {authors.map((author, index) => (
              <span key={index}>
                {author}
                {index < authors.length - 1 ? ', ' : ''}
              </span>
            ))}
          </div>
        )}
        
        <button
          className={styles.startReadingButton}
          onClick={onStartReading}
          type="button"
        >
          Start Reading →
        </button>
        
        <div className={styles.scrollIndicator}>
          <div className={styles.scrollArrow}>↓</div>
          <span>Scroll to explore</span>
        </div>
      </div>
      
      {/* Background Pattern */}
      <div className={styles.backgroundPattern} />
    </div>
  );
}

export default BookCover;