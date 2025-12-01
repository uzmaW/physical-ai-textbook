import React from 'react';
import MDXComponents from '@theme-original/MDXComponents';
import { PersonalizedChapter } from '@site/src/components/PersonalizedChapter';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

// Wrapper component to add PersonalizedChapter controls
const ChapterWrapper = ({ children, ...props }: any) => {
  // Check if children already contains PersonalizedChapter to avoid double wrapping
  const hasPersonalizedChapter = React.Children.toArray(children).some(
    (child: any) => child?.type?.name === 'PersonalizedChapter'
  );

  // If already wrapped, just return children
  if (hasPersonalizedChapter) {
    return <>{children}</>;
  }

  // Get chapter ID from URL path
  const getChapterId = () => {
    if (typeof window !== 'undefined') {
      const path = window.location.pathname;
      return path.replace(/\//g, '-').substring(1) || 'chapter';
    }
    return 'chapter';
  };

  return (
    <PersonalizedChapter id={getChapterId()}>
      {children}
    </PersonalizedChapter>
  );
};

export default {
  ...MDXComponents,
  wrapper: ChapterWrapper,
  PersonalizedChapter,
  Tabs,
  TabItem,
};
