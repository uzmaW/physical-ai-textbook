/**
 * Enhanced Table of Contents Component for Docusaurus
 * Shows hierarchical book structure with progress tracking
 */

import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import Link from '@docusaurus/Link';
import { useDocusaurusContext } from '@docusaurus/core';
import clsx from 'clsx';
import { 
  ChevronRight, 
  ChevronDown, 
  BookOpen, 
  FileText, 
  Code, 
  CheckCircle, 
  Circle 
} from 'lucide-react';
import styles from './styles.module.css';

// Mock book structure - in real implementation, this would come from Docusaurus
const bookStructure = [
  {
    id: 'intro',
    title: 'Introduction',
    type: 'chapter',
    path: '/intro',
    completed: true,
  },
  {
    id: 'week-1',
    title: 'Week 1: Foundations of Robotics',
    type: 'section',
    expanded: true,
    children: [
      {
        id: 'week-1-theory',
        title: 'Theoretical Foundations',
        type: 'chapter',
        path: '/week-01/theory',
        completed: true,
      },
      {
        id: 'week-1-lab',
        title: 'Lab 1: ROS2 Setup',
        type: 'lab',
        path: '/week-01/lab',
        completed: false,
      },
    ],
  },
  {
    id: 'week-2',
    title: 'Week 2: Sensors and Perception',
    type: 'section',
    expanded: false,
    children: [
      {
        id: 'week-2-sensors',
        title: 'Sensor Technologies',
        type: 'chapter',
        path: '/week-02/sensors',
        completed: false,
      },
      {
        id: 'week-2-vision',
        title: 'Computer Vision Basics',
        type: 'chapter',
        path: '/week-02/vision',
        completed: false,
      },
      {
        id: 'week-2-lab',
        title: 'Lab 2: Camera Integration',
        type: 'lab',
        path: '/week-02/lab',
        completed: false,
      },
    ],
  },
  // Add more weeks as needed...
];

function TableOfContents() {
  const { siteConfig } = useDocusaurusContext();
  const location = useLocation();
  const [expandedSections, setExpandedSections] = useState(new Set());
  const [completedItems, setCompletedItems] = useState(new Set());

  // Initialize expanded sections and completed items
  useEffect(() => {
    const expanded = new Set();
    const completed = new Set();
    
    const processItems = (items) => {
      items.forEach(item => {
        if (item.expanded) {
          expanded.add(item.id);
        }
        if (item.completed) {
          completed.add(item.id);
        }
        if (item.children) {
          processItems(item.children);
        }
      });
    };
    
    processItems(bookStructure);
    setExpandedSections(expanded);
    setCompletedItems(completed);
  }, []);

  const toggleSection = (sectionId) => {
    setExpandedSections(prev => {
      const newSet = new Set(prev);
      if (newSet.has(sectionId)) {
        newSet.delete(sectionId);
      } else {
        newSet.add(sectionId);
      }
      return newSet;
    });
  };

  const toggleCompleted = (itemId) => {
    setCompletedItems(prev => {
      const newSet = new Set(prev);
      if (newSet.has(itemId)) {
        newSet.delete(itemId);
      } else {
        newSet.add(itemId);
      }
      // Save to localStorage
      localStorage.setItem('book-progress', JSON.stringify([...newSet]));
      return newSet;
    });
  };

  const getIcon = (type) => {
    switch (type) {
      case 'chapter':
        return <FileText size={16} />;
      case 'lab':
        return <Code size={16} />;
      case 'section':
        return <BookOpen size={16} />;
      default:
        return <Circle size={16} />;
    }
  };

  const renderItem = (item, level = 0) => {
    const isCurrentPage = location.pathname === item.path;
    const isExpanded = expandedSections.has(item.id);
    const isCompleted = completedItems.has(item.id);
    const hasChildren = item.children && item.children.length > 0;

    return (
      <div key={item.id} className={styles.tocItem}>
        <div 
          className={clsx(
            styles.tocItemContent,
            {
              [styles.currentPage]: isCurrentPage,
              [styles.completed]: isCompleted,
              [styles.section]: item.type === 'section',
              [styles.lab]: item.type === 'lab',
            }
          )}
          style={{ paddingLeft: `${level * 1.5}rem` }}
        >
          {/* Expand/Collapse Button for Sections */}
          {hasChildren && (
            <button
              className={styles.expandButton}
              onClick={() => toggleSection(item.id)}
              aria-label={isExpanded ? 'Collapse section' : 'Expand section'}
            >
              {isExpanded ? 
                <ChevronDown size={14} /> : 
                <ChevronRight size={14} />
              }
            </button>
          )}

          {/* Item Icon */}
          <span className={styles.itemIcon}>
            {getIcon(item.type)}
          </span>

          {/* Item Link/Title */}
          {item.path ? (
            <Link to={item.path} className={styles.itemLink}>
              {item.title}
            </Link>
          ) : (
            <span className={styles.itemTitle}>{item.title}</span>
          )}

          {/* Completion Toggle */}
          <button
            className={styles.completionButton}
            onClick={() => toggleCompleted(item.id)}
            aria-label={isCompleted ? 'Mark as incomplete' : 'Mark as complete'}
          >
            {isCompleted ? 
              <CheckCircle size={14} className={styles.completedIcon} /> : 
              <Circle size={14} className={styles.incompleteIcon} />
            }
          </button>
        </div>

        {/* Render Children */}
        {hasChildren && isExpanded && (
          <div className={styles.tocChildren}>
            {item.children.map(child => renderItem(child, level + 1))}
          </div>
        )}
      </div>
    );
  };

  // Calculate progress
  const totalItems = bookStructure.reduce((count, item) => {
    let itemCount = 1;
    if (item.children) {
      itemCount += item.children.length;
    }
    return count + itemCount;
  }, 0);
  
  const completedCount = completedItems.size;
  const progressPercent = Math.round((completedCount / totalItems) * 100);

  return (
    <div className={styles.tableOfContents}>
      {/* Header */}
      <div className={styles.tocHeader}>
        <h3 className={styles.tocTitle}>Table of Contents</h3>
        
        {/* Progress Bar */}
        <div className={styles.progressSection}>
          <div className={styles.progressInfo}>
            <span className={styles.progressText}>
              Progress: {completedCount}/{totalItems}
            </span>
            <span className={styles.progressPercent}>
              {progressPercent}%
            </span>
          </div>
          <div className={styles.progressBar}>
            <div 
              className={styles.progressFill}
              style={{ width: `${progressPercent}%` }}
            />
          </div>
        </div>
      </div>

      {/* Table of Contents List */}
      <nav className={styles.tocNav}>
        {bookStructure.map(item => renderItem(item))}
      </nav>

      {/* Quick Actions */}
      <div className={styles.tocFooter}>
        <button
          className={styles.actionButton}
          onClick={() => {
            // Expand all sections
            const allSectionIds = new Set();
            const collectSectionIds = (items) => {
              items.forEach(item => {
                if (item.children) {
                  allSectionIds.add(item.id);
                  collectSectionIds(item.children);
                }
              });
            };
            collectSectionIds(bookStructure);
            setExpandedSections(allSectionIds);
          }}
        >
          Expand All
        </button>
        
        <button
          className={styles.actionButton}
          onClick={() => setExpandedSections(new Set())}
        >
          Collapse All
        </button>
      </div>
    </div>
  );
}

export default TableOfContents;