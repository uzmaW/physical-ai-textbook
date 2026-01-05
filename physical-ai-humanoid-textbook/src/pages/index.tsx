/**
 * Cover page for Physical AI & Humanoid Robotics Textbook
 * First page users see when accessing the textbook
 */

import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Open-Access University Textbook on Physical AI and Humanoid Robotics"
      noFooter={true}
      isHomePage={true}
    >
      <div className="cover-page">
        {/* Book Logo */}
        <div className="cover-logo">
          <svg
            viewBox="0 0 200 200"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
            className="w-full h-full"
          >
            {/* Robot head outline */}
            <rect
              x="50"
              y="40"
              width="100"
              height="120"
              rx="10"
              fill="white"
              fillOpacity="0.1"
              stroke="white"
              strokeWidth="3"
            />

            {/* Eyes */}
            <circle cx="75" cy="80" r="8" fill="#60A5FA" />
            <circle cx="125" cy="80" r="8" fill="#60A5FA" />

            {/* Neural network connections */}
            <g stroke="white" strokeWidth="2" opacity="0.6">
              <line x1="100" y1="110" x2="70" y2="130" />
              <line x1="100" y1="110" x2="130" y2="130" />
              <line x1="100" y1="110" x2="100" y2="140" />
            </g>

            {/* AI nodes */}
            <circle cx="70" cy="130" r="5" fill="#60A5FA" />
            <circle cx="100" cy="140" r="5" fill="#60A5FA" />
            <circle cx="130" cy="130" r="5" fill="#60A5FA" />

            {/* Book pages at bottom */}
            <path
              d="M 60 170 L 60 180 L 140 180 L 140 170"
              stroke="white"
              strokeWidth="2"
              fill="none"
            />
            <line x1="100" y1="170" x2="100" y2="180" stroke="white" strokeWidth="2" />
          </svg>
        </div>

        {/* Title */}
        <h1 className="cover-title">
          Physical AI &<br />
          Humanoid Robotics
        </h1>

        {/* Subtitle */}
        <p className="cover-subtitle">
          Embodied Intelligence in Practice
        </p>

        {/* Meta information */}
        <div className="cover-meta">
          <p className="mb-2">13 Weeks • ROS 2 • NVIDIA Isaac</p>
        </div>

        {/* CTA Button */}
        <Link className="cover-cta" to="/chapters/intro">
          Start Reading →
        </Link>

        {/* Alternative quick links */}
        <div className="mt-8 flex gap-4 text-sm">
          <Link to="/chapters/week-01" className="text-blue-400 hover:text-blue-300">
            Week 1
          </Link>
          <Link to="/chapters/week-02" className="text-blue-400 hover:text-blue-300">
            Week 2
          </Link>
        </div>

        {/* Copyright */}
        <div className="mt-16 text-sm opacity-75">
          <p>© 2025 PIAIC • Open Access CC BY-SA 4.0</p>
        </div>
      </div>
    </Layout>
  );
}
