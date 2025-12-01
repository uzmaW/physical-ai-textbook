// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive University Course on Modern Robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-domain.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',

  // GitHub pages deployment config
  organizationName: 'your-org',
  projectName: 'robotics-book',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Internationalization config
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'], // English and Urdu support
  },

  // Custom fields for book configuration
  customFields: {
    bookConfig: {
      logo: '/img/robotics-logo.png',
      coverImage: '/img/book-cover.jpg',
      authors: [
        'Dr. Ahmed Hassan',
        'Prof. Sarah Chen',
        'Dr. Muhammad Ali'
      ],
      isbn: '978-0-123456-78-9',
      publisher: 'Robotics Education Press',
      publishYear: '2024',
      version: '1.0',
      // Chat API configuration
      chatApiUrl: process.env.CHAT_API_URL || 'http://localhost:8000',
      // Course configuration
      totalWeeks: 13,
      difficulty: 'University Level',
      prerequisites: [
        'Linear Algebra',
        'Calculus',
        'Basic Programming (Python)',
        'Physics (Mechanics)'
      ]
    }
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/your-org/robotics-book/tree/main/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
          // Enable math equations
          remarkPlugins: [require('remark-math')],
          rehypePlugins: [require('rehype-katex')],
        },
        blog: false, // Disable blog for book-focused site
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
        sitemap: {
          changefreq: 'weekly',
          priority: 0.5,
        },
      }),
    ],
  ],

  // Stylesheets for math equations
  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
      type: 'text/css',
      integrity:
        'sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM',
      crossorigin: 'anonymous',
    },
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/robotics-social-card.jpg',
      
      navbar: {
        title: 'Physical AI & Robotics',
        logo: {
          alt: 'Robotics Course Logo',
          src: 'img/robotics-logo.png',
          srcDark: 'img/robotics-logo-dark.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Course Content',
          },
          {
            href: '/docs/labs',
            label: 'Labs',
            position: 'left',
          },
          {
            href: '/docs/resources',
            label: 'Resources',
            position: 'left',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
          {
            href: 'https://github.com/your-org/robotics-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Course',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'Week 1',
                to: '/docs/week-01',
              },
              {
                label: 'Labs',
                to: '/docs/labs',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS2 Documentation',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'NVIDIA Isaac Sim',
                href: 'https://developer.nvidia.com/isaac-sim',
              },
              {
                label: 'Course Repository',
                href: 'https://github.com/your-org/robotics-book',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Discord',
                href: 'https://discord.gg/robotics-course',
              },
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/ros2',
              },
              {
                label: 'Reddit',
                href: 'https://reddit.com/r/robotics',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course. Built with Docusaurus.`,
      },
      
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'cpp', 'bash', 'yaml', 'xml'],
      },
      
      // Enhanced search
      algolia: {
        appId: 'YOUR_APP_ID',
        apiKey: 'YOUR_SEARCH_API_KEY',
        indexName: 'robotics-book',
        contextualSearch: true,
        searchParameters: {},
        searchPagePath: 'search',
      },
      
      // Color mode config
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      
      // Announcement bar for important updates
      announcementBar: {
        id: 'course_announcement',
        content:
          '🤖 New: AI-Powered Chat Assistant is now available! Ask questions about any chapter content.',
        backgroundColor: '#3b82f6',
        textColor: '#ffffff',
        isCloseable: true,
      },
      
      // Metadata
      metadata: [
        {name: 'keywords', content: 'robotics, ai, machine learning, ros2, education, university'},
        {name: 'author', content: 'Robotics Education Team'},
        {property: 'og:type', content: 'book'},
      ],
    }),

  // Plugins for enhanced functionality
  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'labs',
        path: 'labs',
        routeBasePath: 'labs',
        sidebarPath: require.resolve('./sidebars-labs.js'),
        editUrl: 'https://github.com/your-org/robotics-book/tree/main/labs/',
      },
    ],
    [
      '@docusaurus/plugin-pwa',
      {
        debug: true,
        offlineModeActivationStrategies: [
          'appInstalled',
          'standalone',
          'queryString',
        ],
        pwaHead: [
          {
            tagName: 'link',
            rel: 'icon',
            href: '/img/robotics-logo.png',
          },
          {
            tagName: 'link',
            rel: 'manifest',
            href: '/manifest.json',
          },
          {
            tagName: 'meta',
            name: 'theme-color',
            content: '#3b82f6',
          },
        ],
      },
    ],
    // Custom plugin for chat integration
    './plugins/chat-plugin.js',
  ],
  
  // Scripts for enhanced functionality
  scripts: [
    {
      src: '/js/chat-integration.js',
      async: true,
    },
  ],
  
  // Client modules for custom functionality
  clientModules: [
    require.resolve('./src/modules/chat-module.js'),
  ],
};

module.exports = config;