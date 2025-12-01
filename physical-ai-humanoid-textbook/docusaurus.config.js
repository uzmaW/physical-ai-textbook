// @ts-check
// Docusaurus configuration for Physical AI & Humanoid Robotics Textbook
// Custom layout: Cover page → 3-column layout (sidebar, content, RAG chat)

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Embodied Intelligence in Practice',
  favicon: 'img/favicon.ico',

  url: 'https://humanoid-ai-textbook.piaic.org',
  baseUrl: '/',

  organizationName: 'piaic',
  projectName: 'humanoid-ai-textbook',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          routeBasePath: 'chapters',
          sidebarPath: './sidebars.js',
          remarkPlugins: [
            [require('remark-math'), {}],
          ],
          rehypePlugins: [
            [require('rehype-katex'), {}],
          ],
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
      type: 'text/css',
      integrity:
        'sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM',
      crossorigin: 'anonymous',
    },
    {
      href: 'https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;700&display=swap',
      type: 'text/css',
    },
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Disable default navbar
      navbar: {
        hideOnScroll: false,
        style: 'dark',
        items: [],
      },

      // Disable footer (will add copyright line in content)
      footer: undefined,

      docs: {
        sidebar: {
          hideable: false,
          autoCollapseCategories: false,
        },
      },

      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 5,
      },

      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },

      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'cpp', 'bash', 'yaml', 'json'],
      },
    }),

  plugins: [
    async function tailwindPlugin(context, options) {
      return {
        name: 'docusaurus-tailwindcss',
        configurePostCss(postcssOptions) {
          postcssOptions.plugins.push(require('tailwindcss'));
          postcssOptions.plugins.push(require('autoprefixer'));
          return postcssOptions;
        },
      };
    },
  ],

};

export default config;
