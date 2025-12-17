// @ts-check
// Docusaurus configuration for Physical AI & Humanoid Robotics Textbook
// Custom layout: Cover page → 3-column layout (sidebar, content, RAG chat)

import {themes as prismThemes} from 'prism-react-renderer';
import dotenv from 'dotenv';
import path from 'path';
import { fileURLToPath } from 'url';

// Establish __dirname in ESM context
const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// Load base and environment-specific .env files
dotenv.config({ path: path.resolve(__dirname, '.env') });
dotenv.config({
  path: path.resolve(
    __dirname,
    process.env.NODE_ENV === 'production' ? '.env.production' : '.env.local'
  ),
});

// Resolve API base URL
const RESOLVED_API_BASE_URL =
  process.env.REACT_APP_API_URL ||
  process.env.NEXT_PUBLIC_API_URL ||
  process.env.VITE_API_URL ||
  process.env.API_BASE_URL ||
  'http://localhost:8000';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Embodied Intelligence in Practice',
  favicon: 'img/favicon.ico',

  url: 'https://physical-ai-textbook-snowy.vercel.app',
  baseUrl: '/',

  organizationName: 'piaic',
  projectName: 'humanoid-ai-textbook',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: { label: 'English', direction: 'ltr', htmlLang: 'en-US' },
      ur: { label: 'اردو', direction: 'rtl', htmlLang: 'ur-PK' },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          routeBasePath: 'chapters',
          sidebarPath: './sidebars.js',
          remarkPlugins: [[require('remark-math'), {}]],
          rehypePlugins: [[require('rehype-katex'), {}]],
        },
        blog: false,

        // ✅ Keep custom CSS (Tailwind will merge with your theme)
        theme: {
          customCss: './src/css/custom.css',
        },
      },
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

  themeConfig: {
    navbar: {
      hideOnScroll: false,
      style: 'dark',
      items: [],
    },

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
  },

  plugins: [
    // ✅ Tailwind plugin stays unchanged
    async function tailwindPlugin() {
      return {
        name: 'docusaurus-tailwindcss',
        configurePostCss(postcssOptions) {
          postcssOptions.plugins.push(require('tailwindcss'));
          postcssOptions.plugins.push(require('autoprefixer'));
          return postcssOptions;
        },
      };
    },

    // ✅ Environment variable injection
    async function envDefinePlugin() {
      return {
        name: 'env-define-plugin',
        configureWebpack() {
          return {
            plugins: [
              new (require('webpack')).DefinePlugin({
                'process.env.REACT_APP_API_URL': JSON.stringify(process.env.REACT_APP_API_URL || ''),
                'process.env.NEXT_PUBLIC_API_URL': JSON.stringify(process.env.NEXT_PUBLIC_API_URL || ''),
                'process.env.VITE_API_URL': JSON.stringify(process.env.VITE_API_URL || ''),
                'process.env.API_BASE_URL': JSON.stringify(process.env.API_BASE_URL || ''),
                __API_BASE_URL__: JSON.stringify(RESOLVED_API_BASE_URL),
              }),
            ],
          };
        },
      };
    },
  ],

  customFields: {
    API_BASE_URL: RESOLVED_API_BASE_URL,
  },
};

export default config;

