import { DocSection } from './types';

export const APP_NAME = "DocuChat";

export const MOCK_DOCS: DocSection[] = [
  {
    title: "Getting Started",
    pages: [
      {
        id: "intro",
        title: "Introduction",
        content: `
# Introduction

Welcome to **DocuChat**. This is a demonstration of a Docusaurus-like layout powered by React and Tailwind CSS, featuring an intelligent right-sidebar assistant.

## Features

- **Responsive Layout**: Adapts to mobile, tablet, and desktop.
- **AI Integration**: Chat with the documentation using the panel on the right.
- **Modern Styling**: Built with Tailwind CSS for a clean, elegant aesthetic.

### Why this layout?

Documentation users often need context. By placing the chat interface directly alongside the content, users can ask questions about what they are reading without switching tabs.
        `
      },
      {
        id: "install",
        title: "Installation",
        content: `
# Installation

To get started with this project, you'll need a modern Node.js environment.

\`\`\`bash
npm install docuchat-sdk
\`\`\`

## Configuration

Create a \`config.json\` file in your root directory:

\`\`\`json
{
  "apiKey": "YOUR_GEMINI_API_KEY",
  "theme": "light"
}
\`\`\`
        `
      }
    ]
  },
  {
    title: "Core Concepts",
    pages: [
      {
        id: "architecture",
        title: "Architecture",
        content: `
# Architecture

DocuChat follows a **tri-pane architecture**:

1.  **Navigation (Left)**: Allows users to traverse the hierarchy of information.
2.  **Content (Center)**: The primary focus area, rendering Markdown.
3.  **Assistant (Right)**: An ever-present help desk powered by Gemini.

## Data Flow

Data flows unidirectionally from the top-level App component down to the layout regions. Chat state is lifted to the top to persist conversation history as you navigate different pages.
        `
      },
      {
        id: "components",
        title: "Components",
        content: `
# Components

The UI is built using functional React components.

- \`<SidebarLeft />\`: Handles the navigation tree.
- \`<MainContent />\`: Renders markdown.
- \`<ChatSidebar />\`: Manages the interaction loop with the LLM.

## Styling

We use utility classes for rapid development and consistent theming.
        `
      }
    ]
  },
  {
    title: "API Reference",
    pages: [
      {
        id: "endpoints",
        title: "Endpoints",
        content: `
# API Endpoints

## GET /v1/chat

Stream a chat response.

**Parameters:**

- \`message\`: String. The user query.
- \`context\`: String. The current page content.

## POST /v1/search

Perform a semantic search across the documentation.
        `
      }
    ]
  }
];
