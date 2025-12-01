# How ChatSidebar Loads in the Application

## Component Flow

```
Docusaurus Layout
    ↓
TOC Component (Right Sidebar)
    ↓
RAGChatWidget Component
    ↓
ChatUI Styles (ChatUI.module.css)
```

## File Structure

1. **`src/theme/TOC/index.tsx`**
   - Docusaurus uses this component for the right sidebar (Table of Contents position)
   - We've swizzled it to render `RAGChatWidget` instead of the default TOC
   - Wraps the widget in a sticky container with 100vh height

2. **`src/theme/ChatSidebar/index.js`**
   - Alternative entry point (not currently used)
   - Also renders `RAGChatWidget`

3. **`src/components/RAGChatWidget.tsx`**
   - Main chat component with all functionality
   - Handles messages, user input, API calls
   - Manages state (collapsed, expanded, history, settings)
   - Uses `ChatUI.module.css` for styling

4. **`src/components/ChatUI.module.css`**
   - All chat styling
   - Message bubbles, containers, animations
   - Responsive design

## Recent Fixes

### Width Issues Fixed
- Removed fixed widths (`480px !important`) from `.chatContainer`
- Changed to `width: 100%` to be responsive to parent container
- Added `overflow: hidden` to prevent horizontal scroll

### Message Truncation Fixed
- Increased `.messageBubble` max-width from 85% to 90%
- Added `overflow-wrap: break-word` and `word-break: break-word`
- Added `overflow-x: hidden` to `.messagesContainer`
- Added `width: 100%` and `min-width: 0` to `.messagesContainer`

## CSS Hierarchy

```css
.chatContainer (100% width, responsive)
  ├── .chatHeader (header with controls)
  ├── .messagesContainer (scrollable, no horizontal overflow)
  │   └── .message
  │       └── .messageBubble (90% max-width, word-wrap)
  └── .inputContainer (input area)
```

## How to Test

1. Clear cache: `./clear-cache.sh`
2. Start dev server: `npm start`
3. Check that:
   - Chat takes full width of right sidebar
   - Messages don't get truncated
   - Long text wraps properly
   - No horizontal scrolling in chat

## Customization

To adjust chat width, modify the grid in `src/css/custom.css`:

```css
.main-wrapper {
  grid-template-columns: 280px 1fr 480px;
  /* Change 480px to desired chat width */
}
```
