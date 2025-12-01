# URL Structure Guide - Physical AI Textbook

## ✅ Correct URL Structure

All documentation pages are accessible under the `/chapters/` path.

### Homepage
- **URL**: http://localhost:3000
- **Description**: Landing page with book cover and navigation
- **Features**:
  - "Start Reading" button → Goes to `/chapters/intro`
  - Quick links to Week 1 and Week 2

### Documentation Pages

All chapter pages follow this pattern: `http://localhost:3000/chapters/{page-id}`

#### Available Pages:

| Page | URL | Description |
|------|-----|-------------|
| Introduction | http://localhost:3000/chapters/intro | Book introduction |
| Week 1 | http://localhost:3000/chapters/week-01 | Foundations of Robotics |
| Week 2 | http://localhost:3000/chapters/week-02 | Sensors and Perception |
| Week 3 | http://localhost:3000/chapters/week-03 | Motion Planning |
| Week 4 | http://localhost:3000/chapters/week-04 | Manipulation |
| Week 5 | http://localhost:3000/chapters/week-05 | Navigation |
| Week 6 | http://localhost:3000/chapters/week-06 | Computer Vision |
| Week 7 | http://localhost:3000/chapters/week-07 | Machine Learning |
| Week 8 | http://localhost:3000/chapters/week-08 | Deep Learning |
| Week 9 | http://localhost:3000/chapters/week-09 | Reinforcement Learning |
| Week 10 | http://localhost:3000/chapters/week-10 | Sim-to-Real Transfer |
| Week 11 | http://localhost:3000/chapters/week-11 | Advanced Topics |
| Week 12 | http://localhost:3000/chapters/week-12 | Project Work |
| Week 13 | http://localhost:3000/chapters/week-13 | Final Project |
| Glossary | http://localhost:3000/chapters/glossary | Terms and definitions |
| Bibliography | http://localhost:3000/chapters/bibliography | References |

## ⚠️ Common Mistakes

### ❌ WRONG - Missing `/chapters/` prefix:
- ~~http://localhost:3000/intro~~ → **404 Not Found**
- ~~http://localhost:3000/week-01~~ → **404 Not Found**

### ✅ CORRECT - With `/chapters/` prefix:
- http://localhost:3000/chapters/intro → **200 OK**
- http://localhost:3000/chapters/week-01 → **200 OK**

## 🔧 Configuration

The URL structure is defined in `docusaurus.config.js`:

```javascript
docs: {
  routeBasePath: 'chapters',  // All docs under /chapters/
  sidebarPath: './sidebars.js',
}
```

### Why `/chapters/`?

Using `/chapters/` as the base path:
1. **Keeps homepage separate**: Cover page at `/` remains distinct
2. **Clear organization**: All chapter content is grouped under one path
3. **Consistent URLs**: Easy to remember pattern for all documentation

## 📱 Navigation

### From Homepage:
1. Click **"Start Reading →"** button → Goes to `/chapters/intro`
2. Click **Week 1** or **Week 2** quick links
3. Use browser navigation to any `/chapters/*` URL

### From Chapter Pages:
1. **Left Sidebar**: Enhanced Table of Contents with progress tracking
2. **Right Sidebar**: AI Chat Assistant (ChatSidebar)
3. **Navigation**: Previous/Next chapter links at bottom

## 🎯 Features on Chapter Pages:

When you visit any `/chapters/*` page, you'll see:

### Left Sidebar (TableOfContents):
- Hierarchical chapter structure
- Progress tracking checkboxes
- Expand/collapse sections
- Current page highlighting

### Right Sidebar (ChatSidebar):
- AI Assistant for questions
- Context-aware based on current chapter
- Message history
- Citation support
- Collapsible interface

### Main Content:
- Markdown rendered content
- Code syntax highlighting
- Math equations (KaTeX)
- Interactive elements

## 🚀 Quick Start

1. **Start here**: http://localhost:3000
2. **Read intro**: http://localhost:3000/chapters/intro
3. **Begin course**: http://localhost:3000/chapters/week-01

## ✅ Status Check

Test if everything is working:

```bash
# Homepage (should return 200)
curl -I http://localhost:3000

# Intro page (should return 200)
curl -I http://localhost:3000/chapters/intro

# Week 1 (should return 200)
curl -I http://localhost:3000/chapters/week-01
```

All should return `HTTP/1.1 200 OK`.

## 📝 Notes

- Development server runs on port 3000
- Backend API is on port 8000
- Hot reload enabled (changes reflect immediately)
- URLs are case-sensitive
- Trailing slashes are handled automatically by Docusaurus
