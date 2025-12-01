# Docusaurus Configuration for Deployment

## Overview

Your Docusaurus textbook is in `physical-ai-humanoid-textbook/` and uses environment variables to connect to the FastAPI backend.

---

## Local Development Setup

### 1. Install Dependencies

```bash
cd physical-ai-humanoid-textbook
npm install
```

### 2. Create `.env.local`

```bash
REACT_APP_API_URL=http://localhost:8000
REACT_APP_ENVIRONMENT=development
```

### 3. Start Development Server

```bash
npm start
```

Open: http://localhost:3000

---

## How Docusaurus Connects to Backend

### Environment Variable Flow

1. **At Build Time (Vercel)**:
   - Vercel reads `REACT_APP_API_URL` from project environment variables
   - Value is baked into the HTML/JS during build

2. **At Runtime (Browser)**:
   - JavaScript reads the API URL from `window.REACT_APP_API_URL`
   - Used by `src/services/apiService.ts`

### API Service Location

File: `physical-ai-humanoid-textbook/src/services/apiService.ts`

```typescript
const API_BASE_URL = typeof window !== 'undefined'
  ? (window as any).REACT_APP_API_URL || 'http://localhost:8000'
  : 'http://localhost:8000';
```

This:
- Checks if running in browser (`typeof window !== 'undefined'`)
- Gets API URL from window object
- Falls back to localhost:8000 if not set

---

## Docusaurus Config Updates

### 1. Update docusaurus.config.js for Production

Your current config has hardcoded URL:

```javascript
url: 'https://humanoid-ai-textbook.piaic.org',
baseUrl: '/',
```

**For Vercel deployment, update to:**

```javascript
// Dynamic URL based on environment
const isProduction = process.env.NODE_ENV === 'production';
const url = isProduction 
  ? 'https://physical-ai-textbook.vercel.app'
  : 'http://localhost:3000';

const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Embodied Intelligence in Practice',
  url: url,
  baseUrl: '/',
  // ... rest of config
```

### 2. Add Custom Head Tags

In `docusaurus.config.js`, add meta tags:

```javascript
headTags: [
  {
    tagName: 'meta',
    attributes: {
      name: 'description',
      content: 'Interactive textbook for Physical AI and Humanoid Robotics with AI-powered chat',
    },
  },
],
```

### 3. Configure SSR for Production

Add to `docusaurus.config.js`:

```javascript
staticDirs: ['static'],
noIndex: false,
customFields: {
  apiUrl: process.env.REACT_APP_API_URL || 'http://localhost:8000',
},
```

---

## Vercel Deployment Configuration

### vercel.json Setup

Your `physical-ai-humanoid-textbook/vercel.json` should have:

```json
{
  "buildCommand": "npm run build",
  "outputDirectory": "build",
  "env": {
    "REACT_APP_API_URL": "@api_url_production",
    "REACT_APP_ENVIRONMENT": "production"
  },
  "headers": [
    {
      "source": "/(.*)",
      "headers": [
        {
          "key": "Cache-Control",
          "value": "public, max-age=3600, must-revalidate"
        }
      ]
    }
  ]
}
```

### Build Settings in Vercel Dashboard

1. Go to Vercel Project Settings
2. **Build & Development Settings**:
   - **Framework Preset**: `Docusaurus`
   - **Build Command**: `npm run build`
   - **Output Directory**: `build`
   - **Install Command**: `npm install`

3. **Environment Variables**:
   ```
   REACT_APP_API_URL = https://physical-ai-backend.onrender.com
   REACT_APP_ENVIRONMENT = production
   ```

---

## Docusaurus Build Configuration

### package.json Scripts

Your `package.json` should have:

```json
{
  "scripts": {
    "docusaurus": "docusaurus",
    "start": "docusaurus start",
    "build": "docusaurus build",
    "serve": "docusaurus serve",
    "deploy": "docusaurus deploy",
    "clear": "docusaurus clear"
  }
}
```

### Optimize for Production

Add to `docusaurus.config.js`:

```javascript
// Disable development-only features in production
const isDevelopment = process.env.NODE_ENV !== 'production';

// In themeConfig:
themeConfig: {
  colorMode: {
    defaultMode: 'light',
    disableSwitch: false,
    respectPrefersColorScheme: true,
  },
  announcementBar: isDevelopment ? {
    id: 'dev-mode',
    content: '🚧 Development Mode - Using local backend',
    backgroundColor: '#fbbf24',
    textColor: '#000',
    isCloseable: false,
  } : undefined,
}
```

---

## API Integration Checklist

- [ ] `apiService.ts` reads `REACT_APP_API_URL` from window object
- [ ] Backend CORS allows `https://physical-ai-textbook.vercel.app`
- [ ] Environment variables set in Vercel dashboard
- [ ] Build command: `npm run build`
- [ ] Output directory: `build`
- [ ] `package.json` has all Docusaurus dependencies
- [ ] Test locally with: `npm start`

---

## Troubleshooting Docusaurus Deployment

### Build Fails on Vercel

**"Cannot find module X"**
- Check `package.json` dependencies
- Run `npm install` locally
- Verify `package-lock.json` is committed

**"Build timeout"**
- Docusaurus build can take 2-5 minutes
- Check Vercel logs for specific errors

### API Not Connecting

**CORS Error**
- Backend CORS must include: `https://physical-ai-textbook.vercel.app`
- Check `backend/app/main.py`:
  ```python
  allow_origins=[
    "https://physical-ai-textbook.vercel.app",
    # ... other origins
  ]
  ```

**Wrong API URL**
- Verify `REACT_APP_API_URL` in Vercel Environment Variables
- Should be: `https://physical-ai-backend.onrender.com`
- Check browser DevTools → Network tab for actual requests

**"Failed to fetch"**
- Backend might be offline (cold start on Render)
- Try again in 1-2 minutes
- Check if `/health` endpoint responds

### Static Assets Not Loading

- Ensure `build/` directory is correct
- Check `publicPath` in `docusaurus.config.js`
- Verify `static/` folder permissions

---

## Environment Variables Reference

### Development (Local)

File: `physical-ai-humanoid-textbook/.env.local`

```
REACT_APP_API_URL=http://localhost:8000
REACT_APP_ENVIRONMENT=development
```

### Production (Vercel)

Set in Vercel Project Settings → Environment Variables:

```
REACT_APP_API_URL=https://physical-ai-backend.onrender.com
REACT_APP_ENVIRONMENT=production
```

### How They're Used

- `REACT_APP_API_URL`: Backend endpoint for chat, auth, etc.
- `REACT_APP_ENVIRONMENT`: Logging level, analytics, features

---

## Performance Optimization

### Add to docusaurus.config.js

```javascript
// Cache buster for CSS/JS
ssrTemplate: `<!DOCTYPE html>
<html <%~ it.htmlAttributes %>>
  <head>
    <%~ it.headTags %>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta name="theme-color" content="#1890ff">
    <%~ it.styleTags %>
  </head>
  <body <%~ it.bodyAttributes %>>
    <%~ it.preBodyTags %>
    <div id="__docusaurus"><%~ it.appHtml %></div>
    <%~ it.postBodyTags %>
  </body>
</html>`,
```

### Image Optimization

For textbook images, use:
```markdown
![Humanoid Robot](./images/robot.png "width: 400px; height: auto;")
```

---

## Custom Domain (Optional)

### Add Custom Domain to Vercel

1. Go to Vercel Project Settings → Domains
2. Add domain: `textbook.yourdomain.com`
3. Update DNS records as instructed
4. Update backend CORS:
   ```python
   allow_origins=[
     "https://textbook.yourdomain.com",
     "https://physical-ai-textbook.vercel.app",
   ]
   ```

---

## Auto-Deployment

Push to GitHub auto-deploys:

```bash
git add .
git commit -m "Update textbook content"
git push origin main
```

Vercel will:
1. Detect push
2. Run `npm install`
3. Run `npm run build`
4. Deploy to `physical-ai-textbook.vercel.app`
5. Takes 2-5 minutes

---

## Useful Commands

```bash
# Build locally
npm run build

# Serve build output
npm run serve

# Preview production build
npm start

# Clear cache
npm run clear && npm run build

# Check for errors
npm run build 2>&1 | grep -i error
```

---

**Done!** Your Docusaurus textbook is configured for Vercel deployment. 🚀
