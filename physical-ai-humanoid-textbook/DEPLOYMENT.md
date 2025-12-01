# Deployment Guide

## Physical AI & Humanoid Robotics Textbook

This guide covers deploying the Docusaurus-based textbook to production.

---

## Prerequisites

- Node.js 18+ and npm
- Git
- (Optional) Docker for containerized deployment

---

## Local Development

### 1. Install Dependencies

```bash
cd /path/to/physical-ai-humanoid-textbook
npm install
```

### 2. Start Development Server

```bash
npm start
```

The textbook will be available at `http://localhost:3000`

### 3. Build for Production

```bash
npm run build
```

Output will be in `build/` directory.

---

## Deployment Options

### Option 1: GitHub Pages (Recommended)

```bash
# Configure docusaurus.config.js
# Set organizationName and projectName

# Deploy
GIT_USER=<your-username> npm run deploy
```

### Option 2: Vercel

1. Push code to GitHub
2. Go to [vercel.com](https://vercel.com)
3. Import repository
4. Vercel auto-detects Docusaurus
5. Click "Deploy"

### Option 3: Netlify

1. Push code to GitHub
2. Go to [netlify.com](https://netlify.com)
3. New site from Git
4. Build command: `npm run build`
5. Publish directory: `build`

### Option 4: Docker

```dockerfile
# Dockerfile
FROM node:18-alpine

WORKDIR /app

COPY package*.json ./
RUN npm ci --only=production

COPY . .
RUN npm run build

FROM nginx:alpine
COPY --from=0 /app/build /usr/share/nginx/html

EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]
```

```bash
# Build and run
docker build -t humanoid-textbook .
docker run -p 80:80 humanoid-textbook
```

---

## Backend API Deployment

The RAG chatbox requires a backend API.

### Prerequisites

- Neon Postgres database
- Qdrant vector database
- OpenAI API key

### Environment Variables

Create `.env`:

```env
# Database
POSTGRES_URL=postgresql://user:pass@neon.tech/dbname
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key

# OpenAI
OPENAI_API_KEY=sk-your-openai-key

# Server
PORT=3001
NODE_ENV=production
```

### Deploy Backend

```bash
cd backend/
npm install
npm run build
npm start
```

### Deploy to Railway/Fly.io

```bash
# Railway
railway up

# Fly.io
flyctl deploy
```

---

## Post-Deployment

### 1. Test Deployment

```bash
curl https://your-site.com
curl https://your-api.com/health
```

### 2. Configure Analytics (Optional)

Add to `docusaurus.config.js`:

```javascript
themeConfig: {
  // ...
  gtag: {
    trackingID: 'G-XXXXXXXXXX',
  },
},
```

### 3. Enable Search

Configure Algolia DocSearch or use local search plugin.

---

## Monitoring

- **Uptime**: UptimeRobot, Pingdom
- **Errors**: Sentry
- **Analytics**: Plausible, Google Analytics

---

## CI/CD Pipeline (GitHub Actions)

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy Textbook

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - run: npm ci
      - run: npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

---

## Troubleshooting

### Build Fails

```bash
# Clear cache
npm run clear

# Reinstall dependencies
rm -rf node_modules package-lock.json
npm install

# Rebuild
npm run build
```

### Broken Links

```bash
# Check for broken links
npm run build -- --no-minify
```

---

**For support**: humanoid-ai@piaic.org

