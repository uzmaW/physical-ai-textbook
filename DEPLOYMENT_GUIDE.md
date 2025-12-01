# Deployment Guide: Render + Vercel

## Overview
- **Backend**: FastAPI on Render.com
- **Frontend**: Docusaurus on Vercel
- **Vector DB**: Qdrant Cloud (free tier)

---

## Step 1: Prepare GitHub Repository

```bash
# Make sure you have both in one repo
git add .
git commit -m "Add deployment configs"
git push origin main
```

---

## Step 2: Deploy Backend to Render

### Create Render Account
1. Go to [render.com](https://render.com)
2. Sign up (free)
3. Connect GitHub account

### Deploy Backend Service
1. Click "New+" → "Web Service"
2. Select your GitHub repository
3. Configure:
   - **Name**: `physical-ai-backend`
   - **Environment**: `Python`
   - **Region**: `Oregon` (or nearest)
   - **Plan**: `Free`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

### Add Environment Variables in Render
Go to Service Settings → Environment:

```
OPENAI_API_KEY=sk-...
QDRANT_URL=https://your-qdrant-url
QDRANT_API_KEY=...
DATABASE_URL=...
FRONTEND_URL=https://physical-ai.vercel.app
```

### Get Backend URL
After deploy completes, you'll get:
```
https://physical-ai-backend.onrender.com
```

---

## Step 3: Setup Qdrant Vector DB (Free)

### Option A: Qdrant Cloud (Recommended)
1. Go to [cloud.qdrant.io](https://cloud.qdrant.io)
2. Sign up (free tier: 1GB storage)
3. Create cluster
4. Get API key and URL
5. Add to Render environment variables

### Option B: Self-hosted (Advanced)
Skip if using Qdrant Cloud

---

## Step 4: Deploy Frontend to Vercel

### Create Vercel Account
1. Go to [vercel.com](https://vercel.com)
2. Sign up (free)
3. Import GitHub project

### Deploy Frontend
1. Click "New Project"
2. Select the repository
3. Configure:
   - **Framework**: `Docusaurus`
   - **Root Directory**: `./physical-ai-humanoid-textbook`
   - **Build Command**: `npm run build`
   - **Output Directory**: `build`

### Add Environment Variables
In Vercel Project Settings → Environment Variables:

```
REACT_APP_API_URL=https://physical-ai-backend.onrender.com
REACT_APP_ENVIRONMENT=production
```

### Deploy
Click "Deploy" and wait for build to complete.

You'll get:
```
https://physical-ai.vercel.app (or custom domain)
```

---

## Step 5: Update CORS Settings

In `backend/app/main.py`, update:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://physical-ai.vercel.app",  # Your Vercel URL
        "https://your-custom-domain.com",  # If you add custom domain
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

Then push to GitHub (auto-redeploy).

---

## Step 6: Update Frontend API Config

In `physical-ai-humanoid-textbook/src/services/api.ts` or equivalent:

```typescript
const API_BASE_URL = process.env.REACT_APP_API_URL || 
                      'http://localhost:8000';
```

---

## Step 7: Verify Deployment

### Test Backend Health
```bash
curl https://physical-ai-backend.onrender.com/health
```

Should return:
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "services": { ... }
}
```

### Test Frontend
Open in browser:
```
https://physical-ai.vercel.app
```

### Test Chat API
From frontend, try sending a chat message. It should:
1. Send request to backend
2. Get response from OpenAI (if API key configured)
3. Display in ChatUI

---

## Step 8: Add Custom Domain (Optional)

### Vercel
1. Project Settings → Domains
2. Add custom domain
3. Follow DNS instructions

### Render
1. Service Settings → Custom Domain
2. Add domain
3. Update CORS in backend

---

## Troubleshooting

### Backend not responding
- Check Render build logs: Service → Logs
- Verify environment variables are set
- Check `uvicorn` is installed: `pip freeze | grep uvicorn`

### CORS errors
- Verify `FRONTEND_URL` matches Vercel domain
- Restart Render service

### Chat not working
- Check browser DevTools → Network tab
- Verify API URL is correct
- Check OpenAI API key is set

### Free tier limitations
- Render free web service: spins down after 15 mins of inactivity (cold start ~30s)
- Vercel: limited to 100,000 function invocations/month
- Qdrant Cloud: 1GB storage, 10k requests/month

---

## Cost Estimate

| Service | Free Tier | Paid |
|---------|-----------|------|
| Render Backend | Full (with cold start) | $7/month |
| Vercel Frontend | Full | $20/month |
| Qdrant Cloud | 1GB + 10k req/mo | $10/month |
| **Total** | **Free** | **~$37/month** |

---

## Next Steps

1. Create GitHub token for CI/CD
2. Set up monitoring alerts
3. Enable auto-redeploy on push
4. Add custom domain for production

## Support

- Render Docs: [render.com/docs](https://render.com/docs)
- Vercel Docs: [vercel.com/docs](https://vercel.com/docs)
- Qdrant Docs: [qdrant.tech/documentation](https://qdrant.tech/documentation)
