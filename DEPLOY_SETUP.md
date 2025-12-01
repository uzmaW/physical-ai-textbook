# Deployment Setup Guide

## Prerequisites

Before deploying, gather these values:

### 1. Qdrant Vector Database (Free)
Go to [cloud.qdrant.io](https://cloud.qdrant.io):
- Sign up (free account)
- Create cluster (takes 1-2 min)
- Copy:
  - `QDRANT_URL`: https://xxxxx.qdrant.io
  - `QDRANT_API_KEY`: ey...

### 2. OpenAI API Key
Go to [platform.openai.com/api-keys](https://platform.openai.com/api-keys):
- Create new API key
- Copy: `sk-...`

---

## Deployment Steps

### Step 1: Deploy Backend to Render

**Go to**: https://render.com

1. Click "New+" → "Web Service"
2. Select repo: `uzmaW/physical-ai-textbook`
3. Fill form:
   - **Name**: `physical-ai-backend`
   - **Environment**: Python
   - **Region**: Oregon
   - **Plan**: Free
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
   - **Root Directory**: `backend`

4. **Environment Variables** (scroll down):
   Click "Add Environment Variable" for each:
   ```
   OPENAI_API_KEY = sk-...
   QDRANT_URL = https://xxxxx.qdrant.io
   QDRANT_API_KEY = ey...
   FRONTEND_URL = (leave blank for now, update after Vercel deploy)
   ```

5. Click "Create Web Service"
6. Wait for deployment (2-3 min)
7. Get your backend URL: `https://physical-ai-backend.onrender.com`
   - Test it: https://physical-ai-backend.onrender.com/health

---

### Step 2: Deploy Frontend to Vercel

**Go to**: https://vercel.com

1. Click "Add New" → "Project"
2. Import repo: `uzmaW/physical-ai-textbook`
3. Fill form:
   - **Project Name**: `physical-ai-textbook`
   - **Framework Preset**: Docusaurus
   - **Root Directory**: `physical-ai-humanoid-textbook`

4. **Environment Variables**:
   Click "Add" for each:
   ```
   REACT_APP_API_URL = https://physical-ai-backend.onrender.com
   REACT_APP_ENVIRONMENT = production
   ```

5. Click "Deploy"
6. Wait for build (2-3 min)
7. Get your frontend URL: `https://physical-ai-textbook.vercel.app`
   - Open it in browser to verify

---

### Step 3: Update Backend CORS

Now that you have both URLs, update backend:

1. Go back to Render dashboard
2. Select `physical-ai-backend` service
3. Go to "Environment"
4. Update: `FRONTEND_URL = https://physical-ai-textbook.vercel.app`
5. Service auto-redeployments

---

## Test the Connection

1. Open frontend: https://physical-ai-textbook.vercel.app
2. Navigate to any chapter
3. Try chat:
   - Type question in ChatUI
   - Should see "Connecting..." then response
4. Check browser DevTools (F12):
   - Network tab: requests to `/api/chat` should show 200
   - Console: no CORS errors

---

## If Deployment Fails

### Render (Backend)

**Build error?**
- Go to Service → Logs
- Look for Python import errors
- Check `requirements.txt` compatibility

**Runtime error?**
- Check if `OPENAI_API_KEY` is set (not empty)
- Verify Qdrant cloud connectivity

### Vercel (Frontend)

**Build error?**
- Go to Project → Deployments → View logs
- Usually Node.js/npm issues
- Check `physical-ai-humanoid-textbook/package.json`

**Not connecting to backend?**
- Check `REACT_APP_API_URL` is correct
- Open DevTools Console → look for API errors
- Verify backend is running (test `/health` endpoint)

---

## Auto-Redeploy on Push

Both services auto-deploy when you push to GitHub:

```bash
git add .
git commit -m "Fix chat API"
git push origin main

# Both Render & Vercel auto-rebuild within 1-2 min
```

---

## Custom Domain (Optional)

### Vercel
1. Project Settings → Domains
2. Add custom domain (e.g., `textbook.yoursite.com`)
3. Follow DNS instructions

### Render
1. Service Settings → Custom Domains
2. Add domain
3. Update `FRONTEND_URL` in backend

---

## Production Checklist

- [ ] Backend deployed on Render
- [ ] Frontend deployed on Vercel
- [ ] Environment variables set in both
- [ ] CORS updated with frontend URL
- [ ] Chat API working end-to-end
- [ ] No errors in browser console
- [ ] Qdrant Cloud account created
- [ ] OpenAI API key added

---

## Monitoring

### Render
- Service Logs: https://render.com/docs/observability
- Health check: `/health` endpoint

### Vercel
- Analytics: https://vercel.com/docs/analytics
- Error tracking: Console tab

---

**Deployment Complete!** 🚀

Your app is live at:
- Frontend: https://physical-ai-textbook.vercel.app
- Backend API: https://physical-ai-backend.onrender.com
- API Docs: https://physical-ai-backend.onrender.com/docs
