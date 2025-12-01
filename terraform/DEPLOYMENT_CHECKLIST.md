# Render Deployment Checklist

## Step 1: Create Web Service on Render Dashboard

1. Go to render.com
2. Click "New+" → "Web Service"
3. Select repo: uzmaW/physical-ai-textbook
4. Fill form:
   - Name: physical-ai-backend
   - Environment: Python
   - Region: Oregon
   - Plan: Free
   - Root Directory: backend
   - Build: pip install -r requirements.txt
   - Start: uvicorn app.main:app --host 0.0.0.0 --port \$PORT

## Step 2: Set Environment Variables

In Render Service Settings → Environment:

- OPENAI_API_KEY = sk-your-openai-key-here
- QDRANT_URL = https://xxxxx.qdrant.io
- QDRANT_API_KEY = ey-your-qdrant-key-here
- FRONTEND_URL = https://physical-ai-textbook.vercel.app

## Step 3: Deploy

Click "Create Web Service" and wait 3-5 minutes.

## Step 4: Verify

Test health endpoint:
\`\`\`bash
curl https://physical-ai-backend.onrender.com/health
\`\`\`

Should return: { "status": "healthy", ... }

## Step 5: Update Frontend

Add backend URL to Vercel:
REACT_APP_API_URL = https://physical-ai-backend.onrender.com

Done! 🚀
