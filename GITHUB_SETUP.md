# GitHub Setup Instructions

## Step 1: Create New Repository on GitHub

1. Go to [github.com/new](https://github.com/new)
2. Create repository:
   - **Repository name**: `physical-ai-textbook` (or your choice)
   - **Description**: `Physical AI & Humanoid Robotics: Interactive Textbook with RAG & Chat`
   - **Visibility**: Public (for open-source)
   - **Initialize repository**: ❌ NO (we'll push existing code)
3. Click "Create repository"

You'll get a URL like: `https://github.com/YOUR_USERNAME/physical-ai-textbook.git`

---

## Step 2: Initialize Git Locally

Run these commands in the project root:

```bash
cd /mnt/workingdir/piaic_projects/humanoid_ai

# Initialize git
git init

# Add all files
git add .

# Initial commit
git commit -m "Initial commit: Physical AI Textbook + FastAPI Backend"

# Add remote (replace with your URL from Step 1)
git remote add origin https://github.com/YOUR_USERNAME/physical-ai-textbook.git

# Push to GitHub
git branch -M main
git push -u origin main
```

---

## Step 3: Verify on GitHub

1. Refresh your GitHub repo page
2. You should see:
   - ✅ `backend/` folder
   - ✅ `physical-ai-humanoid-textbook/` folder
   - ✅ All config files (render.yaml, vercel.json, etc.)
   - ✅ DEPLOYMENT_GUIDE.md

---

## Step 4: Connect to Render & Vercel

### Render
1. Go to [render.com](https://render.com)
2. Click "New+" → "Web Service"
3. **Connect to GitHub**
4. Select: `YOUR_USERNAME/physical-ai-textbook`
5. Configure with `backend/render.yaml`

### Vercel
1. Go to [vercel.com](https://vercel.com)
2. Click "New Project"
3. **Import Git Repository**
4. Select: `YOUR_USERNAME/physical-ai-textbook`
5. Root Directory: `./physical-ai-humanoid-textbook`
6. Click "Deploy"

---

## Step 5: Set Environment Variables

### Render (Backend)
Service Settings → Environment Variables:
```
OPENAI_API_KEY=sk-...
QDRANT_URL=https://...
QDRANT_API_KEY=...
FRONTEND_URL=https://your-vercel-domain.vercel.app
```

### Vercel (Frontend)
Project Settings → Environment Variables:
```
REACT_APP_API_URL=https://your-render-service.onrender.com
REACT_APP_ENVIRONMENT=production
```

---

## Step 6: Auto-Deploy on Push

Both services will auto-deploy when you push to `main`:

```bash
# Make changes locally
git add .
git commit -m "Your commit message"
git push origin main

# Changes automatically deploy to:
# - Backend: https://your-backend.onrender.com
# - Frontend: https://your-domain.vercel.app
```

---

## Repository Structure

```
physical-ai-textbook/
├── backend/
│   ├── app/
│   │   ├── main.py
│   │   ├── routers/
│   │   ├── services/
│   │   └── models/
│   ├── requirements.txt
│   ├── render.yaml
│   ├── Procfile
│   └── README.md
│
├── physical-ai-humanoid-textbook/
│   ├── docs/
│   ├── src/
│   ├── package.json
│   ├── docusaurus.config.js
│   ├── vercel.json
│   └── README.md
│
├── DEPLOYMENT_GUIDE.md
├── GITHUB_SETUP.md
├── .gitignore
└── README.md
```

---

## Useful Git Commands

```bash
# Check status
git status

# View commit history
git log --oneline

# See what will be deployed
git diff origin/main

# View remote
git remote -v

# Update from remote
git pull origin main
```

---

## Troubleshooting

**"Permission denied" when pushing**
- Generate SSH key: `ssh-keygen -t ed25519`
- Add to GitHub: Settings → SSH and GPG keys
- Use SSH URL: `git@github.com:USERNAME/physical-ai-textbook.git`

**Large files rejected**
- Remove PDFs before committing
- Update `.gitignore`

**Build fails on Render/Vercel**
- Check build logs in service dashboard
- Verify environment variables are set
- Ensure Python 3.11+ and Node 18+

---

## Next: Custom Domain (Optional)

Once deployment is working:

1. **Vercel**: Add custom domain in Project Settings
2. **Render**: Add custom domain in Service Settings
3. Point DNS records to services
4. Update CORS in backend for new domain

---

Done! Your app will auto-deploy on every push to GitHub. 🚀
