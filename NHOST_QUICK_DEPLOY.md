# 🚀 Quick Nhost Deployment

## ✅ Files Created
- `nhost.toml` - Nhost configuration  
- `nhost/functions/fastapi/` - FastAPI wrapper
- `NHOST_DEPLOYMENT.md` - Complete deployment guide
- `physical-ai-humanoid-textbook/.env.nhost` - Frontend config

## ⚡ Quick Start (5 minutes)

### 1. Connect to Nhost
1. Go to [app.nhost.io](https://app.nhost.io)
2. Click "New Project" 
3. Connect your GitHub repository
4. Choose "physical-ai-textbook" as subdomain

### 2. Set Environment Variables
Copy these from your `backend/.env.example`:
```bash
DATABASE_URL=your_neon_postgresql_url
OPENAI_API_KEY=your_openai_key
QDRANT_URL=your_qdrant_url  
QDRANT_API_KEY=your_qdrant_key
JWT_SECRET=your_jwt_secret
HASURA_ADMIN_SECRET=generate_random_string
HASURA_WEBHOOK_SECRET=generate_random_string
```

### 3. Deploy
```bash
git add .
git commit -m "Add Nhost deployment configuration"
git push origin main
```

### 4. Access Your App
- **Frontend**: `https://physical-ai-textbook.nhost.run`
- **API**: `https://physical-ai-textbook.nhost.run/v1/functions/fastapi`
- **Hasura Console**: `https://physical-ai-textbook.nhost.run/console`

## 🔧 Update Frontend (Optional)
To use Nhost in production, update `apiService.ts`:
```typescript
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://physical-ai-textbook.nhost.run/v1/functions/fastapi'  
  : 'http://localhost:8000';
```

---
**That's it! Your app will be live in ~5 minutes** 🎉
