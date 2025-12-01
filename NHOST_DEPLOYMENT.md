# Nhost Deployment Guide

This guide helps you deploy your Physical AI Textbook to Nhost while keeping your existing FastAPI backend structure and Neon PostgreSQL database.

## 📁 Files Created

- `nhost.toml` - Main Nhost configuration
- `nhost/functions/fastapi/index.py` - FastAPI wrapper function
- `nhost/functions/fastapi/requirements.txt` - Python dependencies
- This deployment guide

## 🔧 Environment Variables

Set these secrets in your Nhost dashboard (https://app.nhost.io):

### Database & Core Services
```
DATABASE_URL=your_neon_postgresql_connection_string
JWT_SECRET=your_jwt_secret_key
REDIS_URL=your_redis_url (optional)
```

### AI Services
```
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
VOICE_API_BASE_URL=your_voice_api_url (optional)
```

### Hasura Configuration
```
HASURA_ADMIN_SECRET=generate_secure_random_string
HASURA_WEBHOOK_SECRET=generate_secure_random_string
```

### OAuth Authentication (Optional)
```
AUTH_GITHUB_CLIENT_ID=your_github_oauth_client_id
AUTH_GITHUB_CLIENT_SECRET=your_github_oauth_client_secret
AUTH_GOOGLE_CLIENT_ID=your_google_oauth_client_id
AUTH_GOOGLE_CLIENT_SECRET=your_google_oauth_client_secret
```

## 🚀 Deployment Steps

### 1. Connect Repository
1. Go to [Nhost Console](https://app.nhost.io)
2. Create a new project
3. Connect your GitHub repository
4. Select this repository

### 2. Configure Environment Variables
1. In Nhost dashboard, go to **Settings > Environment Variables**
2. Add all the variables listed above
3. Use the same values from your `backend/.env` file

### 3. Deploy
1. Push your code to the main branch
2. Nhost will automatically detect the `nhost.toml` file
3. It will build and deploy your FastAPI backend as a serverless function
4. Your app will be available at: `https://your-subdomain.nhost.run`

## 📡 API Endpoints

After deployment, your FastAPI backend will be accessible at:

```
Base URL: https://your-subdomain.nhost.run/v1/functions/fastapi
```

### Example Endpoints:
- Chat: `POST https://your-subdomain.nhost.run/v1/functions/fastapi/chat`
- Auth: `POST https://your-subdomain.nhost.run/v1/functions/fastapi/auth/login`
- Voice: `POST https://your-subdomain.nhost.run/v1/functions/fastapi/voice/synthesize`
- Translation: `POST https://your-subdomain.nhost.run/v1/functions/fastapi/translate`

## 🔄 Frontend Configuration

Update your frontend API base URL in `physical-ai-humanoid-textbook/src/services/apiService.ts`:

```typescript
const API_BASE_URL = process.env.NODE_ENV === 'production' 
  ? 'https://your-subdomain.nhost.run/v1/functions/fastapi'
  : 'http://localhost:8000';
```

## 🗄️ Database

- Uses your existing **Neon PostgreSQL** database
- No migration needed - same connection string
- All your existing data remains intact

## 📊 Monitoring

- View logs in Nhost Console > Functions
- Monitor performance and usage
- Set up alerts for errors

## 🔧 Local Development

Your local development remains unchanged:
```bash
cd backend
python -m uvicorn app.main:app --reload
```

## 🎯 Benefits of Nhost Deployment

- ✅ **Serverless Functions**: Auto-scaling FastAPI backend
- ✅ **Global CDN**: Fast worldwide performance  
- ✅ **Built-in Auth**: OAuth integration ready
- ✅ **Real-time Database**: Hasura GraphQL API
- ✅ **File Storage**: Built-in file management
- ✅ **Monitoring**: Built-in logging and metrics

## 🆘 Troubleshooting

### Function Timeout
If functions timeout, increase compute resources in `nhost.toml`:
```toml
[hasura.resources]
compute_units = 1.0
memory = 2048
```

### Environment Variables Not Loading
1. Verify all secrets are set in Nhost dashboard
2. Redeploy after adding new variables
3. Check function logs for missing variables

### Database Connection Issues
1. Verify DATABASE_URL is correct
2. Ensure Neon database allows connections
3. Check if IP allowlisting is needed

---

🎉 **Your Physical AI Textbook is now ready for Nhost deployment!**
