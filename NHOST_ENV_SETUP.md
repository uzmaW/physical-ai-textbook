# 🔧 Nhost Environment Variables Setup Guide

## 📊 Required Environment Variables for Nhost Dashboard

Go to your Nhost project dashboard → **Settings** → **Environment Variables** and add these:

### 🗄️ Database Configuration
```
DATABASE_URL
Value: your_neon_postgresql_connection_string
Example: postgresql://user:password@ep-xxxxx.neon.tech/humanoid_textbook
```

### 🤖 AI Services
```
OPENAI_API_KEY
Value: your_openai_api_key
Example: sk-your-openai-api-key

QDRANT_URL
Value: your_qdrant_cloud_url
Example: https://xxxxx.qdrant.io:6333

QDRANT_API_KEY
Value: your_qdrant_api_key
Example: your-qdrant-api-key
```

### 🔐 Authentication & Security
```
JWT_SECRET
Value: generate_secure_32_char_string
Generate with: openssl rand -hex 32
Example: a1b2c3d4e5f6789012345678901234567890abcdef1234567890abcdef123456

JWT_ALGORITHM
Value: HS256

JWT_EXPIRATION_DAYS
Value: 30

HASURA_ADMIN_SECRET
Value: generate_secure_random_string
Generate with: openssl rand -hex 32
Example: b2c3d4e5f6789012345678901234567890abcdef1234567890abcdef123456a1

HASURA_WEBHOOK_SECRET
Value: generate_secure_random_string
Generate with: openssl rand -hex 32
Example: c3d4e5f6789012345678901234567890abcdef1234567890abcdef123456a1b2
```

### 🌐 OAuth Providers (Optional)
```
AUTH_GITHUB_CLIENT_ID
Value: your_github_oauth_client_id
Example: Iv1.a1b2c3d4e5f67890

AUTH_GITHUB_CLIENT_SECRET
Value: your_github_oauth_client_secret
Example: a1b2c3d4e5f6789012345678901234567890abcd

AUTH_GOOGLE_CLIENT_ID
Value: your_google_oauth_client_id
Example: 123456789012-abcdefghijklmnop.apps.googleusercontent.com

AUTH_GOOGLE_CLIENT_SECRET
Value: your_google_oauth_client_secret
Example: GOCSPX-abcdefghijklmnopqrstuvwxyz
```

### 🌍 Translation Service (Optional)
```
GOOGLE_TRANSLATE_API_KEY
Value: your_google_translate_api_key
Example: AIzaSyABC123DEF456GHI789JKL012MNO345PQR
```

### 🎵 Voice Services (Optional)
```
VOICE_API_BASE_URL
Value: your_voice_service_url
Example: https://your-voice-api.com/v1
```

### 🌐 CORS & Frontend
```
FRONTEND_URL
Value: https://physical-ai-textbook.nhost.run
(This will be your Nhost app URL)

DEBUG
Value: false

API_VERSION
Value: 1.0.0
```

## 🚀 Quick Setup Commands

### 1. Generate Required Secrets
Run these commands to generate secure secrets:

```bash
# Generate JWT Secret
echo "JWT_SECRET=$(openssl rand -hex 32)"

# Generate Hasura Admin Secret
echo "HASURA_ADMIN_SECRET=$(openssl rand -hex 32)"

# Generate Hasura Webhook Secret  
echo "HASURA_WEBHOOK_SECRET=$(openssl rand -hex 32)"
```

### 2. Copy Your Existing Values
From your local `backend/.env` file, copy these values:
- DATABASE_URL
- OPENAI_API_KEY
- QDRANT_URL
- QDRANT_API_KEY
- GITHUB_CLIENT_ID (if using OAuth)
- GITHUB_CLIENT_SECRET (if using OAuth)

## 📋 Environment Variables Checklist

### ✅ Essential (Required for basic functionality)
- [ ] DATABASE_URL
- [ ] OPENAI_API_KEY
- [ ] QDRANT_URL
- [ ] QDRANT_API_KEY
- [ ] JWT_SECRET
- [ ] HASURA_ADMIN_SECRET
- [ ] HASURA_WEBHOOK_SECRET

### 🔄 OAuth (Optional - for user authentication)
- [ ] AUTH_GITHUB_CLIENT_ID
- [ ] AUTH_GITHUB_CLIENT_SECRET
- [ ] AUTH_GOOGLE_CLIENT_ID
- [ ] AUTH_GOOGLE_CLIENT_SECRET

### 🌍 Translation (Optional - for multi-language support)
- [ ] GOOGLE_TRANSLATE_API_KEY

### 🎵 Voice (Optional - for voice features)
- [ ] VOICE_API_BASE_URL

### ⚙️ Application Settings
- [ ] FRONTEND_URL
- [ ] DEBUG
- [ ] API_VERSION
- [ ] JWT_ALGORITHM
- [ ] JWT_EXPIRATION_DAYS

## 🎯 Step-by-Step Nhost Dashboard Setup

1. **Login to Nhost**: Go to [app.nhost.io](https://app.nhost.io)
2. **Select Your Project**: Choose your "physical-ai-textbook" project
3. **Navigate to Settings**: Click "Settings" in the sidebar
4. **Open Environment Variables**: Click "Environment Variables"
5. **Add Variables**: Click "Add Variable" for each one above
6. **Deploy**: Changes auto-deploy when you save

## ⚠️ Important Notes

- **Secrets are encrypted** in Nhost dashboard
- **Never commit secrets** to your repository
- **Test locally first** with the same values
- **Regenerate secrets** if compromised
- **Use strong passwords** for OAuth apps

## 🔍 Verification

After setting up, verify your deployment:
1. Check function logs in Nhost dashboard
2. Test API endpoints: `https://your-subdomain.nhost.run/v1/functions/fastapi/docs`
3. Verify database connection works
4. Test authentication flows (if using OAuth)

---

🎉 **Your environment is now configured for Nhost deployment!**
