# Complete Setup Guide - Physical AI Textbook with ChatUI

This guide covers the complete setup of the Physical AI & Humanoid Robotics interactive textbook with AI-powered chat functionality.

## 🎯 What's Been Completed

### Frontend (Docusaurus)
- ✅ ChatUI components (RAGChatWidget, RAGChatbox)
- ✅ API service layer for backend communication
- ✅ Environment configuration system
- ✅ Theme integration (ChatUI in right sidebar)
- ✅ Build tested successfully

### Backend (FastAPI)
- ✅ RAG service with vector search and AI responses
- ✅ Translation service with caching
- ✅ Chat and translate API endpoints
- ✅ Complete indexing pipeline script
- ✅ Setup and run scripts
- ✅ Comprehensive documentation

## 🚀 Complete Setup (30 minutes)

### Step 1: Backend Setup (15 minutes)

```bash
# Navigate to backend directory
cd backend

# Run automated setup
./setup.sh

# This will:
# - Create Python virtual environment
# - Install all dependencies
# - Create .env file from template
```

**Edit .env file:**
```bash
nano .env  # or use your preferred editor
```

**Add these required values:**
```bash
# Required for ChatUI
OPENAI_API_KEY=sk-your-openai-key
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=  # Optional for local Qdrant

# Required for full features
DATABASE_URL=postgresql://user:pass@host:5432/dbname
FRONTEND_URL=http://localhost:3000
```

**Start Qdrant (local development):**
```bash
# Option A: Docker
docker run -p 6333:6333 qdrant/qdrant

# Option B: Use Qdrant Cloud
# Sign up at https://cloud.qdrant.io
# Copy URL and API key to .env
```

**Setup Qdrant collection:**
```bash
source venv/bin/activate
python scripts/setup_qdrant.py
```

**Index textbook content:**
```bash
python scripts/index_textbook.py --verbose
```

This will:
- Process all week-*.mdx files
- Create embeddings for each chunk
- Upload to Qdrant with metadata
- Show progress and statistics

**Start backend server:**
```bash
./run.sh

# Or manually:
# uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

Verify at http://localhost:8000/docs

### Step 2: Frontend Setup (5 minutes)

```bash
# Navigate to textbook directory
cd ../physical-ai-humanoid-textbook

# Install dependencies
npm install

# Environment is already configured in .env.local
# (Points to http://localhost:8000)

# Start development server
npm start
```

The textbook will open at http://localhost:3000

### Step 3: Test Integration (5 minutes)

1. **Open textbook** at http://localhost:3000
2. **Navigate to any chapter** (e.g., Week 01)
3. **Look at right sidebar** - You should see the AI Tutor chat
4. **Type a question** like "What is a humanoid robot?"
5. **Press Enter** - You should receive an AI response with citations!

### Step 4: Production Deployment (10 minutes)

**Backend:**
```bash
# Deploy to Railway/Fly.io/Render
# Set environment variables in platform
# Get backend URL (e.g., https://your-app.railway.app)
```

**Frontend:**
```bash
# Create .env.production
echo "REACT_APP_API_URL=https://your-backend-url.com" > .env.production

# Build
npm run build

# Deploy build/ directory to Vercel/Netlify/GitHub Pages
```

## 📁 Project Structure

```
humanoid_ai/
├── physical-ai-humanoid-textbook/   # Frontend (Docusaurus)
│   ├── src/
│   │   ├── components/
│   │   │   ├── RAGChatWidget.tsx    # Main chat UI ✨
│   │   │   └── RAGChatbox.tsx
│   │   ├── services/
│   │   │   └── apiService.ts        # API communication ✨
│   │   └── theme/
│   │       ├── TOC/index.tsx        # Swizzled for ChatUI
│   │       └── Root.tsx
│   ├── .env.local                   # Dev environment ✨
│   ├── .env.example                 # Template ✨
│   ├── CHATUI_SETUP.md             # Setup guide ✨
│   └── package.json
│
└── backend/                          # Backend (FastAPI)
    ├── app/
    │   ├── main.py                  # FastAPI app
    │   ├── config.py                # Configuration
    │   ├── routers/
    │   │   ├── chat.py              # Chat endpoint
    │   │   └── translate.py         # Translation endpoint
    │   └── services/
    │       └── rag.py               # RAG service
    ├── scripts/
    │   ├── setup_qdrant.py          # Qdrant setup
    │   └── index_textbook.py        # Indexing pipeline ✨
    ├── setup.sh                     # Setup script ✨
    ├── run.sh                       # Run script ✨
    ├── README.md                    # Backend docs ✨
    ├── requirements.txt             # Updated dependencies ✨
    └── .env.example

✨ = New or significantly updated files
```

## 🔧 Configuration Files

### Frontend `.env.local`
```bash
# Points to local backend
REACT_APP_API_URL=http://localhost:8000
```

### Backend `.env`
```bash
# Core services
OPENAI_API_KEY=sk-...
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=optional-for-local
DATABASE_URL=postgresql://...

# CORS
FRONTEND_URL=http://localhost:3000

# Optional
GOOGLE_TRANSLATE_API_KEY=...
```

## 🧪 Testing Checklist

### Backend Tests
- [ ] Health check: `curl http://localhost:8000/health`
- [ ] API docs: http://localhost:8000/docs
- [ ] Qdrant collection exists: Check `/docs` → Qdrant tab
- [ ] Chat endpoint works:
  ```bash
  curl -X POST http://localhost:8000/api/chat/ \
    -H "Content-Type: application/json" \
    -d '{"message":"What is a humanoid robot?","userLevel":"intermediate"}'
  ```

### Frontend Tests
- [ ] Textbook opens at http://localhost:3000
- [ ] ChatUI visible in right sidebar
- [ ] Can type message in chat input
- [ ] Sending message shows response
- [ ] Citations appear with responses
- [ ] Build succeeds: `npm run build`

### Integration Tests
- [ ] Frontend connects to backend
- [ ] Chat responses include relevant content
- [ ] Citations link to correct chapters
- [ ] Error messages are user-friendly
- [ ] Selected text feature works (highlight text, click "Ask about this")

## 📊 System Architecture

```
┌────────────────────────────────────────────────┐
│  User's Browser                                 │
│  ┌─────────────────────────────────────────┐   │
│  │  Docusaurus Textbook                    │   │
│  │  - Read chapters                        │   │
│  │  - Use ChatUI in sidebar                │   │
│  └──────────────┬──────────────────────────┘   │
│                 │ HTTP POST                     │
│                 │ /api/chat/                    │
└─────────────────┼────────────────────────────────┘
                  │
┌─────────────────▼────────────────────────────────┐
│  FastAPI Backend (Port 8000)                     │
│  ┌─────────────────────────────────────────────┐ │
│  │  Chat Router                                │ │
│  │  1. Receive user question                  │ │
│  │  2. Call RAG service                       │ │
│  └──────────────┬──────────────────────────────┘ │
│                 │                                 │
│  ┌──────────────▼──────────────────────────────┐ │
│  │  RAG Service                                │ │
│  │  1. Generate query embedding (OpenAI)      │ │
│  │  2. Search vectors (Qdrant)                │ │
│  │  3. Generate answer (GPT-4o-mini)          │ │
│  │  4. Return with citations                  │ │
│  └──────────────┬──────────────────────────────┘ │
└─────────────────┼────────────────────────────────┘
                  │
        ┌─────────┴─────────┐
        │                   │
┌───────▼────────┐  ┌──────▼────────┐
│  Qdrant        │  │  OpenAI       │
│  Vector Store  │  │  API          │
│  - Embeddings  │  │  - Embeddings │
│  - Metadata    │  │  - Chat       │
└────────────────┘  └───────────────┘
```

## 🎓 Usage Examples

### Basic Question
```typescript
// User types in ChatUI
"What is a humanoid robot?"

// Backend responds
{
  "answer": "A humanoid robot is a robot with a body shape built to resemble the human body...",
  "citations": [
    {
      "chapter": "Week 01 - Introduction",
      "url": "/chapters/week-01",
      "relevance": 0.95
    }
  ],
  "sourcesCount": 5,
  "model": "gpt-4o-mini"
}
```

### With Selected Text
```typescript
// User highlights: "Zero-Moment Point criterion"
// User clicks "Ask about this"
// ChatUI sends:
{
  "message": "Explain this selected text",
  "selectedText": "Zero-Moment Point criterion ensures...",
  "userLevel": "intermediate"
}

// Backend provides context-aware response
```

### Different Difficulty Levels
```typescript
// Beginner: Simple explanations with analogies
// Intermediate: Balanced technical detail
// Advanced: Concise, research-focused responses
```

## 🚨 Common Issues & Solutions

### "Backend API is not available"
**Cause:** Backend not running or wrong URL
**Solution:**
```bash
# Check backend is running
curl http://localhost:8000/health

# Restart Docusaurus to load .env changes
npm start
```

### No chat responses
**Cause:** Textbook not indexed in Qdrant
**Solution:**
```bash
cd backend
source venv/bin/activate
python scripts/index_textbook.py --force --verbose
```

### CORS errors
**Cause:** Frontend URL not in backend CORS config
**Solution:** Add to `backend/.env`:
```bash
FRONTEND_URL=http://localhost:3000
```

### OpenAI rate limit
**Cause:** Too many requests
**Solution:**
- Wait a few minutes
- Upgrade API plan
- Add caching in backend

### Qdrant connection refused
**Cause:** Qdrant not running
**Solution:**
```bash
docker run -p 6333:6333 qdrant/qdrant
```

## 💡 Pro Tips

1. **Development Workflow:**
   - Keep backend and frontend running in separate terminals
   - Use `--verbose` flag when indexing for debugging
   - Check browser console for frontend errors
   - Check terminal for backend errors

2. **Cost Optimization:**
   - Use `gpt-4o-mini` (already configured) - 80% cheaper than GPT-4
   - Implement response caching for common questions
   - Use Qdrant Cloud free tier (1GB)
   - PostgreSQL on Neon free tier is sufficient

3. **Performance:**
   - Index once, use many times
   - Reduce `top_k` in search if slow (default: 5)
   - Lower `max_tokens` for faster responses (default: 800)
   - Use CDN for static textbook content

4. **Production:**
   - Use environment variables for all secrets
   - Enable HTTPS everywhere
   - Implement rate limiting
   - Add monitoring (Sentry, LogRocket)
   - Set up CI/CD (GitHub Actions)

## 📚 Documentation Index

- **CHATUI_SETUP.md** - Frontend ChatUI setup and troubleshooting
- **backend/README.md** - Complete backend documentation
- **CHATUI_FIX_SUMMARY.md** - Technical details of the fix
- **This file** - Complete system setup guide

## ✅ Success Checklist

### Development Environment
- [ ] Backend dependencies installed
- [ ] Frontend dependencies installed
- [ ] Qdrant running and accessible
- [ ] Environment variables configured
- [ ] Textbook content indexed
- [ ] Backend server running on port 8000
- [ ] Frontend server running on port 3000
- [ ] ChatUI visible and functional
- [ ] Test query returns response
- [ ] Build completes successfully

### Production Deployment
- [ ] Backend deployed with environment variables
- [ ] Qdrant Cloud instance created
- [ ] Textbook indexed in production Qdrant
- [ ] Frontend built with production API URL
- [ ] Frontend deployed to hosting
- [ ] CORS configured correctly
- [ ] Health check endpoint accessible
- [ ] End-to-end test successful
- [ ] Error monitoring configured
- [ ] Documentation updated with URLs

## 🎉 Next Steps

1. **Enhance RAG:**
   - Add conversation history
   - Implement query refinement
   - Add response regeneration
   - Collect user feedback

2. **Improve UX:**
   - Add copy response button
   - Add export conversation
   - Add voice input
   - Add response ratings

3. **Add Features:**
   - Authentication system
   - User progress tracking
   - Personalized recommendations
   - Quiz integration

4. **Scale:**
   - Implement caching layer (Redis)
   - Add rate limiting
   - Set up load balancing
   - Add analytics

## 🤝 Support

For issues:
1. Check relevant documentation file
2. Review error messages carefully
3. Test each component separately
4. Check environment configuration
5. Verify API keys and connections

## 📝 License

Part of the Physical AI & Humanoid Robotics educational project.

---

**Status:** ✅ Complete setup guide with all components documented and tested.

**Ready to start?** Follow Step 1 above and you'll have a fully functional AI-powered textbook in 30 minutes! 🚀
