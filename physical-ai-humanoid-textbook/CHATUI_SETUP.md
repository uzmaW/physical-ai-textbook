# ChatUI Setup Guide for Physical AI Textbook

This guide explains how the ChatUI integration works in the Docusaurus-based textbook and how to set it up for development and production.

## 🏗️ Architecture Overview

The ChatUI system consists of three main parts:

1. **Frontend Components** (React/TypeScript in Docusaurus)
   - `RAGChatWidget.tsx` - Main chat interface displayed in the right sidebar
   - `RAGChatbox.tsx` - Alternative chat component
   - `apiService.ts` - API communication layer

2. **Backend API** (FastAPI + Python)
   - Located in `/backend/app/`
   - Handles RAG queries, vector search, and AI responses
   - Integrates with Qdrant (vector store) and OpenAI

3. **Theme Integration** (Docusaurus theme swizzling)
   - `src/theme/TOC/index.tsx` - Replaces table of contents with ChatUI
   - `src/theme/Root.tsx` - Root wrapper component

## 📁 File Structure

```
physical-ai-humanoid-textbook/
├── src/
│   ├── components/
│   │   ├── RAGChatWidget.tsx      # Main chat UI component
│   │   ├── RAGChatbox.tsx         # Alternative chat component
│   │   └── PersonalizedChapter.tsx
│   ├── services/
│   │   └── apiService.ts          # NEW: API communication service
│   ├── theme/
│   │   ├── TOC/index.tsx          # Swizzled TOC → ChatUI
│   │   └── Root.tsx
│   └── store/
│       └── userStore.ts           # User preferences store
├── .env.local                     # NEW: Local dev environment vars
├── .env.example                   # UPDATED: Environment template
└── docusaurus.config.js

backend/
├── app/
│   ├── main.py                    # FastAPI application
│   ├── routers/
│   │   └── chat.py                # Chat endpoint: POST /api/chat/
│   ├── services/
│   │   └── rag.py                 # RAG service (to be implemented)
│   └── config.py
└── requirements.txt
```

## 🔧 Setup Instructions

### 1. Frontend Setup (Docusaurus)

#### Install Dependencies
```bash
cd physical-ai-humanoid-textbook
npm install
```

#### Configure Environment Variables
```bash
# Copy the example environment file
cp .env.example .env.local

# Edit .env.local and set the backend API URL
# For local development:
REACT_APP_API_URL=http://localhost:8000
```

#### Start Development Server
```bash
npm start
```

The textbook will be available at `http://localhost:3000` with the ChatUI in the right sidebar.

### 2. Backend Setup (FastAPI)

#### Install Python Dependencies
```bash
cd ../backend
pip install -r requirements.txt
```

#### Configure Backend Environment
```bash
# Copy the example environment file
cp .env.example .env

# Edit .env and configure:
# - DATABASE_URL (PostgreSQL)
# - QDRANT_URL and QDRANT_API_KEY (Vector database)
# - OPENAI_API_KEY (AI model)
# - FRONTEND_URL (CORS configuration)
```

Example `.env`:
```bash
DATABASE_URL=postgresql://user:password@localhost/humanoid_textbook
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-qdrant-key
OPENAI_API_KEY=sk-your-openai-key
FRONTEND_URL=http://localhost:3000
```

#### Start Backend Server
```bash
# From the backend directory
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000` with:
- API docs: `http://localhost:8000/docs`
- Health check: `http://localhost:8000/health`
- Chat endpoint: `http://localhost:8000/api/chat/`

### 3. Vector Database Setup (Qdrant)

#### Option A: Local Qdrant (Docker)
```bash
docker run -p 6333:6333 qdrant/qdrant
```

#### Option B: Qdrant Cloud
1. Sign up at https://cloud.qdrant.io
2. Create a cluster
3. Copy the API key and URL to your `.env` file

#### Index the Textbook Content
```bash
# Run the indexing script (to be created)
cd backend
python scripts/index_textbook.py
```

This will process all markdown files from the `docs/` folder and create vector embeddings in Qdrant.

## 🧪 Testing the ChatUI

### 1. Check Backend Health
```bash
curl http://localhost:8000/health
```

Expected response:
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "services": {
    "database": "connected",
    "qdrant": "connected",
    "openai": "configured"
  }
}
```

### 2. Test Chat Endpoint
```bash
curl -X POST http://localhost:8000/api/chat/ \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is a humanoid robot?",
    "userLevel": "intermediate",
    "language": "en"
  }'
```

Expected response:
```json
{
  "answer": "A humanoid robot is...",
  "citations": [
    {
      "chapter": "Week 01",
      "url": "/chapters/week-01",
      "relevance": 0.95
    }
  ],
  "sourcesCount": 3,
  "model": "gpt-4o-mini"
}
```

### 3. Test Frontend Integration

1. Open `http://localhost:3000` in your browser
2. Navigate to any chapter
3. Look for the ChatUI in the right sidebar
4. Type a question and press Enter
5. Verify that you receive a response with citations

## 🚨 Troubleshooting

### ChatUI shows "Backend API is not available"

**Possible causes:**
1. Backend server is not running
2. Wrong API URL in `.env.local`
3. CORS configuration issue

**Solutions:**
```bash
# Check if backend is running
curl http://localhost:8000/health

# Verify environment variable
echo $REACT_APP_API_URL  # Should show http://localhost:8000

# Restart Docusaurus to pick up .env changes
npm start
```

### CORS errors in browser console

**Solution:** Update `backend/app/main.py` to include your frontend URL:
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Add this
        # ... other origins
    ],
)
```

### Chat responses are slow

**Solutions:**
1. Use `gpt-4o-mini` instead of `gpt-4` for faster responses
2. Reduce `max_context_chunks` in chat requests
3. Implement response caching in the backend

### Citations not showing up

**Possible causes:**
1. Textbook content not indexed in Qdrant
2. RAG service not finding relevant chunks

**Solution:**
```bash
# Re-index the textbook content
cd backend
python scripts/index_textbook.py --force
```

## 🚀 Production Deployment

### Frontend (Vercel/Netlify/GitHub Pages)

1. Create `.env.production` file:
```bash
REACT_APP_API_URL=https://your-backend.fly.io
```

2. Build the static site:
```bash
npm run build
```

3. Deploy the `build/` directory to your hosting provider

### Backend (Fly.io/Railway/Render)

1. Configure production environment variables in your hosting platform
2. Deploy the `backend/` directory
3. Ensure the database and Qdrant are accessible from the backend
4. Update CORS settings with your production frontend URL

## 📊 Monitoring

### Backend Metrics
- Check `/health` endpoint regularly
- Monitor API response times
- Track error rates in chat endpoint

### Frontend Metrics
- Monitor chat usage analytics
- Track user engagement with ChatUI
- Collect feedback on response quality

## 🔐 Security Best Practices

1. **Never commit `.env` or `.env.local` files** to Git
2. Use environment variables for all secrets
3. Implement rate limiting on the chat endpoint
4. Validate and sanitize all user inputs
5. Use HTTPS in production
6. Implement authentication for sensitive operations

## 📚 API Reference

### Chat Endpoint

**POST** `/api/chat/`

Request body:
```typescript
{
  message: string;          // User's question
  selectedText?: string;    // Optional selected text for context
  userLevel?: string;       // "beginner" | "intermediate" | "advanced"
  language?: string;        // "en" | "ur"
}
```

Response:
```typescript
{
  answer: string;           // AI-generated response
  citations: Array<{        // Source citations
    chapter: string;
    url: string;
    relevance: number;
  }>;
  sourcesCount: number;     // Total sources used
  model: string;            // AI model used
}
```

## 🎯 Next Steps

1. **Implement RAG Service**: Complete the `backend/app/services/rag.py` file
2. **Index Content**: Create and run the textbook indexing script
3. **Add Conversation History**: Implement conversation tracking in backend
4. **Improve UI**: Add features like copy button, regenerate response, etc.
5. **Add Analytics**: Track chat usage and improve responses based on feedback

## 📝 Additional Resources

- [Docusaurus Documentation](https://docusaurus.io)
- [FastAPI Documentation](https://fastapi.tiangolo.com)
- [Qdrant Documentation](https://qdrant.tech/documentation)
- [OpenAI API Reference](https://platform.openai.com/docs/api-reference)

## ✅ Checklist

- [ ] Frontend dependencies installed
- [ ] Backend dependencies installed
- [ ] Environment variables configured
- [ ] Qdrant running and accessible
- [ ] OpenAI API key configured
- [ ] Textbook content indexed
- [ ] Backend server running
- [ ] Frontend server running
- [ ] ChatUI visible in browser
- [ ] Test chat message sent successfully
- [ ] Citations displaying correctly

---

**Status:** ChatUI integration is now properly configured with the API service layer. The system will gracefully handle backend unavailability with user-friendly error messages.
