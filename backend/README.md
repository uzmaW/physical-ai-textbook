title: Physical AI Textbook
emoji: 📚
colorFrom: blue
colorTo: green
sdk: docker
pinned: false
app_file: app/main.py

# Physical AI Textbook Backend

FastAPI backend providing RAG-powered Q&A, translation, and personalization services for the Physical AI & Humanoid Robotics interactive textbook.

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│  FastAPI Backend (Python 3.8+)                          │
│  ┌───────────────────────────────────────────────────┐  │
│  │  API Routers                                      │  │
│  │  - /api/chat/    → RAG Q&A                       │  │
│  │  - /api/translate/ → Urdu translation           │  │
│  │  - /health       → Health check                  │  │
│  └───────────┬───────────────────────────────────────┘  │
│              │                                           │
│  ┌───────────▼───────────────────────────────────────┐  │
│  │  Services                                         │  │
│  │  - RAG Service (rag.py)                         │  │
│  │    • Semantic search in Qdrant                  │  │
│  │    • AI response generation with GPT-4o-mini    │  │
│  │  - Translation (via router)                     │  │
│  │    • Google Cloud Translate                     │  │
│  │    • PostgreSQL caching                         │  │
│  └───────────┬───────────────────────────────────────┘  │
│              │                                           │
│  ┌───────────▼───────────────────────────────────────┐  │
│  │  External Services                                │  │
│  │  - Qdrant (Vector Store)                         │  │
│  │  - OpenAI API                                    │  │
│  │  - PostgreSQL (Neon)                             │  │
│  │  - Google Translate API                          │  │
│  └───────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

## 📁 Project Structure

```
backend/
├── app/
│   ├── main.py              # FastAPI application entry point
│   ├── config.py            # Configuration management
│   ├── routers/
│   │   ├── chat.py          # RAG Q&A endpoint
│   │   └── translate.py     # Translation endpoint
│   ├── services/
│   │   └── rag.py           # RAG service (vector search + AI)
│   ├── models/
│   │   └── user.py          # SQLAlchemy models
│   └── db/
│       ├── neon.py          # PostgreSQL connection
│       └── __init__.py
├── scripts/
│   ├── setup_qdrant.py      # Create Qdrant collection
│   ├── generate_embeddings.py  # Generate embeddings (legacy)
│   └── index_textbook.py    # Complete indexing pipeline (NEW)
├── requirements.txt         # Python dependencies
├── .env.example            # Environment variables template
├── setup.sh                # Setup script (NEW)
└── run.sh                  # Run script (NEW)
```

## 🚀 Quick Start

### 1. Setup

```bash
# Clone repository and navigate to backend
cd backend

# Run setup script (creates venv, installs dependencies)
./setup.sh

# Edit .env with your API keys
nano .env
```

### 2. Configure Environment

Edit `.env` and add your credentials:

```bash
# Required for RAG Q&A
OPENAI_API_KEY=sk-your-key-here
QDRANT_URL=http://localhost:6333  # or cloud URL
QDRANT_API_KEY=your-qdrant-key     # optional for local

# Required for PostgreSQL features
DATABASE_URL=postgresql://user:pass@host/db

# Optional: Google Translate
GOOGLE_TRANSLATE_API_KEY=your-key
```

### 3. Setup Vector Store

```bash
# Activate virtual environment
source venv/bin/activate

# Create Qdrant collection
python scripts/setup_qdrant.py
```

### 4. Index Textbook Content

```bash
# Index all chapters (auto-detects docs directory)
python scripts/index_textbook.py

# Or with options:
python scripts/index_textbook.py --verbose --force

# Specify custom docs directory:
python scripts/index_textbook.py --docs-dir /path/to/docs
```

### 5. Start Backend

```bash
# Using run script
./run.sh

# Or manually with uvicorn
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at:
- **Docs:** http://localhost:8000/docs
- **Health:** http://localhost:8000/health
- **Chat:** http://localhost:8000/api/chat/

## 🧪 Testing the Backend

### Health Check

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

### Chat Endpoint

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
  "answer": "A humanoid robot is a robot with a body shape built to resemble the human body...",
  "citations": [
    {
      "chapter": "Week 01 - Introduction",
      "url": "/chapters/week-01",
      "relevance": 0.92
    }
  ],
  "sourcesCount": 5,
  "model": "gpt-4o-mini"
}
```

### Translation Endpoint

```bash
curl -X POST http://localhost:8000/api/translate/ \
  -H "Content-Type: application/json" \
  -d '{
    "userId": "test-user",
    "chapterId": "week-01",
    "content": "Humanoid robots are designed to mimic human form.",
    "targetLang": "ur"
  }'
```

## 📊 Services Details

### RAG Service (`app/services/rag.py`)

**Features:**
- ✅ Semantic search using OpenAI embeddings
- ✅ Qdrant vector store integration
- ✅ Context-aware AI responses with GPT-4o-mini
- ✅ Difficulty level filtering (beginner/intermediate/advanced)
- ✅ Multi-language support (English/Urdu)
- ✅ Source citations with relevance scores

**Main Methods:**
- `embed_query()` - Generate query embedding
- `search_context()` - Find relevant text chunks
- `generate_answer()` - Create AI response
- `ask()` - Complete RAG pipeline

### Translation Service (`app/routers/translate.py`)

**Features:**
- ✅ Google Cloud Translate API integration
- ✅ PostgreSQL caching for cost optimization
- ✅ HTML format preservation
- ✅ User-specific translation management

**Endpoints:**
- `POST /api/translate/` - Translate content
- `GET /api/translate/cached` - Get cached translation
- `DELETE /api/translate/cache/{userId}/{chapterId}` - Clear cache

## 🔧 Configuration

### Required Environment Variables

```bash
# OpenAI (Required for RAG)
OPENAI_API_KEY=sk-...

# Qdrant (Required for RAG)
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=optional-for-local

# PostgreSQL (Required for caching/auth)
DATABASE_URL=postgresql://user:pass@host:5432/dbname

# Frontend CORS (Required)
FRONTEND_URL=http://localhost:3000
```

### Optional Environment Variables

```bash
# Google Translate
GOOGLE_APPLICATION_CREDENTIALS=/path/to/service-account.json
GOOGLE_TRANSLATE_API_KEY=...

# Authentication (Future features)
JWT_SECRET=your-secret-key
GITHUB_CLIENT_ID=...
GITHUB_CLIENT_SECRET=...
GOOGLE_CLIENT_ID=...
GOOGLE_CLIENT_SECRET=...

# Application
DEBUG=true
API_VERSION=1.0.0
```

## 📦 Dependencies

### Core
- **FastAPI** - Web framework
- **Uvicorn** - ASGI server
- **SQLAlchemy** - Database ORM
- **Psycopg** - PostgreSQL adapter

### AI & Vector Search
- **OpenAI** - Embeddings and chat completion
- **Qdrant Client** - Vector database
- **Tiktoken** - Token counting

### Translation
- **Google Cloud Translate** - Translation API

### Utils
- **Pydantic Settings** - Configuration
- **Python Frontmatter** - Parse markdown frontmatter

## 🎯 Indexing Script Details

The `index_textbook.py` script provides a complete indexing pipeline:

### Features
- ✅ Automatic collection setup
- ✅ Semantic chunking with overlap
- ✅ Batch embedding generation
- ✅ Error handling and progress tracking
- ✅ CLI options for flexibility

### Usage

```bash
# Basic usage (auto-detects docs directory)
python scripts/index_textbook.py

# Force recreate collection
python scripts/index_textbook.py --force

# Verbose output
python scripts/index_textbook.py --verbose

# Custom docs directory
python scripts/index_textbook.py --docs-dir ../my-textbook/docs

# Custom file pattern
python scripts/index_textbook.py --pattern "chapter-*.md"

# Combine options
python scripts/index_textbook.py --force --verbose --docs-dir /path/to/docs
```

### What It Does

1. **Validates Environment** - Checks API keys and connections
2. **Creates Collection** - Sets up Qdrant with proper configuration
3. **Processes Chapters** - Reads all matching files
4. **Chunks Content** - Splits into semantic chunks with overlap
5. **Generates Embeddings** - Creates 3072-dim vectors with OpenAI
6. **Uploads to Qdrant** - Stores vectors with rich metadata
7. **Reports Results** - Shows progress and final statistics

### Metadata Stored

Each chunk includes:
```python
{
    "content": "Actual text content...",
    "chapter_id": "week-01",
    "chapter_title": "Introduction to Humanoid Robotics",
    "url": "/chapters/week-01",
    "chunk_index": 0,
    "total_chunks": 15,
    "difficulty": "intermediate",
    "tokens": 234,
    "file_path": "/path/to/week-01.mdx"
}
```

## 🚨 Troubleshooting

### OpenAI API Errors

**Error:** `openai.AuthenticationError`
**Solution:** Check `OPENAI_API_KEY` in `.env`

**Error:** Rate limit exceeded
**Solution:** Add delays between requests or upgrade API plan

### Qdrant Connection Issues

**Error:** Connection refused
**Solution:**
```bash
# Start local Qdrant
docker run -p 6333:6333 qdrant/qdrant

# Or check cloud URL/API key
```

### Database Errors

**Error:** Connection to PostgreSQL failed
**Solution:** Verify `DATABASE_URL` format:
```bash
postgresql://username:password@hostname:5432/database_name
```

### Indexing Fails

**Error:** No files found
**Solution:** Check docs directory path:
```bash
python scripts/index_textbook.py --docs-dir /absolute/path/to/docs
```

**Error:** Frontmatter parsing error
**Solution:** Ensure MDX files have valid frontmatter:
```yaml
---
id: week-01
title: Introduction
---
```

## 🔐 Security Best Practices

1. **Never commit `.env` files** - Use `.env.example` as template
2. **Use strong secrets** - Generate with `openssl rand -hex 32`
3. **Enable HTTPS in production** - Use reverse proxy (nginx)
4. **Implement rate limiting** - Prevent API abuse
5. **Validate all inputs** - Pydantic models handle this
6. **Use environment-specific configs** - Different keys for dev/prod

## 📈 Performance Optimization

### Vector Search
- **Index size:** 3072-dim vectors, ~12KB per chunk
- **Search speed:** <100ms for top-5 results
- **Optimize:** Reduce `top_k` if slow

### AI Generation
- **Model:** GPT-4o-mini (fast, cost-effective)
- **Typical latency:** 500ms-2s
- **Optimize:** Lower `max_tokens` or add caching

### Translation
- **Caching:** PostgreSQL stores translations
- **Hit rate:** ~80% for popular chapters
- **Cost:** $20/1M characters (Google Translate)

## 🚀 Production Deployment

### Option 1: Railway

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login and deploy
railway login
railway up
```

### Option 2: Fly.io

```bash
# Install flyctl
curl -L https://fly.io/install.sh | sh

# Deploy
fly launch
fly deploy
```

### Option 3: Render

1. Connect GitHub repository
2. Set build command: `pip install -r requirements.txt`
3. Set start command: `uvicorn app.main:app --host 0.0.0.0 --port 8000`
4. Add environment variables in dashboard

### Production Checklist

- [ ] Environment variables configured
- [ ] Database hosted (Neon, Supabase, etc.)
- [ ] Qdrant Cloud instance created
- [ ] Textbook content indexed
- [ ] CORS configured for frontend URL
- [ ] Health check endpoint working
- [ ] API documentation accessible
- [ ] Error monitoring setup (Sentry)
- [ ] Rate limiting configured

## 📚 API Reference

### Chat Endpoint

**POST** `/api/chat/`

Request:
```typescript
{
  message: string;          // User's question
  selectedText?: string;    // Optional selected context
  userLevel?: string;       // "beginner" | "intermediate" | "advanced"
  language?: string;        // "en" | "ur"
}
```

Response:
```typescript
{
  answer: string;           // AI-generated response
  citations: Array<{
    chapter: string;
    url: string;
    relevance: number;
  }>;
  sourcesCount: number;     // Total sources used
  model: string;            // "gpt-4o-mini"
}
```

### Translation Endpoint

**POST** `/api/translate/`

Request:
```typescript
{
  userId: string;
  chapterId: string;
  content: string;
  targetLang: string;       // "ur"
}
```

Response:
```typescript
{
  translatedContent: string;
  cached: boolean;
  chapterId: string;
  characterCount: number;
}
```

## 🎓 Learning Resources

- [FastAPI Documentation](https://fastapi.tiangolo.com)
- [Qdrant Documentation](https://qdrant.tech/documentation)
- [OpenAI API Reference](https://platform.openai.com/docs/api-reference)
- [SQLAlchemy Docs](https://docs.sqlalchemy.org)

## 🤝 Contributing

1. Follow existing code structure
2. Add type hints to all functions
3. Write docstrings for new services
4. Test endpoints before committing
5. Update this README for new features

## 📝 License

This backend is part of the Physical AI & Humanoid Robotics textbook project.

---

**Ready to Start?** Run `./setup.sh` and follow the Quick Start guide above! 🚀
