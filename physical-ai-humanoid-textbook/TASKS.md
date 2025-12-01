# Implementation Tasks: Intelligent Textbook Platform

## Phase 1: Frontend Enhancement ✅ (COMPLETED)

### ✅ Task 1.1: PersonalizedChapter Component
**Status**: COMPLETE
**File**: `src/components/PersonalizedChapter.tsx`
**Features**:
- Difficulty toggle (beginner/intermediate/advanced)
- Translation button with loading state
- API integration for translation caching
- RTL support for Urdu content

### ✅ Task 1.2: MDXComponents Registration
**Status**: COMPLETE
**File**: `src/theme/MDXComponents.tsx`
**Features**:
- Registered PersonalizedChapter for use in MDX
- Tabs and TabItem components available

### ✅ Task 1.3: Enhanced RAG Chatbot
**Status**: COMPLETE
**File**: `src/components/RAGChatWidget.tsx`
**Features**:
- Selected text detection (mouseup event)
- "Ask about this" button for selected text
- Citations display
- Loading states with animated dots
- Backend API integration ready

---

## Phase 2: Backend Development 🔧 (TO DO)

### Task 2.1: FastAPI Project Setup
**Priority**: HIGH
**Estimated Time**: 2 hours

**Steps**:
1. Create directory structure:
```bash
mkdir -p backend/{app/{routers,services,models,db},scripts}
touch backend/app/__init__.py
touch backend/app/{routers,services,models,db}/__init__.py
```

2. Create `backend/requirements.txt`:
```txt
fastapi==0.109.0
uvicorn[standard]==0.27.0
sqlalchemy==2.0.25
psycopg[binary]==3.1.16
alembic==1.13.1
qdrant-client==1.7.3
openai==1.10.0
tiktoken==0.5.2
python-jose[cryptography]==3.3.0
passlib[bcrypt]==1.7.4
python-dotenv==1.0.0
google-cloud-translate==3.15.0
langchain==0.1.0
langchain-openai==0.0.5
httpx==0.26.0
pydantic==2.5.3
pydantic-settings==2.1.0
```

3. Create `backend/app/main.py` (FastAPI app with CORS)

4. Create `backend/app/config.py` (environment variables with pydantic-settings)

**Acceptance Criteria**:
- [ ] FastAPI app runs on port 8000
- [ ] `/health` endpoint returns `{"status": "healthy"}`
- [ ] CORS configured for localhost:3000
- [ ] Environment variables loaded from .env

---

### Task 2.2: Database Models
**Priority**: HIGH
**Estimated Time**: 1.5 hours
**Depends On**: Task 2.1

**Files to Create**:

**`backend/app/models/user.py`**:
```python
from sqlalchemy import Column, String, JSON, DateTime, Enum
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime
import enum

Base = declarative_base()

class HardwareLevel(str, enum.Enum):
    NONE = "none"
    ARDUINO = "arduino"
    ROS = "ros"
    INDUSTRIAL = "industrial"

class SoftwareLevel(str, enum.Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class User(Base):
    __tablename__ = "users"
    id = Column(String, primary_key=True)
    email = Column(String, unique=True, nullable=False)
    hashed_password = Column(String)
    oauth_provider = Column(String)
    oauth_id = Column(String)
    created_at = Column(DateTime, default=datetime.utcnow)

class UserMetadata(Base):
    __tablename__ = "user_metadata"
    user_id = Column(String, primary_key=True)
    hardware_background = Column(Enum(HardwareLevel))
    software_level = Column(Enum(SoftwareLevel))
    learning_goal = Column(String)
    difficulty_preference = Column(String, default='intermediate')
    completed_chapters = Column(JSON, default=list)

class TranslationCache(Base):
    __tablename__ = "translation_cache"
    id = Column(String, primary_key=True)
    user_id = Column(String)
    chapter_id = Column(String)
    translated_content = Column(String)
    created_at = Column(DateTime, default=datetime.utcnow)
```

**`backend/app/db/neon.py`**:
```python
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
import os

DATABASE_URL = os.getenv("DATABASE_URL")
engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(bind=engine)

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
```

**Acceptance Criteria**:
- [ ] Models defined with proper types
- [ ] Database connection works
- [ ] Tables created in Neon (run migrations)

---

### Task 2.3: Qdrant Vector Store Setup
**Priority**: HIGH
**Estimated Time**: 2 hours
**Depends On**: Task 2.1

**Files to Create**:

**`backend/scripts/setup_qdrant.py`** - Create collection

**`backend/scripts/generate_embeddings.py`** - Embed all 13 chapters

**Key Steps**:
1. Create Qdrant collection with 3072-dim vectors
2. Read all week-*.mdx files
3. Chunk content (1000 tokens, 200 overlap)
4. Generate embeddings with `text-embedding-3-large`
5. Upload to Qdrant with metadata

**Acceptance Criteria**:
- [ ] Qdrant collection created
- [ ] All 13 chapters embedded (~390 chunks estimated)
- [ ] Metadata includes: chapter_id, title, url, difficulty, chunk_index
- [ ] Search returns relevant results

---

### Task 2.4: RAG Service Implementation
**Priority**: HIGH
**Estimated Time**: 3 hours
**Depends On**: Task 2.3

**File**: `backend/app/services/rag.py`

**Features**:
- Embed user query
- Semantic search in Qdrant (top-5)
- Filter by user difficulty level
- Build context from search results
- Call GPT-4o-mini with context
- Return answer + citations

**File**: `backend/app/routers/chat.py`

```python
from fastapi import APIRouter
from pydantic import BaseModel
from app.services.rag import RAGService

router = APIRouter()
rag_service = RAGService()

class ChatRequest(BaseModel):
    message: str
    selectedText: str | None = None
    userLevel: str = "intermediate"
    language: str = "en"

@router.post("/")
async def chat(request: ChatRequest):
    result = await rag_service.ask(
        question=request.message,
        selected_text=request.selectedText,
        user_level=request.userLevel
    )
    return result
```

**Acceptance Criteria**:
- [ ] `/api/chat` endpoint works
- [ ] Returns answer + citations
- [ ] Response time < 2 seconds
- [ ] Handles selected text context

---

### Task 2.5: Translation Service
**Priority**: MEDIUM
**Estimated Time**: 2 hours
**Depends On**: Task 2.2

**Files to Create**:

**`backend/app/services/translation.py`**:
- Google Translate API wrapper
- Caching logic (check DB first)

**`backend/app/routers/translate.py`**:
```python
@router.get("/cached")
async def get_cached(userId: str, chapterId: str):
    # Query translation_cache table
    pass

@router.post("/")
async def translate(request: TranslateRequest):
    # 1. Check cache
    # 2. If miss, call Google Translate
    # 3. Store in DB
    # 4. Return translated content
    pass
```

**Acceptance Criteria**:
- [ ] Translation works for sample chapter
- [ ] Caching reduces API calls
- [ ] Urdu text displays correctly (RTL)

---

## Phase 3: Authentication 🔐 (TO DO)

### Task 3.1: better-auth Integration
**Priority**: MEDIUM
**Estimated Time**: 3 hours

**Frontend**:
```bash
npm install better-auth @better-auth/react
```

**File**: `src/lib/auth.ts`
```typescript
import { createAuthClient } from "@better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_API_URL || "http://localhost:8000",
});

export const { signIn, signUp, signOut, useSession } = authClient;
```

**Backend**:
```python
# backend/app/routers/auth.py
# Endpoints: /signup, /login, /logout, /me
```

**Acceptance Criteria**:
- [ ] Signup creates user in Neon
- [ ] Login returns JWT token
- [ ] OAuth works (GitHub, Google)
- [ ] Session persists in frontend

---

### Task 3.2: Signup Survey Form
**Priority**: MEDIUM
**Estimated Time**: 2 hours
**Depends On**: Task 3.1

**File**: `src/components/SignupForm.tsx`

**Flow**:
1. Step 1: Email + Password
2. Step 2: Hardware background dropdown
3. Step 3: Software level dropdown
4. Step 4: Learning goal textarea
5. Submit → POST `/api/auth/signup` with metadata

**Acceptance Criteria**:
- [ ] 4-step form works
- [ ] Data saved to user_metadata table
- [ ] Redirects to /chapters/week-01 after signup

---

## Phase 4: Claude Subagents 🤖 (TO DO)

### Task 4.1-4.5: Create 5 Subagent Commands
**Priority**: LOW
**Estimated Time**: 3 hours total

**Directory**: `.claude/commands/`

**Files to Create**:

1. **`sp.ros2-lab-gen.md`**
   - Generates ROS 2 Python node templates
   - Input: topic name, message type
   - Output: Complete .py file

2. **`sp.gazebo-scene.md`**
   - Generates Gazebo worlds
   - Input: environment description
   - Output: .sdf + .urdf + .launch.py

3. **`sp.isaac-pipeline.md`**
   - Generates Isaac ROS configs
   - Input: perception task
   - Output: YAML + launch file

4. **`sp.vla-planner.md`**
   - Generates VLA planners
   - Input: action primitives
   - Output: Python node with OpenAI

5. **`sp.quiz-maker.md`**
   - Auto-generates MCQs
   - Input: chapter ID
   - Output: JSON with questions

**Acceptance Criteria**:
- [ ] All 5 command files created
- [ ] Each follows Claude Code command format
- [ ] Tested with sample inputs

---

## Phase 5: Deployment 🚀 (TO DO)

### Task 5.1: GitHub Actions Workflow
**Priority**: HIGH
**Estimated Time**: 1 hour

**File**: `.github/workflows/deploy.yml`

**Triggers**: Push to main
**Jobs**:
1. Lint (ESLint, Prettier)
2. Build (npm run build)
3. Deploy to gh-pages

**Acceptance Criteria**:
- [ ] Workflow runs on push
- [ ] Builds successfully
- [ ] Deploys to https://piaic.github.io/humanoid-ai-textbook

---

### Task 5.2: Environment Configuration
**Priority**: HIGH
**Estimated Time**: 30 minutes

**Files**:
- `.env.example` (frontend)
- `backend/.env.example`
- GitHub Secrets configuration

**Variables**:
- `DATABASE_URL` (Neon)
- `QDRANT_URL`, `QDRANT_API_KEY`
- `OPENAI_API_KEY`
- `GOOGLE_TRANSLATE_API_KEY`
- `GITHUB_CLIENT_ID/SECRET`
- `GOOGLE_CLIENT_ID/SECRET`
- `JWT_SECRET`

**Acceptance Criteria**:
- [ ] All secrets configured
- [ ] Frontend connects to backend
- [ ] No secrets committed to git

---

## Task Summary

| Phase | Tasks | Status |
|-------|-------|--------|
| Frontend | 3 tasks | ✅ COMPLETE |
| Backend | 5 tasks | 🔧 PENDING |
| Auth | 2 tasks | 🔧 PENDING |
| Subagents | 5 tasks | 🔧 PENDING |
| Deployment | 2 tasks | 🔧 PENDING |

**Total**: 17 tasks
**Completed**: 3
**Remaining**: 14

---

## Next Steps

1. **Set up Neon PostgreSQL database** (create free tier account)
2. **Set up Qdrant Cloud** (create free tier collection)
3. **Implement backend** (FastAPI structure)
4. **Generate embeddings** (run embedding script)
5. **Test RAG endpoint** (verify question answering)
6. **Deploy to production** (GitHub Pages + Railway)

---

**Ready to continue implementation!** 🚀

Current server running at: http://localhost:3000
- ✅ PersonalizedChapter component created
- ✅ RAG chatbot enhanced
- ✅ Beautiful sidebar with separators
- 🔧 Backend implementation next

