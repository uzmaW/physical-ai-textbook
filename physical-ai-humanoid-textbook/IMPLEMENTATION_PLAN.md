# Implementation Plan: Full-Stack Intelligent Textbook Platform

## 🎯 Objective

Deploy the Physical AI & Humanoid Robotics textbook as a complete intelligent learning platform with personalization, translation, RAG, authentication, and AI-powered content generation.

---

## 📋 Current Status

### ✅ Completed
- 13 comprehensive chapters (332 KB, ~75,000 words)
- Docusaurus v3 platform with Tailwind CSS
- Beautiful sidebar with visual separators
- 3-column layout (sidebar | content | RAG chat)
- Zustand state management
- Basic RAG chatbox UI
- Cover page with logo

### 🔧 To Implement
- PersonalizedChapter MDX wrapper
- Translation functionality with caching
- better-auth integration
- FastAPI backend (Neon + Qdrant + OpenAI)
- 5 Claude Code subagents
- GitHub Pages deployment with CI/CD

---

## Phase 1: Frontend Enhancement (Week 1)

### Task 1.1: PersonalizedChapter Component

**File**: `src/components/PersonalizedChapter.tsx`

**Features**:
```typescript
interface PersonalizedChapterProps {
  id: string;
  children: React.ReactNode;
}

export function PersonalizedChapter({ id, children }: PersonalizedChapterProps) {
  const { userProfile, updatePreferences } = useUserStore();
  const [difficulty, setDifficulty] = useState(
    userProfile?.preferences?.difficulty_preference || 'intermediate'
  );
  const [isUrdu, setIsUrdu] = useState(false);
  const [urduContent, setUrduContent] = useState<string | null>(null);
  const [isTranslating, setIsTranslating] = useState(false);

  // Cycle through difficulty levels
  const handlePersonalize = () => {
    const levels = ['beginner', 'intermediate', 'advanced'];
    const currentIndex = levels.indexOf(difficulty);
    const nextLevel = levels[(currentIndex + 1) % 3];
    setDifficulty(nextLevel);
    updatePreferences({ difficulty_preference: nextLevel });
  };

  // Toggle Urdu translation
  const handleTranslate = async () => {
    if (isUrdu) {
      setIsUrdu(false);
      return;
    }

    setIsTranslating(true);

    try {
      // Check cache first
      const response = await fetch(
        `/api/translate/cached?userId=${userProfile?.id}&chapterId=${id}`
      );

      if (response.ok) {
        const cached = await response.json();
        setUrduContent(cached.content);
      } else {
        // Translate and cache
        const translateResponse = await fetch('/api/translate', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            userId: userProfile?.id,
            chapterId: id,
            content: extractTextContent(children)
          })
        });

        const result = await translateResponse.json();
        setUrduContent(result.translatedContent);
      }

      setIsUrdu(true);
    } catch (error) {
      console.error('Translation failed:', error);
    } finally {
      setIsTranslating(false);
    }
  };

  return (
    <div className="personalized-chapter">
      {/* Header with action buttons */}
      <div className="chapter-header flex items-center justify-between p-4 bg-gray-50 dark:bg-gray-800 rounded-lg mb-6">
        <div className="flex items-center gap-3">
          <span className={`difficulty-badge difficulty-${difficulty} px-3 py-1 rounded-full text-sm font-medium`}>
            {difficulty.charAt(0).toUpperCase() + difficulty.slice(1)} Mode
          </span>
        </div>

        <div className="flex gap-3">
          <button
            onClick={handlePersonalize}
            className="personalize-btn px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition-colors"
            title="Cycle through difficulty levels"
          >
            🎯 Personalize Content
          </button>

          <button
            onClick={handleTranslate}
            className="translate-btn px-4 py-2 bg-green-600 text-white rounded-lg hover:bg-green-700 transition-colors font-urdu"
            disabled={isTranslating}
            title={isUrdu ? 'Switch to English' : 'Translate to Urdu'}
          >
            {isTranslating ? '⏳ Translating...' : isUrdu ? 'English' : 'ترجمہ کریں'}
          </button>
        </div>
      </div>

      {/* Content */}
      <div
        className={`chapter-content ${isUrdu ? 'urdu-text' : ''}`}
        data-difficulty={difficulty}
      >
        {isUrdu && urduContent ? (
          <div className="prose dark:prose-invert max-w-none" dangerouslySetInnerHTML={{ __html: urduContent }} />
        ) : (
          <div className="prose dark:prose-invert max-w-none">
            {children}
          </div>
        )}
      </div>
    </div>
  );
}
```

**Acceptance Criteria**:
- [ ] Difficulty toggle cycles: beginner → intermediate → advanced → beginner
- [ ] Badge displays current mode
- [ ] Translation button calls API
- [ ] Cached translations load instantly (<100ms)
- [ ] Urdu content displays with RTL layout
- [ ] State persists in Zustand

---

### Task 1.2: MDXComponents Registration

**File**: `src/theme/MDXComponents.tsx`

```typescript
import React from 'react';
import MDXComponents from '@theme-original/MDXComponents';
import { PersonalizedChapter } from '@site/src/components/PersonalizedChapter';

export default {
  ...MDXComponents,
  PersonalizedChapter,
};
```

**Update all chapter MDX files** to wrap content:
```mdx
---
id: week-01
title: Week 1 - Foundations
---

import { PersonalizedChapter } from '@site/src/components/PersonalizedChapter';

<PersonalizedChapter id="week-01">

# Week 1: Foundations of Physical AI

... (rest of content) ...

</PersonalizedChapter>
```

---

### Task 1.3: Enhanced RAG Chatbot

**File**: `src/components/RAGChatWidget.tsx` (enhance existing)

**New Features**:
- Selected text detection with "Ask about this" prompt
- Backend API integration
- Citation links
- Loading states
- Error handling

```typescript
// Add to existing component
const [selectedText, setSelectedText] = useState('');

useEffect(() => {
  const handleSelection = () => {
    const selection = window.getSelection();
    const text = selection?.toString().trim() || '';
    if (text && text.length > 10) {
      setSelectedText(text);
    }
  };

  document.addEventListener('mouseup', handleSelection);
  return () => document.removeEventListener('mouseup', handleSelection);
}, []);

const sendMessage = async (message: string, context?: string) => {
  // API call to /api/chat
  const response = await fetch('/api/chat', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${userProfile?.authToken}`
    },
    body: JSON.stringify({
      message,
      selectedText: context || selectedText,
      history: messages.slice(-5),
      userLevel: userProfile?.preferences?.difficulty_preference || 'intermediate',
      language: userProfile?.preferences?.language || 'en'
    })
  });

  const data = await response.json();
  // Display response with citations
};
```

---

## Phase 2: Backend Development (Week 2)

### Task 2.1: FastAPI Project Structure

```bash
mkdir -p backend/{app/{routers,services,models,db},scripts}
```

**Files to create**:

**`backend/requirements.txt`**:
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
redis==5.0.1
```

---

### Task 2.2: Database Models

**File**: `backend/app/models/user.py`

```python
from sqlalchemy import Column, String, JSON, DateTime, Enum as SQLEnum
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
    oauth_provider = Column(String)  # 'github', 'google', null
    oauth_id = Column(String)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class UserMetadata(Base):
    __tablename__ = "user_metadata"

    user_id = Column(String, primary_key=True)
    hardware_background = Column(SQLEnum(HardwareLevel), nullable=False)
    software_level = Column(SQLEnum(SoftwareLevel), nullable=False)
    learning_goal = Column(String, nullable=False)
    difficulty_preference = Column(String, default='intermediate')
    completed_chapters = Column(JSON, default=list)
    current_chapter = Column(String, default='week-01')

class TranslationCache(Base):
    __tablename__ = "translation_cache"

    id = Column(String, primary_key=True)  # f"{user_id}:{chapter_id}"
    user_id = Column(String, nullable=False)
    chapter_id = Column(String, nullable=False)
    source_lang = Column(String, default='en')
    target_lang = Column(String, default='ur')
    translated_content = Column(String, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
```

---

### Task 2.3: Qdrant Vector Store Setup

**File**: `backend/scripts/setup_qdrant.py`

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PayloadSchemaType
import os

def setup_qdrant():
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Create collection
    client.create_collection(
        collection_name="textbook_chapters",
        vectors_config=VectorParams(
            size=3072,  # text-embedding-3-large
            distance=Distance.COSINE
        )
    )

    # Create payload indexes
    client.create_payload_index(
        collection_name="textbook_chapters",
        field_name="chapter_id",
        field_schema=PayloadSchemaType.KEYWORD
    )

    client.create_payload_index(
        collection_name="textbook_chapters",
        field_name="difficulty",
        field_schema=PayloadSchemaType.KEYWORD
    )

    print("✅ Qdrant collection 'textbook_chapters' created")

if __name__ == "__main__":
    setup_qdrant()
```

---

### Task 2.4: Embedding Generation

**File**: `backend/scripts/generate_embeddings.py`

```python
import os
import re
from pathlib import Path
import frontmatter
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
import hashlib

def chunk_content(text: str, chunk_size: int = 1000, overlap: int = 200) -> list[str]:
    """Split text into overlapping chunks preserving code blocks."""
    # Simple paragraph-based chunking
    paragraphs = text.split('\n\n')
    chunks = []
    current = ""

    for para in paragraphs:
        if len(current) + len(para) < chunk_size:
            current += para + "\n\n"
        else:
            if current:
                chunks.append(current.strip())
            current = para + "\n\n"

    if current:
        chunks.append(current.strip())

    return chunks

async def embed_chapter(chapter_path: Path):
    """Generate and upload embeddings for a chapter."""
    # Parse frontmatter
    post = frontmatter.load(chapter_path)
    metadata = post.metadata
    content = post.content

    # Remove MDX imports
    content = re.sub(r"import .+ from .+;?\n", "", content)

    # Chunk
    chunks = chunk_content(content)

    openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    points = []

    for idx, chunk in enumerate(chunks):
        # Generate embedding
        response = openai_client.embeddings.create(
            model="text-embedding-3-large",
            input=chunk
        )
        vector = response.data[0].embedding

        # Create point ID
        point_id = hashlib.md5(f"{metadata['id']}-chunk-{idx}".encode()).hexdigest()

        # Create point
        point = PointStruct(
            id=point_id,
            vector=vector,
            payload={
                "content": chunk,
                "chapter_id": metadata['id'],
                "chapter_title": metadata.get('title', ''),
                "url": f"/chapters/{metadata['id']}",
                "chunk_index": idx,
                "total_chunks": len(chunks),
                "difficulty": "intermediate"  # Can be enhanced
            }
        )
        points.append(point)

    # Upload to Qdrant
    qdrant_client.upsert(
        collection_name="textbook_chapters",
        points=points
    )

    print(f"✅ Embedded {len(chunks)} chunks from {chapter_path.name}")

async def embed_all_chapters():
    """Embed all textbook chapters."""
    docs_dir = Path("../../docs")

    for md_file in sorted(docs_dir.glob("week-*.mdx")):
        print(f"Processing {md_file.name}...")
        await embed_chapter(md_file)

    print("\n🎉 All chapters embedded successfully!")
    print("Total chapters: 13")

if __name__ == "__main__":
    import asyncio
    asyncio.run(embed_all_chapters())
```

**Run**:
```bash
cd backend/scripts
python generate_embeddings.py
```

---

### Task 2.5: RAG Service

**File**: `backend/app/services/rag.py`

```python
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue
import os

class RAGService:
    def __init__(self):
        self.openai = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.qdrant = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )

    async def ask(
        self,
        question: str,
        selected_text: str = None,
        user_level: str = "intermediate",
        top_k: int = 5
    ):
        """Answer question using RAG."""

        # Augment query with selected text
        query = question
        if selected_text:
            query = f"Context: '{selected_text[:200]}...'\n\nQuestion: {question}"

        # Generate query embedding
        embedding_response = self.openai.embeddings.create(
            model="text-embedding-3-large",
            input=query
        )
        query_vector = embedding_response.data[0].embedding

        # Search Qdrant
        search_results = self.qdrant.search(
            collection_name="textbook_chapters",
            query_vector=query_vector,
            limit=top_k,
            query_filter=Filter(
                must=[
                    FieldCondition(
                        key="difficulty",
                        match=MatchValue(value=user_level)
                    )
                ]
            ) if user_level != "all" else None
        )

        # Build context
        context_parts = []
        for result in search_results:
            context_parts.append(
                f"[{result.payload['chapter_title']}]\n{result.payload['content']}"
            )

        context = "\n\n---\n\n".join(context_parts)

        # Generate answer
        chat_response = self.openai.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "system",
                    "content": f"""You are an expert AI tutor for Physical AI and Humanoid Robotics.

User Level: {user_level}

Use the following context from the textbook to answer questions accurately.
Always cite sources by chapter title when providing information.

Context from textbook:
{context}
"""
                },
                {
                    "role": "user",
                    "content": question
                }
            ],
            temperature=0.7,
            max_tokens=800
        )

        answer = chat_response.choices[0].message.content

        # Extract citations
        citations = [
            {
                "chapter": result.payload['chapter_title'],
                "url": result.payload['url']
            }
            for result in search_results[:3]
        ]

        return {
            "answer": answer,
            "citations": citations,
            "sources_count": len(search_results)
        }
```

---

### Task 2.6: FastAPI Main App

**File**: `backend/app/main.py`

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routers import auth, chat, translate, personalization
from app.config import settings

app = FastAPI(
    title="Physical AI Textbook API",
    description="Backend for intelligent robotics education platform",
    version="1.0.0"
)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://piaic.github.io",
        "http://localhost:3000",
        settings.FRONTEND_URL
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Routers
app.include_router(auth.router, prefix="/api/auth", tags=["auth"])
app.include_router(chat.router, prefix="/api/chat", tags=["chat"])
app.include_router(translate.router, prefix="/api/translate", tags=["translation"])
app.include_router(personalization.router, prefix="/api/personalization", tags=["personalization"])

@app.get("/health")
async def health():
    return {"status": "healthy", "version": "1.0.0"}

@app.get("/")
async def root():
    return {
        "message": "Physical AI Textbook API",
        "docs": "/docs",
        "health": "/health"
    }
```

---

### Task 2.7: Chat Router

**File**: `backend/app/routers/chat.py`

```python
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from app.services.rag import RAGService

router = APIRouter()
rag_service = RAGService()

class ChatRequest(BaseModel):
    message: str
    selectedText: str = None
    userLevel: str = "intermediate"
    language: str = "en"

class ChatResponse(BaseModel):
    answer: str
    citations: list[dict]
    sourcesCount: int

@router.post("/", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """Handle RAG chat requests."""
    try:
        result = await rag_service.ask(
            question=request.message,
            selected_text=request.selectedText,
            user_level=request.userLevel
        )

        return ChatResponse(**result)

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

---

## Phase 3: Authentication (Week 3)

### Task 3.1: better-auth Setup

**Install**:
```bash
npm install better-auth @better-auth/react
```

**File**: `src/lib/auth.ts`

```typescript
import { createAuthClient } from "@better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_API_URL || "http://localhost:8000",
  basePath: "/api/auth"
});

export const {
  signIn,
  signUp,
  signOut,
  useSession
} = authClient;
```

---

### Task 3.2: Signup Form with Survey

**File**: `src/components/SignupForm.tsx`

```typescript
import React, { useState } from 'react';
import { signUp } from '@site/src/lib/auth';

export function SignupForm() {
  const [step, setStep] = useState(1); // 1: Credentials, 2: Survey
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    hardwareBackground: 'none',
    softwareLevel: 'beginner',
    learningGoal: ''
  });

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (step === 1) {
      setStep(2);
      return;
    }

    // Step 2: Submit with metadata
    try {
      await signUp.email({
        email: formData.email,
        password: formData.password,
        metadata: {
          hardwareBackground: formData.hardwareBackground,
          softwareLevel: formData.softwareLevel,
          learningGoal: formData.learningGoal
        }
      });

      // Redirect to dashboard
      window.location.href = '/chapters/week-01';
    } catch (error) {
      console.error('Signup failed:', error);
    }
  };

  return (
    <div className="max-w-md mx-auto p-6">
      <h2 className="text-2xl font-bold mb-6">
        {step === 1 ? 'Create Account' : 'Tell Us About Yourself'}
      </h2>

      <form onSubmit={handleSubmit}>
        {step === 1 ? (
          <>
            <input type="email" placeholder="Email" required />
            <input type="password" placeholder="Password" required />
            <button type="submit">Continue →</button>
          </>
        ) : (
          <>
            <label>Hardware Experience:</label>
            <select name="hardwareBackground" required>
              <option value="none">None</option>
              <option value="arduino">Arduino/Raspberry Pi</option>
              <option value="ros">ROS/ROS 2</option>
              <option value="industrial">Industrial Robotics</option>
            </select>

            <label>Software Level:</label>
            <select name="softwareLevel" required>
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>

            <label>Learning Goal:</label>
            <textarea name="learningGoal" placeholder="What do you want to achieve?" required />

            <button type="submit">Start Learning 🚀</button>
          </>
        )}
      </form>
    </div>
  );
}
```

---

## Phase 4: Translation Service (Week 3)

### Task 4.1: Translation Router

**File**: `backend/app/routers/translate.py`

```python
from fastapi import APIRouter
from pydantic import BaseModel
from google.cloud import translate_v2 as translate
from app.db.neon import get_db
from app.models.user import TranslationCache

router = APIRouter()
translator = translate.Client()

class TranslateRequest(BaseModel):
    userId: str
    chapterId: str
    content: str
    targetLang: str = "ur"

@router.get("/cached")
async def get_cached_translation(userId: str, chapterId: str):
    """Check if translation exists in cache."""
    db = next(get_db())
    cache_key = f"{userId}:{chapterId}"

    cached = db.query(TranslationCache).filter(
        TranslationCache.id == cache_key
    ).first()

    if cached:
        return {"content": cached.translated_content, "cached": True}
    else:
        return {"cached": False}, 404

@router.post("/")
async def translate_chapter(request: TranslateRequest):
    """Translate chapter and cache result."""
    # Translate with Google
    result = translator.translate(
        request.content,
        target_language=request.targetLang,
        source_language="en"
    )

    translated = result['translatedText']

    # Cache in database
    db = next(get_db())
    cache_key = f"{request.userId}:{request.chapterId}"

    cache_entry = TranslationCache(
        id=cache_key,
        user_id=request.userId,
        chapter_id=request.chapterId,
        target_lang=request.targetLang,
        translated_content=translated
    )

    db.merge(cache_entry)  # Insert or update
    db.commit()

    return {
        "translatedContent": translated,
        "cached": False,
        "chapterId": request.chapterId
    }
```

---

## Phase 5: Claude Subagents (Week 4)

### Task 5.1: Create Subagent Commands

**Files to create in** `.claude/commands/`:

**1. `sp.ros2-lab-gen.md`**:
```markdown
Generate ROS 2 Python node templates for robotics labs.

When user requests a ROS 2 node (e.g., "Create joint state publisher"), generate:

1. Complete Python file with:
   - Proper imports (rclpy, message types)
   - Class inheriting from Node
   - Publishers/subscribers/services as needed
   - Callback methods with docstrings
   - Error handling and logging
   - main() function with rclpy.init/spin/shutdown

2. Docstring with:
   - Description
   - ROS 2 topics/services
   - Dependencies
   - Citation if using known algorithm

3. ROS 2 Humble compatibility
4. PEP 8 formatting

Example output for "joint state publisher":
- Complete `joint_state_publisher.py` file
- Publishes to `/joint_states` topic
- 50 Hz update rate
- Simulated sinusoidal motion
```

**2. `sp.gazebo-scene.md`**:
```markdown
Generate Gazebo simulation environments (URDF, SDF, launch files).

When user describes environment (e.g., "Kitchen with table and mug"):

1. Generate `.sdf` world file with:
   - Physics configuration (ODE/Bullet)
   - Ground plane
   - Lighting
   - Models (furniture, objects)
   - Proper collision/visual geometries

2. If robot needed, generate `.urdf.xacro` with:
   - Links and joints
   - Gazebo plugins (sensors, control)
   - Material properties

3. Generate `.launch.py` file to spawn everything

4. Add comments citing Gazebo documentation
```

**3. `sp.isaac-pipeline.md`**:
```markdown
Generate NVIDIA Isaac ROS perception pipeline configurations.

When user specifies perception task (e.g., "Object detection with camera"):

1. Generate YAML config for Isaac ROS nodes
2. Include:
   - Camera parameters
   - DNN model path (YOLO, SegNet, etc.)
   - Input/output topics
   - Performance settings (batch size, GPU)

3. Generate launch file
4. Add README with setup instructions
```

**4. `sp.vla-planner.md`**:
```markdown
Generate LLM-to-ROS action sequence planners.

When user describes VLA system (e.g., "Voice command to robot actions"):

1. Generate Python node that:
   - Subscribes to task commands (text or voice)
   - Calls OpenAI API with prompt template
   - Parses LLM response to action primitives
   - Publishes action sequence

2. Include:
   - Prompt engineering for task decomposition
   - JSON parsing and validation
   - ROS 2 action/service clients
   - Error handling

3. Provide example commands and expected outputs
```

**5. `sp.quiz-maker.md`**:
```markdown
Auto-generate multiple-choice questions from chapter content.

When user specifies chapter (e.g., "/sp.quiz-maker week-11"):

1. Read chapter MDX file
2. Extract key concepts, code examples, equations
3. Generate 10 MCQs with:
   - 1 correct answer
   - 3 plausible distractors
   - Explanation for correct answer
   - Difficulty level (beginner/intermediate/advanced)

4. Output as JSON:
{
  "chapterId": "week-11",
  "questions": [
    {
      "id": 1,
      "question": "What does ZMP stand for?",
      "options": ["A", "B", "C", "D"],
      "correct": "A",
      "explanation": "...",
      "difficulty": "beginner"
    }
  ]
}

5. Optionally generate Markdown quiz for embedding in chapter
```

---

## Phase 6: CI/CD (Week 4)

### Task 6.1: GitHub Actions Workflow

**File**: `.github/workflows/deploy.yml`

```yaml
name: Deploy Textbook

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

permissions:
  contents: write
  pages: write
  id-token: write

jobs:
  build:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: '18'
          cache: 'npm'

      - name: Install dependencies
        run: npm ci

      - name: Build Docusaurus
        run: npm run build
        env:
          NODE_ENV: production
          REACT_APP_API_URL: ${{ secrets.API_URL }}

      - name: Upload artifact
        uses: actions/upload-pages-artifact@v2
        with:
          path: build

  deploy:
    needs: build
    if: github.ref == 'refs/heads/main'
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}

    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v2
```

---

## Environment Variables

### Frontend (`.env.production`)
```env
REACT_APP_API_URL=https://textbook-api.railway.app
REACT_APP_GITHUB_CLIENT_ID=xxx
REACT_APP_GOOGLE_CLIENT_ID=xxx
```

### Backend (`.env`)
```env
# Database
DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/textbook

# Vector Store
QDRANT_URL=https://xxx.qdrant.io:6333
QDRANT_API_KEY=xxx

# OpenAI
OPENAI_API_KEY=sk-xxx

# Google Translate
GOOGLE_APPLICATION_CREDENTIALS=/path/to/service-account.json

# Auth
JWT_SECRET=xxx (generate with: openssl rand -hex 32)
GITHUB_CLIENT_ID=xxx
GITHUB_CLIENT_SECRET=xxx
GOOGLE_CLIENT_ID=xxx
GOOGLE_CLIENT_SECRET=xxx

# CORS
FRONTEND_URL=https://piaic.github.io/humanoid-ai-textbook
```

---

## Deployment Checklist

### Pre-Deployment
- [ ] Create Neon PostgreSQL database (free tier)
- [ ] Create Qdrant Cloud collection (free tier)
- [ ] Set up Google Cloud project for Translate API
- [ ] Create OAuth apps (GitHub, Google)
- [ ] Generate JWT secret

### Backend Deployment
- [ ] Deploy FastAPI to Railway or Fly.io
- [ ] Run database migrations
- [ ] Generate and upload embeddings to Qdrant
- [ ] Test `/health`, `/api/chat`, `/api/translate` endpoints
- [ ] Configure CORS for GitHub Pages domain

### Frontend Deployment
- [ ] Update `docusaurus.config.js` with correct URL/baseURL
- [ ] Add environment variables to GitHub Secrets
- [ ] Push to main branch → triggers deployment
- [ ] Verify at https://piaic.github.io/humanoid-ai-textbook

### Post-Deployment Testing
- [ ] Signup flow works
- [ ] Personalization toggles difficulty
- [ ] Translation caches properly
- [ ] RAG chatbot returns relevant answers with citations
- [ ] OAuth login (GitHub, Google)
- [ ] Mobile responsive

---

## Cost Estimates (Monthly)

| Service | Tier | Est. Cost |
|---------|------|-----------|
| GitHub Pages | Free | $0 |
| Neon Postgres | Free (1GB) | $0 |
| Qdrant Cloud | Free (1GB) | $0 |
| OpenAI API | PAYG | $30-50 (100 users) |
| Google Translate | PAYG | $10-20 (with caching) |
| Railway/Fly.io | Hobby | $5-10 |
| **Total** | | **$45-80/month** |

---

## Success Criteria

- ✅ Textbook accessible at public URL
- ✅ All 13 chapters display correctly with sidebar navigation
- ✅ Personalization works (3 difficulty levels)
- ✅ Urdu translation works and caches
- ✅ RAG chatbot functional with <2s response time
- ✅ Authentication flow complete
- ✅ 5 Claude subagents defined and working
- ✅ Mobile responsive
- ✅ Performance: Lighthouse score >90

---

## Timeline: 4 Weeks

| Week | Focus | Deliverables |
|------|-------|--------------|
| 1 | Frontend | PersonalizedChapter, translation UI, enhanced RAG widget |
| 2 | Backend Core | FastAPI, Neon, Qdrant, RAG endpoint, embeddings |
| 3 | Auth & Translation | better-auth, signup survey, Google Translate service |
| 4 | Subagents & Deploy | 5 command files, CI/CD, production deployment, testing |

---

**Ready to implement? Let's start with Phase 1: Frontend Enhancement!** 🚀

