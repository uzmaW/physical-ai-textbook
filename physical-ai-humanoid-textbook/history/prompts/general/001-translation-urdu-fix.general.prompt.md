---
ID: "001"
TITLE: "Translation to Urdu - Fix Vector DB Indexing"
STAGE: "general"
DATE_ISO: "2025-01-15"
SURFACE: "agent"
MODEL: "claude-opus"
FEATURE: "translation-urdu"
BRANCH: "main"
USER: "user"
COMMAND: "Fix Urdu translation showing English text, store in vector DB and index"
LABELS: ["translation", "urdu", "vector-db", "qdrant", "rag", "i18n"]
LINKS:
  SPEC: "null"
  TICKET: "null"
  ADR: "null"
  PR: "null"
FILES_YAML:
  - "/backend/app/routers/translate.py"
  - "/backend/app/services/rag.py"
  - "/TRANSLATION_FIX.md"
TESTS_YAML: []
---

## User Request
Translation to Urdu shows English text instead of translated Urdu text. The user wants:
1. Translation to display Urdu text properly
2. Translated text stored in vector DB (Qdrant)
3. Translated text indexed for RAG search

## Analysis & Solution

### Root Cause
1. **Frontend**: PersonalizedChapter component correctly calls backend and displays translation
2. **Backend**: Translation router was caching in PostgreSQL but NOT indexing in Qdrant
3. **RAG**: Service had no language filtering for multilingual search

### Implementation

#### 1. Enhanced Translation Router (`backend/app/routers/translate.py`)
- Added Qdrant indexing capabilities
- Created `index_translated_content()` async function
- Automatically indexes translated text when translation occurs
- Also indexes cached translations on retrieval

#### 2. Updated RAG Service (`backend/app/services/rag.py`)
- Added `language` parameter to `search_context()`
- Filters results by language when specified
- Includes language metadata in returned results
- Passes language through the entire pipeline

### Key Changes
- **POST `/api/translate/`**: Calls `index_translated_content()` after translation
- **GET `/api/translate/cached`**: Calls `index_translated_content()` after retrieval
- **RAG Search**: Now supports language-aware filtering (`language="ur"`)

## Outcome
✅ Translations are now:
1. Displayed correctly in Urdu (already working)
2. Indexed in Qdrant vector DB with language metadata
3. Searchable via RAG with language filtering
4. Cached for performance

The fix ensures translated content is discoverable in semantic searches, enabling the RAG system to provide Urdu responses when users prefer that language.
