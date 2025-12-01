# Translation Feature - Debug & Testing Guide

## Overview

Translation feature allows users to:
1. Click "ترجمہ" button to translate chapter content to Urdu
2. Translation is cached in database for fast re-retrieval
3. Translated content is indexed in Qdrant for RAG search

**Status**: ✅ Fixed - Async/await and Qdrant indexing now working

---

## Architecture

```
Frontend (PersonalizedChapter.tsx)
    ↓ (click "ترجمہ" button)
    ↓ Extract chapter text content
    ↓ POST /api/translate/
    ↓
Backend (translate.py)
    ↓ Check if translator model loaded
    ↓ Split text into sentences
    ↓ Translate each sentence (Helsinki-NLP model)
    ↓ Save to DB (TranslationCache)
    ↓ Index in Qdrant (generate embedding + upsert)
    ↓
Qdrant Vector Database
    ↓ Store translated content with metadata
    ↓ Now searchable via RAG queries

Frontend receives:
{
    "translatedContent": "ترجمہ...",
    "cached": false,
    "chapterId": "week-01",
    "characterCount": 1234,
    "indexed": true  ← Shows if Qdrant indexing succeeded
}
```

---

## Common Issues & Fixes

### Issue 1: "Translation failed" button, no content shown

**Symptoms**:
- Click "ترجمہ" → "⏳ Translating..." spinner → error alert
- No translated text appears

**Root Causes**:
1. Backend not running → Check backend is running on 8000
2. Model not loaded → Check "Helsinki-NLP/opus-mt-en-ur" is downloading
3. Transformers not installed → Need to install transformers, torch
4. Empty content → Chapter text extraction failing

**Debug Steps**:

```bash
# 1. Check backend is running
curl http://localhost:8000/health
# Should return 200 with healthy status

# 2. Check translate endpoint
curl -X POST http://localhost:8000/api/translate/ \
  -H "Content-Type: application/json" \
  -d '{
    "userId": "test-user",
    "chapterId": "week-01",
    "content": "What is a humanoid robot?",
    "targetLang": "ur"
  }'

# 3. Check frontend console (F12 → Console tab)
# Look for logs like:
# [Translation] Starting translation for chapter: week-01
# [Translation] API Base: http://localhost:8000
# [Translation] Checking cache: http://localhost:8000/api/translate/cached?userId=...
```

### Issue 2: Browser console shows error, but no alert

**Check**: F12 → Console tab

**Common errors**:
```
SyntaxError: Unexpected token < in JSON at position 0
→ Backend returned HTML error page (likely 500)

TypeError: Cannot read property 'translatedContent' of undefined
→ Response is 404 (translation not found)

CORS error: Access-Control-Allow-Origin
→ Backend CORS not configured for frontend URL
```

### Issue 3: Translation works but not indexed in Qdrant

**Symptoms**:
- Translated content shows
- But `"indexed": false` in response
- Or console shows: "⚠️  Warning: Could not index translated content in Qdrant"

**Root Causes**:
1. OpenAI API key not set (embedding generation fails)
2. Qdrant server unreachable
3. Collection doesn't exist in Qdrant

**Debug Steps**:

```bash
# Check OpenAI key
echo $OPENAI_API_KEY
# Should print your API key (starts with sk-)

# Check Qdrant connection
curl https://468ff8ad-822f-4a3a-9f5c-b8953bbadc70.us-east-1-1.aws.cloud.qdrant.io:6333/health
# Should return 200

# Check collection exists
curl -H "api-key: YOUR_QDRANT_API_KEY" \
  https://468ff8ad-822f-4a3a-9f5c-b8953bbadc70.us-east-1-1.aws.cloud.qdrant.io:6333/collections/textbook_chapters
```

### Issue 4: Cache not working

**Symptoms**:
- First translation works
- Click English button to switch back
- Click ترجمہ again → translations it again instead of loading from cache

**Root Causes**:
1. Database not connected
2. TranslationCache table doesn't exist
3. Cache key mismatch

**Debug**:

```bash
# Check database migrations ran
psql $DATABASE_URL -c "SELECT * FROM translation_cache LIMIT 5;"

# Check cache key format
# Key should be: "{userId}:{chapterId}"
# Example: "user-123:week-01-intro"

# Verify cache was saved
SELECT * FROM translation_cache 
WHERE chapter_id = 'week-01' 
LIMIT 1;
```

---

## Testing Checklist

### Manual Testing

- [ ] **Happy Path**: Click ترجمہ → See Urdu text in 5-10 seconds
- [ ] **Cache Hit**: Click English → ترجمہ again → Appears instantly
- [ ] **Error Handling**: Kill backend → Click ترجمہ → See error alert
- [ ] **Different Chapters**: Try translating multiple chapters → All work
- [ ] **Long Content**: Translate long chapter → Works without timeout
- [ ] **Console Logs**: F12 → Console shows proper [Translation] logs
- [ ] **Indexed Flag**: Response shows `"indexed": true` in console

### API Testing

```bash
# Test 1: Fresh translation
curl -X POST http://localhost:8000/api/translate/ \
  -H "Content-Type: application/json" \
  -d '{
    "userId": "test-1",
    "chapterId": "test-chapter",
    "content": "Introduction to robotics and artificial intelligence",
    "targetLang": "ur"
  }' | jq

# Expected response:
# {
#   "translatedContent": "روبوٹکس اور مصنوعی ذہانت کا تعارف",
#   "cached": false,
#   "chapterId": "test-chapter",
#   "characterCount": 50,
#   "indexed": true
# }

# Test 2: Cached translation
curl 'http://localhost:8000/api/translate/cached?userId=test-1&chapterId=test-chapter' | jq

# Expected response:
# {
#   "content": "روبوٹکس اور مصنوعی ذہانت کا تعارف",
#   "cached": true,
#   "chapterId": "test-chapter",
#   "indexed": true,
#   "createdAt": "2025-01-15T..."
# }
```

---

## Performance Metrics

### Expected Latencies

```
First Translation (cold start):
├─ Model loading (Helsinki-NLP):  ~3-5s (first time only)
├─ Sentence tokenization:          <100ms
├─ Translation per sentence:       100-500ms (depending on length)
├─ Embedding generation (OpenAI):  500-1000ms
├─ Qdrant upsert:                  200-500ms
└─ Total:                           ~4-7s first time

Cached Translation:
└─ Total:                           ~100-200ms

Cache Hit Rate Target:
└─ >80% of subsequent requests should hit cache
```

### Monitoring

Add these logs to monitor performance:

```python
import time

start = time.time()

# ... translation work ...

elapsed = time.time() - start
print(f"Translation completed in {elapsed:.2f}s")
```

---

## Architecture Decisions

### Why Helsinki-NLP (not Google/Azure)?
- Free (no API key required)
- Works offline (model cached locally)
- Good accuracy for EN→UR
- ~400MB model size (manageable)

### Why Qdrant?
- Vector database for semantic search
- Allow RAG to find relevant sections by topic
- Enables search across languages (EN & UR)
- Works with OpenAI embeddings

### Why local caching?
- Reduce API calls to translation model
- Instant retrieval for repeated requests
- Database-backed (survives restarts)
- Cache key: `{userId}:{chapterId}`

---

## Future Improvements

1. **Performance**:
   - [ ] Pre-load Helsinki-NLP model on startup (3-5s savings)
   - [ ] Cache embeddings locally (reduce OpenAI calls)
   - [ ] Batch translate multiple chapters

2. **Quality**:
   - [ ] Use larger Whisper model (medium/large) for better accuracy
   - [ ] Human review of translations
   - [ ] A/B test different models

3. **Features**:
   - [ ] Save user's favorite chapters
   - [ ] Enable translating to other languages
   - [ ] Show confidence scores for translations
   - [ ] Allow users to suggest corrections

---

## Support

**Debug workflow**:
1. Check browser console (F12) for [Translation] logs
2. Check backend logs for errors/warnings
3. Verify environment variables (.env)
4. Run API tests from "Testing Checklist"
5. Check Qdrant/DB connectivity

**Contact**: Check CLAUDE.md for team contact info
