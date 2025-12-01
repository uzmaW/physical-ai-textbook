# Translation Feature - Troubleshooting Guide

## Issue: "No content to translate" Error

**Error Message**: Alert appears saying "Error: No content to translate. The chapter content could not be extracted."

### Root Cause
The text extraction function fails to get content from the MDX-rendered chapter. This happens when:
1. React children extraction fails (content is in DOM, not props)
2. DOM querySelector doesn't find `.chapter-content` element
3. Content is empty or whitespace-only

### Solution Steps

#### Step 1: Check Browser Console
Open browser Developer Tools (F12) and go to **Console** tab.

Look for logs like:
```
[Translation] Extracted content length: 0 chars
[Translation] First 200 chars: "..."
[Translation] ✗ Failed to extract content from chapter
```

If content length is 0, the extraction failed.

#### Step 2: Verify Chapter is Loaded
In the **Console**, run:
```javascript
// Check if chapter content div exists
document.querySelector('.chapter-content')

// Expected output: <div class="chapter-content ...">...</div>
// If null, the DOM element doesn't exist yet
```

#### Step 3: Check Prose Content
```javascript
// Check if prose div exists
document.querySelector('.chapter-content .prose')

// Check text content
document.querySelector('.chapter-content')?.textContent?.length
```

#### Step 4: Manual Content Extraction
Try manually extracting content:
```javascript
// Get chapter content
const el = document.querySelector('.chapter-content .prose') || 
           document.querySelector('.chapter-content');
           
if (el) {
  const clone = el.cloneNode(true);
  clone.querySelectorAll('button, [role="button"]').forEach(b => b.remove());
  console.log('Content:', clone.textContent.substring(0, 200));
}
```

---

## Common Scenarios & Fixes

### Scenario 1: Chapter Content is Null

**Console Output**:
```
document.querySelector('.chapter-content') → null
```

**Cause**: Chapter hasn't rendered yet

**Fix**:
- Wait for page to fully load before clicking translate
- The chapter content might still be loading
- Try clicking translate again after 2-3 seconds

### Scenario 2: Content Exists but Empty

**Console Output**:
```
[Translation] Extracted content length: 0 chars
document.querySelector('.chapter-content').textContent.length → 0
```

**Cause**: 
- Prose div is rendered but empty
- Content is in a different structure than expected

**Fix**:
- Open DevTools Inspector
- Click on the chapter-content div to inspect structure
- Check if content is nested differently
- Update selector in code if structure differs

### Scenario 3: Content Extracted but Still Showing Error

**Console Output**:
```
[Translation] Extracted content length: 2500 chars
[Translation] First 200 chars: "Introduction to Humanoid Robots..."
[Translation] ✗ Failed to extract content from chapter
```

**Cause**: Content is being extracted after, but error thrown before

**Fix**:
- This indicates a race condition
- Content loads AFTER translation function checks
- Solution: Add small delay or use MutationObserver

### Scenario 4: Translation Sends But Gets Error

**Console Output**:
```
[Translation] Starting translation for chapter: week-01
[Translation] Content length: 2500 chars
[Translation] POST to http://localhost:8000/api/translate/
[Translation] ✗ Failed with status 500
```

**Cause**: Backend error

**Fix**: Check backend logs
```bash
# Terminal: Watch backend logs
tail -f nohup.out

# Look for errors like:
# ERROR: Could not generate embedding (OpenAI): invalid API key
# ERROR: Translation failed: ...
```

---

## Testing the Feature

### Test 1: Manual DOM Check
```bash
# Open a chapter in browser
# Open DevTools Console (F12)
# Run:
document.querySelector('.chapter-content')
// Should return the chapter div, not null
```

### Test 2: Simulate Translation Flow
```javascript
// In browser console:
const content = document.querySelector('.chapter-content .prose')?.textContent || '';
console.log('Content length:', content.length);
console.log('First 300 chars:', content.substring(0, 300));

// If length > 0, extraction works
```

### Test 3: API Test (Backend)
```bash
# Test backend endpoint directly
curl -X POST http://localhost:8000/api/translate/ \
  -H "Content-Type: application/json" \
  -d '{
    "userId": "test-user",
    "chapterId": "week-01",
    "content": "What is a humanoid robot? It is a robot that mimics human form.",
    "targetLang": "ur"
  }'

# Should return translated content with indexed:true
```

---

## Debug Mode: Enhanced Logging

Add this to PersonalizedChapter.tsx for detailed debugging:

```typescript
// Add this inside handleTranslate function
const debugExtractContent = () => {
  const proseEl = document.querySelector('.chapter-content .prose');
  const chapterEl = document.querySelector('.chapter-content');
  
  console.log('[DEBUG] Prose element:', proseEl);
  console.log('[DEBUG] Chapter element:', chapterEl);
  
  if (chapterEl) {
    console.log('[DEBUG] Chapter innerHTML length:', chapterEl.innerHTML.length);
    console.log('[DEBUG] Chapter textContent length:', chapterEl.textContent?.length);
    console.log('[DEBUG] Chapter first 500 chars:', chapterEl.textContent?.substring(0, 500));
  }
};

// Call before extracting:
debugExtractContent();
const textContent = extractTextContent(children);
```

---

## Environment Checklist

- [ ] Backend running on port 8000: `curl http://localhost:8000/health`
- [ ] Chapter page fully loaded (wait 2-3s)
- [ ] Transformers library installed: `pip list | grep transformers`
- [ ] Helsinki-NLP model downloaded: Check `~/.cache/huggingface/hub/`
- [ ] OPENAI_API_KEY set: `echo $OPENAI_API_KEY`
- [ ] Database connected: Check backend logs

---

## Performance Check

### Should Be Fast:
- **First translation**: 5-10 seconds (model loading)
- **Cached translation**: 1-2 seconds
- **Content extraction**: <100ms

### If Slow:
- First translation taking >15s: Model might be downloading
- Extraction taking >1s: DOM is large or extraction inefficient
- No response for 30s+: Backend hanging or API timeout

---

## Solution Summary

| Issue | Check | Fix |
|-------|-------|-----|
| "No content to translate" | F12 Console → Log shows 0 chars | Ensure chapter is fully loaded |
| Content not found | `querySelector('.chapter-content')` returns null | Wait for page render, check MDX |
| API returns 500 error | Backend logs | Check OpenAI API key, Qdrant connection |
| Works first time, then fails | Check console for race condition | Add retry or delay mechanism |
| Translations appear but not in Urdu | Check `indexing` status in response | Verify OpenAI key for embedding |

---

## Contact Support

If issue persists after trying above steps:

1. **Collect Info**:
   - Browser console log (copy all [Translation] messages)
   - Backend error log (from nohup.out)
   - Screenshot of the chapter that fails

2. **Report Issue**:
   - Check CLAUDE.md for team contact
   - Include console output + backend logs
   - Specify browser and OS version

3. **Workaround**:
   - Try different browser (Firefox, Chrome, Safari)
   - Try different chapter to isolate issue
   - Refresh page and retry
