# Chat Auto-Save Testing Guide

## Quick Start Test (5 minutes)

### Prerequisites
- Backend running: `cd backend && uvicorn app.main:app --reload`
- Frontend running: `cd physical-ai-humanoid-textbook && npm start`

### Test Steps

#### Test 1: Basic Auto-Save
```bash
# Terminal 1: Backend logs
cd backend
uvicorn app.main:app --reload

# Terminal 2: Frontend
cd physical-ai-humanoid-textbook
npm start
```

1. Open http://localhost:3000
2. Navigate to any chapter
3. Send message: "What is this chapter about?"
4. Check backend console - should show conversation creation
5. Copy the conversation_id from the response

#### Test 2: Verify History via API
```bash
# Terminal 3: Test API
CONV_ID="<conversation_id_from_response>"

# Get the conversation
curl http://localhost:8000/api/conversations/$CONV_ID

# Should return:
# {
#   "conversation_id": "...",
#   "title": "New Conversation",
#   "messages": [
#     {"role": "user", "content": "What is this chapter about?", ...},
#     {"role": "assistant", "content": "...", ...}
#   ]
# }
```

#### Test 3: Clear Chat & Verify History Still Exists
1. In UI: Click the trash icon to clear chat
2. Messages disappear from UI
3. Run API call from Test 2 again
4. Messages still in database! ✅

#### Test 4: User Conversations (Authenticated)
```bash
# Send a message with user email
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is ZMP?",
    "user_email": "test@example.com",
    "userLevel": "intermediate",
    "language": "en"
  }'

# Get all conversations for this user
curl http://localhost:8000/api/conversations/user/test@example.com

# Should show all conversations for that user
```

## Detailed Test Cases

### TC1: First Message Creates Conversation
**Expected:** conversation_id generated and returned

```javascript
// Frontend
sendChatMessage({
  message: "Hello?",
  conversationId: undefined,  // undefined = new
  userEmail: "user@test.com"
})

// Backend should:
// 1. Generate UUID for conversation_id
// 2. Save user message
// 3. Generate RAG response
// 4. Save assistant message
// 5. Return conversation_id in response

// Frontend should:
// 1. Receive conversation_id
// 2. Store in state for next message
```

### TC2: Second Message Uses Same Conversation
**Expected:** Messages grouped under same conversation_id

```javascript
// Frontend (conversationId already in state)
sendChatMessage({
  message: "Tell me more?",
  conversationId: "abc-123",
  userEmail: "user@test.com"
})

// Backend should:
// 1. Find existing conversation "abc-123"
// 2. Append user message
// 3. Generate response
// 4. Append assistant message
// 5. Update message_count and updated_at
```

### TC3: New Conversation Button
**Expected:** conversationId reset, next message creates new conversation

```javascript
// User clicks "+" button
handleNewConversation()
// → conversationId = null

// User sends new message
sendChatMessage({
  message: "Different topic",
  conversationId: undefined,  // null/undefined again
  userEmail: "user@test.com"
})

// Backend should:
// 1. Generate NEW conversation_id
// 2. NOT append to old conversation
// 3. Create separate message group
```

### TC4: Clear Chat
**Expected:** UI clears, history preserved in DB

```javascript
// User clicks clear
handleClearChat()
// → messages = []
// → conversationId = null (if implemented)

// Messages disappear from UI
// But in database, conversation still exists
// GET /api/conversations/{old_id} still returns messages
```

### TC5: User History Retrieval
**Expected:** All conversations for authenticated user returned

```bash
# After sending 3 messages with user_email "alice@test.com"
curl http://localhost:8000/api/conversations/user/alice@test.com

# Response should contain:
# [
#   {
#     "conversation_id": "conv-1",
#     "title": "New Conversation",
#     "message_count": 6,  // 3 user + 3 assistant
#     "created_at": "2024-12-01T...",
#     "updated_at": "2024-12-01T..."
#   }
# ]
```

### TC6: Citations Preserved
**Expected:** Citations saved and retrieved with messages

```bash
# Send question that gets citations
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is ZMP stability criterion?",
    "conversation_id": "test-conv",
    "userLevel": "intermediate",
    "language": "en"
  }'

# Get conversation
curl http://localhost:8000/api/conversations/test-conv

# Response messages should include citations:
# "messages": [
#   {
#     "role": "assistant",
#     "citations": [
#       {"chapter": "Week 11", "url": "/chapters/week-11", ...}
#     ]
#   }
# ]
```

## Debug Checklist

If tests fail, check:

- [ ] Backend is running: `curl http://localhost:8000/health`
- [ ] Chat endpoint responds: `curl -X POST http://localhost:8000/api/chat/`
- [ ] Conversations endpoint works: `curl http://localhost:8000/api/conversations/user/test@example.com`
- [ ] Backend logs show message saves
- [ ] Frontend console shows conversation_id in response
- [ ] Check browser DevTools Network tab for API responses
- [ ] Python syntax: `python -m py_compile backend/app/routers/chat.py`
- [ ] TypeScript: `npm run build` in frontend (if needed)

## Expected Console Logs

### Backend (Should see):
```
INFO:     POST /api/chat/ HTTP/1.1" 200
# Auto-save happens here silently
```

### Frontend (DevTools Network):
```
POST /api/chat/
Response:
{
  "answer": "...",
  "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
  ...
}
```

## Success Indicators

- ✅ API returns conversation_id in response
- ✅ Same conversation_id used for subsequent messages
- ✅ GET /api/conversations/{id} returns all messages
- ✅ Clear chat removes UI messages but DB still has them
- ✅ GET /api/conversations/user/{email} returns all user conversations
- ✅ Citations appear in conversation history

## Troubleshooting

### Backend Import Error
```
ImportError: cannot import name 'get_conversations' from 'app.storage'
```
**Fix:** Ensure `app/storage.py` exists and is in correct location

### Messages Not Being Saved
```
# Check if _save_message_to_conversation is being called
# Add print statements to backend/app/routers/chat.py
```

### Conversation Not Found
```
# Make sure you're using the exact conversation_id from response
# Check if it's a string format issue
```

### Frontend Not Showing conversation_id
```
# Verify apiService.ts includes conversation_id in response parsing
# Check TypeScript ChatResponse interface has conversation_id field
```

## Performance Test

After verifying basic functionality:

```javascript
// Send 10 messages in rapid succession
for (let i = 0; i < 10; i++) {
  sendChatMessage({
    message: `Test message ${i}`,
    conversationId: "perf-test-conv",
    userEmail: "perf@test.com"
  });
}

// Then retrieve
curl http://localhost:8000/api/conversations/perf-test-conv

// Should return all 20 messages (10 user + 10 assistant)
// Response time should be < 100ms
```

## Integration Test

Testing with authenticated user (if auth is implemented):

```javascript
// 1. Register user
POST /api/auth/register
{
  "email": "testuser@example.com",
  "password": "Test123!@",
  "name": "Test User"
}

// 2. Login
POST /api/auth/login
{
  "email": "testuser@example.com",
  "password": "Test123!@"
}
// Get token

// 3. Send message with token
POST /api/chat
Headers: Authorization: Bearer {token}
{
  "message": "Hello",
  "conversation_id": "auth-test"
}

// 4. Get user conversations
GET /api/conversations/user/testuser@example.com

// Should show conversation saved with authenticated user
```

---

**Run these tests to verify the auto-save feature is working end-to-end!**
