# Chat History Auto-Save Specification

## Requirement
Every message sent in the chat should be saved to the database immediately after being received. History must be preserved before clearing the screen.

## Current Issue
1. Chat endpoint (`/api/chat`) generates answers but doesn't persist messages
2. Frontend doesn't call message save API after getting response
3. Clear chat button doesn't save history before clearing

## Implementation Plan

### Backend Changes

#### 1. Modify `/api/chat` endpoint (backend/app/routers/chat.py)
- Accept `conversation_id` and `user_email` in ChatRequest
- After generating response, automatically save both user message and assistant response
- Return `conversation_id` in response for frontend to use

```python
class ChatRequest(BaseModel):
    message: str
    conversation_id: Optional[str] = None
    user_email: Optional[str] = None  # For authenticated users
    selectedText: Optional[str] = None
    userLevel: str = "intermediate"
    language: str = "en"

class ChatResponse(BaseModel):
    answer: str
    citations: List[Citation]
    sourcesCount: int
    model: str
    conversation_id: str  # Added
```

#### 2. Add save_message helper in chat.py
```python
async def save_conversation_messages(
    conversation_id: str, 
    user_message: str, 
    assistant_message: str,
    user_email: Optional[str],
    citations: List[Citation]
):
    """Save both user and assistant messages to conversation history"""
    # Create conversation if needed
    # Save user message
    # Save assistant message
```

### Frontend Changes

#### 1. Modify RAGChatWidget.tsx sendMessage()
- Generate or reuse conversation_id
- Pass conversation_id and user_email to API
- Store conversation_id in state for current session

```typescript
const [conversationId, setConversationId] = useState<string | null>(null);

const sendMessage = async (messageText?: string, context?: string) => {
  // ... existing code ...
  
  // Create conversation if needed
  if (!conversationId) {
    const newConvId = await createConversation(userProfile?.email);
    setConversationId(newConvId);
  }

  const data = await sendChatMessage({
    message: textToSend,
    conversationId: conversationId,
    userEmail: userProfile?.email,
    // ... other fields ...
  });
  
  // Message is now auto-saved by backend
};
```

#### 2. Modify handleClearChat()
```typescript
const handleClearChat = async () => {
  if (window.confirm('Clear this conversation?')) {
    // History is already saved in DB from each message
    // Just reset UI
    handleNewConversation();
    setConversationId(null);  // Start new conversation
  }
};
```

#### 3. Update apiService.ts
- Add conversation_id and user_email to sendChatMessage call
- Add createConversation API call

### Database/Storage

#### If using PostgreSQL (production):
- Messages already have proper schema in `Conversation` and `Message` models
- Just need to wire up the save calls

#### Current in-memory (conversations.py):
- Already has CONVERSATIONS and MESSAGES storage
- Just need backend to call the add_message endpoint after getting RAG response

## Flow

```
User sends message
  ↓
Frontend calls /api/chat with:
  - message
  - conversation_id (or null for new)
  - user_email
  - selectedText
  ↓
Backend processes with RAG
  ↓
Backend automatically saves:
  - User message to conversation history
  - Assistant response to conversation history
  ↓
Backend returns response + conversation_id
  ↓
Frontend receives response + conversation_id
  ↓
Frontend displays message (already saved in DB)
  ↓
User clears chat (history preserved in DB)
```

## Files to Modify

### Backend
- [ ] `backend/app/routers/chat.py` - Add conversation saving logic
- [ ] `backend/app/models/conversation.py` - Ensure models support auto-save
- [ ] `backend/app/main.py` - Ensure routers registered

### Frontend  
- [ ] `src/components/RAGChatWidget.tsx` - Add conversation_id state and management
- [ ] `src/services/apiService.ts` - Update sendChatMessage signature
- [ ] `src/services/chatService.ts` - Add createConversation method

## Success Criteria
- ✅ Every message auto-saves immediately after being received
- ✅ Clear button doesn't delete history (only clears UI)
- ✅ History visible after logout/login if user authenticated
- ✅ Anonymous users' messages saved temporarily (until logout)
- ✅ Conversation ID preserved during session
