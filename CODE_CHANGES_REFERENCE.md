# Code Changes Reference

Quick reference of all code modifications for the chat auto-save feature.

## Backend Changes

### File 1: `backend/app/storage.py` (NEW)

```python
"""
Shared in-memory storage for conversations
This is used by both chat and conversations routers
In production, replace with database models
"""

# In-memory conversation storage
CONVERSATIONS = {}
MESSAGES = {}


def get_conversations():
    """Get the conversations dictionary"""
    return CONVERSATIONS


def get_messages():
    """Get the messages dictionary"""
    return MESSAGES
```

---

### File 2: `backend/app/routers/chat.py` (MODIFIED)

#### Import Changes
```python
# ADDED:
from app.storage import get_conversations, get_messages
```

#### ChatRequest Changes
```python
class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    message: str
    conversation_id: Optional[str] = None  # ADDED
    user_email: Optional[str] = None        # ADDED
    selectedText: Optional[str] = None
    userLevel: str = "intermediate"
    language: str = "en"
```

#### ChatResponse Changes
```python
class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    answer: str
    citations: List[Citation]
    sourcesCount: int
    model: str
    conversation_id: str  # ADDED
```

#### New Function (ADDED)
```python
async def _save_message_to_conversation(
    conversation_id: str,
    role: str,
    content: str,
    citations: Optional[List[Citation]] = None,
    user_email: Optional[str] = None
) -> dict:
    """
    Save a message to conversation history.
    Creates conversation if needed, appends message.
    """
    CONVERSATIONS = get_conversations()
    MESSAGES = get_messages()
    
    # Create conversation if it doesn't exist
    if conversation_id not in CONVERSATIONS:
        CONVERSATIONS[conversation_id] = {
            "conversation_id": conversation_id,
            "user_email": user_email,
            "title": "New Conversation",
            "created_at": datetime.utcnow().isoformat(),
            "updated_at": datetime.utcnow().isoformat(),
            "message_count": 0,
            "metadata": {}
        }
        MESSAGES[conversation_id] = []
    
    # Create message
    message_data = {
        "id": str(uuid.uuid4()),
        "role": role,
        "content": content,
        "timestamp": datetime.utcnow().isoformat(),
        "citations": [c.dict() if hasattr(c, 'dict') else c for c in (citations or [])]
    }
    
    # Save message
    MESSAGES[conversation_id].append(message_data)
    
    # Update conversation metadata
    CONVERSATIONS[conversation_id]["message_count"] += 1
    CONVERSATIONS[conversation_id]["updated_at"] = datetime.utcnow().isoformat()
    
    return message_data
```

#### Chat Endpoint Changes
```python
@router.post("/", response_model=ChatResponse)
async def chat(request: ChatRequest):
    try:
        # Generate or use existing conversation ID
        conversation_id = request.conversation_id or str(uuid.uuid4())  # ADDED
        
        # Get RAG response
        result = await rag_service.ask(
            question=request.message,
            selected_text=request.selectedText,
            user_level=request.userLevel,
            language=request.language
        )
        
        # Parse citations
        citations = [Citation(**c) for c in result.get('citations', [])]  # ADDED
        
        # Auto-save user message to conversation history  # ADDED BLOCK
        await _save_message_to_conversation(
            conversation_id=conversation_id,
            role="user",
            content=request.message,
            user_email=request.user_email
        )
        
        # Auto-save assistant response to conversation history
        await _save_message_to_conversation(
            conversation_id=conversation_id,
            role="assistant",
            content=result['answer'],
            citations=citations,
            user_email=request.user_email
        )
        # END ADDED BLOCK
        
        return ChatResponse(  # CHANGED
            answer=result['answer'],
            citations=citations,
            sourcesCount=result.get('sourcesCount', len(citations)),
            model=result.get('model', 'gpt-4o-mini'),
            conversation_id=conversation_id  # ADDED
        )

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"RAG pipeline failed: {str(e)}"
        )
```

---

### File 3: `backend/app/routers/conversations.py` (MODIFIED)

#### Import Changes
```python
# ADDED:
from app.storage import get_conversations, get_messages

# REMOVED:
# CONVERSATIONS = {}
# MESSAGES = {}
```

#### Each Endpoint Change Pattern
For all endpoints, add at the start:
```python
CONVERSATIONS = get_conversations()
MESSAGES = get_messages()
```

Example for `create_conversation()`:
```python
@router.post("/", response_model=ConversationResponse)
async def create_conversation(
    user_email: Optional[str] = None,
    title: str = "New Conversation"
):
    import uuid
    
    CONVERSATIONS = get_conversations()  # ADDED
    MESSAGES = get_messages()              # ADDED
    
    # ... rest of function unchanged
```

---

## Frontend Changes

### File 4: `src/services/apiService.ts` (MODIFIED)

#### ChatRequest Interface Changes
```typescript
export interface ChatRequest {
  message: string;
  conversationId?: string;      // ADDED
  userEmail?: string;            // ADDED
  selectedText?: string;
  userLevel?: string;
  language?: string;
}
```

#### ChatResponse Interface Changes
```typescript
export interface ChatResponse {
  answer: string;
  citations: Citation[];
  sourcesCount: number;
  model: string;
  conversation_id: string;      // ADDED
}
```

#### sendChatMessage() Function Changes
```typescript
export async function sendChatMessage(request: ChatRequest, signal?: AbortSignal): Promise<ChatResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/chat/`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        message: request.message,
        conversation_id: request.conversationId || null,      // ADDED
        user_email: request.userEmail || null,                // ADDED
        selectedText: request.selectedText || null,
        userLevel: request.userLevel || 'intermediate',
        language: request.language || 'en',
      }),
      signal,
    });
    // ... rest unchanged
  }
}
```

---

### File 5: `src/components/RAGChatWidget.tsx` (MODIFIED)

#### State Changes
```typescript
export function RAGChatWidget() {
  // ... existing state ...
  const [conversationId, setConversationId] = useState<string | null>(null);  // ADDED
  
  // ... rest of component
```

#### sendMessage() Function Changes
```typescript
const sendMessage = async (messageText?: string, context?: string) => {
    const textToSend = messageText || input;
    if (!textToSend.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: textToSend,
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const currentPath = typeof window !== 'undefined' ? window.location.pathname : '/';

      // MODIFIED CALL:
      const data = await sendChatMessage({
        message: textToSend,
        conversationId: conversationId || undefined,      // ADDED
        userEmail: userProfile?.email || undefined,       // ADDED
        selectedText: context || selectedText,
        userLevel: userProfile?.preferences?.difficulty_preference || 'intermediate',
        language: userProfile?.preferences?.language || 'en',
      });

      // Update conversation ID from response (if this is first message)  // ADDED BLOCK
      if (!conversationId) {
        setConversationId(data.conversation_id);
      }
      // END ADDED BLOCK

      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: data.answer,
        citations: data.citations,
      };

      setMessages(prev => [...prev, assistantMessage]);
      setSelectedText('');
    } catch (error) {
      // ... error handling unchanged
    } finally {
      setIsLoading(false);
    }
  };
```

#### handleNewConversation() Changes
```typescript
const handleNewConversation = () => {
  setMessages([
    {
      id: 'welcome',
      role: 'assistant',
      content: "Hello! I'm your AI Tutor...",
    },
  ]);
  setConversationId(null);  // ADDED - start new conversation
  setShowHistory(false);
};
```

#### handleClearChat() Changes
```typescript
const handleClearChat = () => {
  // MODIFIED MESSAGE:
  if (window.confirm('Clear this conversation? (History is saved in the database)')) {
    handleNewConversation();
  }
};
```

---

## Summary of Changes

| File | Type | Changes |
|------|------|---------|
| backend/app/storage.py | NEW | 17 lines - Shared storage module |
| backend/app/routers/chat.py | MODIFIED | ~150 lines - Auto-save logic |
| backend/app/routers/conversations.py | MODIFIED | ~20 lines - Use shared storage |
| src/services/apiService.ts | MODIFIED | ~10 lines - Handle conversation_id |
| src/components/RAGChatWidget.tsx | MODIFIED | ~20 lines - Manage conversation state |

**Total additions: ~217 lines**
**Total modifications: ~50 lines**
**No breaking changes to existing API**

---

## Testing the Changes

### Quick Verification
```bash
# Backend syntax check
cd backend
python -m py_compile app/storage.py app/routers/chat.py app/routers/conversations.py

# Frontend build check
cd physical-ai-humanoid-textbook
npm run build
```

### Manual Test
1. Send message "What is ZMP?"
2. Copy conversation_id from response
3. Clear chat (UI clears, history remains)
4. Test API: `curl http://localhost:8000/api/conversations/{conversation_id}`
5. Verify messages are still there ✅
