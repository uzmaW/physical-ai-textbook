"""
Chat router for RAG-powered Q&A
Endpoint: POST /api/chat
Auto-saves messages to conversation history
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, List, Dict
from app.services.rag import RAGService
from app.storage import get_conversations, get_messages
import uuid
from datetime import datetime

router = APIRouter()
rag_service = RAGService()


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    message: str
    conversation_id: Optional[str] = None
    user_email: Optional[str] = None
    selectedText: Optional[str] = None
    userLevel: str = "intermediate"
    language: str = "en"


class Citation(BaseModel):
    """Citation model."""
    chapter: str
    url: str
    relevance: float


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    answer: str
    citations: List[Citation]
    sourcesCount: int
    model: str
    conversation_id: str


async def _save_message_to_conversation(
    conversation_id: str,
    role: str,
    content: str,
    citations: Optional[List[Citation]] = None,
    user_email: Optional[str] = None
) -> dict:
    """
    Save a message to conversation history.
    
    Args:
        conversation_id: Conversation ID
        role: "user" or "assistant"
        content: Message content
        citations: Optional citations for assistant messages
        user_email: User email for creating conversation if needed
    
    Returns:
        Saved message data
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


@router.post("/", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Handle chat requests with RAG and auto-save to conversation history.

    Args:
        request: ChatRequest with message, conversation_id, user_email, optional selected text

    Returns:
        ChatResponse with answer, citations, and conversation_id

    Example:
        POST /api/chat
        {
            "message": "What is ZMP?",
            "conversation_id": "conv-uuid",
            "user_email": "user@example.com",
            "selectedText": "Zero-Moment Point criterion...",
            "userLevel": "intermediate",
            "language": "en"
        }

        Response:
        {
            "answer": "ZMP (Zero-Moment Point) is a stability criterion...",
            "citations": [{"chapter": "Week 11", "url": "/chapters/week-11"}],
            "sourcesCount": 5,
            "model": "gpt-4o-mini",
            "conversation_id": "conv-uuid"
        }
    """
    try:
        # Generate or use existing conversation ID
        conversation_id = request.conversation_id or str(uuid.uuid4())
        
        # Get RAG response
        result = await rag_service.ask(
            question=request.message,
            selected_text=request.selectedText,
            user_level=request.userLevel,
            language=request.language
        )
        
        # Parse citations
        citations = [Citation(**c) for c in result.get('citations', [])]
        
        # Auto-save user message to conversation history
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
        
        return ChatResponse(
            answer=result['answer'],
            citations=citations,
            sourcesCount=result.get('sourcesCount', len(citations)),
            model=result.get('model', 'gpt-4o-mini'),
            conversation_id=conversation_id
        )

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"RAG pipeline failed: {str(e)}"
        )


@router.get("/health")
async def chat_health():
    """Health check for chat service."""
    return {
        "status": "healthy",
        "service": "chat",
        "rag_enabled": True
    }
