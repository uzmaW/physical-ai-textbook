"""
Conversation history router
Manages chat history per user with persistence
"""

from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
from app.storage import get_conversations, get_messages

router = APIRouter()


class MessageCreate(BaseModel):
    """Create message model."""
    role: str
    content: str
    citations: Optional[List[dict]] = []


class MessageResponse(BaseModel):
    """Message response model."""
    id: str
    role: str
    content: str
    timestamp: str
    citations: List[dict]


class ConversationResponse(BaseModel):
    """Conversation response model."""
    conversation_id: str
    user_email: Optional[str]
    title: str
    created_at: str
    updated_at: str
    message_count: int


class ConversationWithMessages(BaseModel):
    """Conversation with all messages."""
    conversation_id: str
    title: str
    messages: List[MessageResponse]


@router.post("/", response_model=ConversationResponse)
async def create_conversation(
    user_email: Optional[str] = None,
    title: str = "New Conversation"
):
    """
    Create a new conversation.

    Args:
        user_email: User's email (optional for anonymous)
        title: Conversation title

    Returns:
        Created conversation
    """
    import uuid

    CONVERSATIONS = get_conversations()
    MESSAGES = get_messages()

    conversation_id = str(uuid.uuid4())
    conversation = {
        "conversation_id": conversation_id,
        "user_email": user_email,
        "title": title,
        "created_at": datetime.utcnow().isoformat(),
        "updated_at": datetime.utcnow().isoformat(),
        "message_count": 0,
        "metadata": {}
    }

    CONVERSATIONS[conversation_id] = conversation
    MESSAGES[conversation_id] = []

    return ConversationResponse(**conversation)


@router.get("/user/{user_email}", response_model=List[ConversationResponse])
async def get_user_conversations(user_email: str):
    """
    Get all conversations for a user.

    Args:
        user_email: User's email

    Returns:
        List of conversations
    """
    CONVERSATIONS = get_conversations()

    user_conversations = [
        conv for conv in CONVERSATIONS.values()
        if conv["user_email"] == user_email
    ]

    return [ConversationResponse(**conv) for conv in user_conversations]


@router.get("/{conversation_id}", response_model=ConversationWithMessages)
async def get_conversation(conversation_id: str):
    """
    Get a conversation with all its messages.

    Args:
        conversation_id: Conversation ID

    Returns:
        Conversation with messages
    """
    CONVERSATIONS = get_conversations()
    MESSAGES = get_messages()

    if conversation_id not in CONVERSATIONS:
        raise HTTPException(status_code=404, detail="Conversation not found")

    conversation = CONVERSATIONS[conversation_id]
    messages = MESSAGES.get(conversation_id, [])

    return ConversationWithMessages(
        conversation_id=conversation_id,
        title=conversation["title"],
        messages=[MessageResponse(**msg) for msg in messages]
    )


@router.post("/{conversation_id}/messages", response_model=MessageResponse)
async def add_message(conversation_id: str, message: MessageCreate):
    """
    Add a message to a conversation.

    Args:
        conversation_id: Conversation ID
        message: Message data

    Returns:
        Created message
    """
    CONVERSATIONS = get_conversations()
    MESSAGES = get_messages()

    if conversation_id not in CONVERSATIONS:
        raise HTTPException(status_code=404, detail="Conversation not found")

    import uuid

    message_data = {
        "id": str(uuid.uuid4()),
        "role": message.role,
        "content": message.content,
        "timestamp": datetime.utcnow().isoformat(),
        "citations": message.citations or []
    }

    MESSAGES[conversation_id].append(message_data)

    # Update conversation
    CONVERSATIONS[conversation_id]["message_count"] += 1
    CONVERSATIONS[conversation_id]["updated_at"] = datetime.utcnow().isoformat()

    return MessageResponse(**message_data)


@router.delete("/{conversation_id}")
async def delete_conversation(conversation_id: str):
    """
    Delete a conversation and all its messages.

    Args:
        conversation_id: Conversation ID

    Returns:
        Success message
    """
    CONVERSATIONS = get_conversations()
    MESSAGES = get_messages()

    if conversation_id not in CONVERSATIONS:
        raise HTTPException(status_code=404, detail="Conversation not found")

    del CONVERSATIONS[conversation_id]
    if conversation_id in MESSAGES:
        del MESSAGES[conversation_id]

    return {"message": "Conversation deleted", "conversation_id": conversation_id}


@router.put("/{conversation_id}/title")
async def update_conversation_title(conversation_id: str, title: str):
    """
    Update conversation title.

    Args:
        conversation_id: Conversation ID
        title: New title

    Returns:
        Updated conversation
    """
    CONVERSATIONS = get_conversations()

    if conversation_id not in CONVERSATIONS:
        raise HTTPException(status_code=404, detail="Conversation not found")

    CONVERSATIONS[conversation_id]["title"] = title
    CONVERSATIONS[conversation_id]["updated_at"] = datetime.utcnow().isoformat()

    return ConversationResponse(**CONVERSATIONS[conversation_id])
