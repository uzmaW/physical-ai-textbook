"""
Conversation models for chat history
Stores user conversations for persistence
"""

from sqlalchemy import Column, String, DateTime, JSON, Integer, ForeignKey, Text
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime
import uuid

Base = declarative_base()


class Conversation(Base):
    """Conversation session model."""
    __tablename__ = "conversations"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_email = Column(String, index=True, nullable=True)  # Null for anonymous users
    title = Column(String, default="New Conversation")
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    message_count = Column(Integer, default=0)
    extra_data = Column(JSON, default={})  # Store chapter context, preferences, etc.


class Message(Base):
    """Individual message in a conversation."""
    __tablename__ = "messages"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    conversation_id = Column(String, ForeignKey("conversations.id"), index=True)
    role = Column(String)  # "user" or "assistant"
    content = Column(Text)
    timestamp = Column(DateTime, default=datetime.utcnow)
    citations = Column(JSON, default=[])  # Store retrieved context citations
    extra_data = Column(JSON, default={})  # Store additional info
