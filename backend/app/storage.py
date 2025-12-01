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
