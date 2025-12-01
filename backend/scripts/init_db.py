"""
Initialize database tables
Creates users and conversations tables in PostgreSQL
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from sqlalchemy import create_engine
from app.config import settings
from app.models.user import Base as UserBase
from app.models.conversation import Base as ConversationBase

def init_database():
    """Create all database tables."""
    print("🔧 Initializing database...")
    print(f"📍 Database: {settings.DATABASE_URL.split('@')[1] if '@' in settings.DATABASE_URL else 'local'}")

    # Create engine
    engine = create_engine(settings.DATABASE_URL)

    # Create all tables
    print("📊 Creating tables...")
    UserBase.metadata.create_all(bind=engine)
    ConversationBase.metadata.create_all(bind=engine)

    print("✅ Database initialized successfully!")
    print("Tables created:")
    print("  - users")
    print("  - conversations")
    print("  - messages")

if __name__ == "__main__":
    init_database()
