"""
Database models for users, metadata, and translation cache
Uses SQLAlchemy ORM with Neon PostgreSQL
"""

from sqlalchemy import Column, String, JSON, DateTime, Enum as SQLEnum
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime
import enum

Base = declarative_base()


class HardwareLevel(str, enum.Enum):
    """Hardware experience levels."""
    NONE = "none"
    ARDUINO = "arduino"
    ROS = "ros"
    INDUSTRIAL = "industrial"


class SoftwareLevel(str, enum.Enum):
    """Software proficiency levels."""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class User(Base):
    """
    User account table.
    Supports both email/password and OAuth authentication.
    """
    __tablename__ = "users"

    id = Column(String, primary_key=True)  # UUID
    email = Column(String, unique=True, nullable=False, index=True)
    hashed_password = Column(String)  # Null for OAuth users
    oauth_provider = Column(String)  # 'github', 'google', or null
    oauth_id = Column(String)  # Provider's user ID
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)


class UserMetadata(Base):
    """
    User profile and learning preferences.
    Collected during signup survey.
    """
    __tablename__ = "user_metadata"

    user_id = Column(String, primary_key=True)  # Foreign key to users.id
    hardware_background = Column(SQLEnum(HardwareLevel), nullable=False)
    software_level = Column(SQLEnum(SoftwareLevel), nullable=False)
    learning_goal = Column(String, nullable=False)

    # Personalization
    difficulty_preference = Column(String, default='intermediate')
    language_preference = Column(String, default='en')

    # Progress tracking
    completed_chapters = Column(JSON, default=list)  # List of chapter IDs
    current_chapter = Column(String, default='week-01')
    total_time_minutes = Column(JSON, default=dict)  # {chapter_id: minutes}

    # Timestamps
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)


class TranslationCache(Base):
    """
    Cached translations to reduce Google Translate API costs.
    One cache entry per user+chapter combination.
    """
    __tablename__ = "translation_cache"

    id = Column(String, primary_key=True)  # Format: "{user_id}:{chapter_id}"
    user_id = Column(String, nullable=False, index=True)
    chapter_id = Column(String, nullable=False, index=True)

    source_lang = Column(String, default='en')
    target_lang = Column(String, default='ur')

    # Cached content
    translated_content = Column(String, nullable=False)  # Full HTML/Markdown

    # Metadata
    translation_service = Column(String, default='google')  # Future: support multiple services
    created_at = Column(DateTime, default=datetime.utcnow)
    character_count = Column(JSON, default=dict)  # Cost tracking


class VoiceSession(Base):
    """
    Voice interaction session - records user's speech input and AI responses.
    Used for tracking voice feature usage and improving accuracy.
    """
    __tablename__ = "voice_sessions"

    id = Column(String, primary_key=True)  # UUID
    user_id = Column(String, nullable=False, index=True)
    chapter_id = Column(String, nullable=False, index=True)
    language = Column(String, default='en')  # "en" or "ur"

    # Input (speech)
    input_audio_duration_seconds = Column(String)  # Duration of recorded audio
    input_transcript = Column(String, nullable=False)  # Text extracted from speech
    stt_confidence = Column(String)  # Confidence score (0.0-1.0)

    # Output (response)
    output_text = Column(String, nullable=False)  # AI response text
    output_audio_cached = Column(String, default='false')  # "true" if audio was cached

    # Metadata
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False, index=True)
    deleted_at = Column(DateTime, nullable=True)  # Soft delete for GDPR

    # Indexes for fast querying
    __table_args__ = (
        # Index for user history
        # Index for cleanup queries (old sessions)
    )


class VoicePreference(Base):
    """
    User voice settings - persisted preferences for voice feature.
    One preference record per user.
    """
    __tablename__ = "voice_preferences"

    user_id = Column(String, primary_key=True)  # Foreign key to users.id

    # Language and voice preferences
    preferred_language = Column(String, default='en')  # "en", "ur", or "auto-detect"
    preferred_voice = Column(String, default='narrator_en')  # e.g., "narrator_en", "professor_ur"
    playback_speed = Column(String, default='1.0')  # 0.5 to 2.0

    # Feature flags
    microphone_enabled = Column(String, default='true')  # "true" or "false"
    auto_submit_voice = Column(String, default='true')  # Auto-send voice queries

    # Timestamps
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)


class SpeechCache(Base):
    """
    Cache for speech-to-text results to avoid re-processing identical audio.
    Uses audio hash as key.
    """
    __tablename__ = "speech_cache"

    id = Column(String, primary_key=True)  # MD5 hash of audio blob
    language = Column(String, default='en')

    # Cached result
    transcript = Column(String, nullable=False)
    confidence = Column(String)  # Confidence score

    # Metadata
    created_at = Column(DateTime, default=datetime.utcnow, index=True)
    expires_at = Column(DateTime, nullable=False)  # 30 day TTL


# Create all tables
def init_db(engine):
    """Initialize database tables."""
    Base.metadata.create_all(bind=engine)
