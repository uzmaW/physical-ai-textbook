"""
Configuration management using pydantic-settings
Loads environment variables from .env file
"""

from pydantic_settings import BaseSettings
from functools import lru_cache

class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Database
    DATABASE_URL: str = "postgresql+psycopg://user:pass@localhost/textbook"

    # Qdrant Vector Store
    QDRANT_URL: str = "https://468ff8ad-822f-4a3a-9f5c-b8953bbadc70.us-east-1-1.aws.cloud.qdrant.io:6333"
    QDRANT_API_KEY: str = ""

    # OpenAI
    OPENAI_API_KEY: str = ""

    # HuggingFace
    HUGGINGFACE_API_KEY: str = ""  # Optional for free Inference API
    USE_HF_API: bool = True  # True = API mode (small image), False = local model

    # Google Cloud
    GOOGLE_APPLICATION_CREDENTIALS: str = ""
    GOOGLE_TRANSLATE_API_KEY: str = ""

    # Authentication
    JWT_SECRET: str = "change-this-secret-key-in-production"
    JWT_ALGORITHM: str = "HS256"
    JWT_EXPIRATION_DAYS: int = 30

    # OAuth
    GITHUB_CLIENT_ID: str = ""
    GITHUB_CLIENT_SECRET: str = ""
    GOOGLE_CLIENT_ID: str = ""
    GOOGLE_CLIENT_SECRET: str = ""

    # CORS
    FRONTEND_URL: str = "http://localhost:3000"

    # Application
    DEBUG: bool = True
    API_VERSION: str = "1.0.0"
    
    # Render deployment
    RENDER_API_KEY: str = ""
    
    # Voice & Audio (Free/Open-Source)
    WHISPER_MODEL: str = "base"  # base, small, medium, large (base = ~140MB, fastest)
    VOICE_CACHE_DIR: str = "/tmp/voice_cache"  # Local caching directory
    VOICE_SESSION_TIMEOUT_HOURS: int = 24
    VOICE_MAX_AUDIO_LENGTH_SECONDS: int = 30

    class Config:
        env_file = ".env"
        case_sensitive = True


@lru_cache()
def get_settings() -> Settings:
    """Cached settings instance."""
    return Settings(_env_file='.env')


# Global settings instance
settings = get_settings()
