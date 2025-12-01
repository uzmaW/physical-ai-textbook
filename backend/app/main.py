"""
FastAPI Backend for Physical AI Textbook Platform
Provides RAG, translation, authentication, and personalization services
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routers import chat, auth, conversations, translate, voice
# from app.routers import personalization  # To be added
import os

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI Textbook API",
    description="Backend for intelligent, personalized robotics education",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# CORS Configuration
allowed_origins = [
    "http://localhost:3000",  # Local development
    "http://localhost:5173",  # Vite dev
    "https://piaic.github.io",  # Production
]

# Add environment-specific domain
frontend_url = os.getenv("FRONTEND_URL", "")
if frontend_url and frontend_url not in allowed_origins:
    allowed_origins.append(frontend_url)

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat.router, prefix="/api/chat", tags=["chat"])
app.include_router(auth.router, prefix="/api/auth", tags=["auth"])
app.include_router(conversations.router, prefix="/api/conversations", tags=["conversations"])
app.include_router(translate.router, prefix="/api/translate", tags=["translation"])
app.include_router(voice.router, prefix="/api/voice", tags=["voice"])
# app.include_router(personalization.router, prefix="/api/personalization", tags=["personalization"])

@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "Physical AI Textbook API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/health"
    }

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "version": "1.0.0",
        "services": {
            "database": "connected",  # TODO: Add actual DB check
            "qdrant": "connected",  # TODO: Add actual Qdrant check
            "openai": "configured"
        }
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )
