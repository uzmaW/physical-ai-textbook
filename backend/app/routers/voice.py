"""
Voice router for speech-to-text and text-to-speech integration
Handles voice input, RAG queries, and audio response generation
Uses free models: Whisper (local) + Edge-TTS (free)
"""

import uuid
from fastapi import APIRouter, HTTPException, Depends, UploadFile, File
from pydantic import BaseModel
from sqlalchemy.orm import Session
from typing import Optional
from datetime import datetime, timedelta
import io

from app.db.neon import get_db
from app.models.user import VoiceSession, VoicePreference
from app.services.voice import get_voice_service
from app.services.rag import RAGService

router = APIRouter()


# ============================================================================
# Pydantic Models (Request/Response)
# ============================================================================

class TranscribeRequest(BaseModel):
    """Request to transcribe audio to text"""
    language: str = "en"  # "en" or "ur"


class TranscribeResponse(BaseModel):
    """Response with transcribed text"""
    transcript: str
    language: str
    confidence: float
    processingTimeMs: int
    durationSeconds: float


class VoiceAskRequest(BaseModel):
    """Request to ask a voice question"""
    transcript: str
    language: str = "en"
    userLevel: str = "intermediate"
    chapterId: Optional[str] = None


class VoiceAskResponse(BaseModel):
    """Response with AI answer and audio"""
    answer: str
    audioUrl: str  # URL to cached audio file or inline data
    audioAvailable: bool
    audioLanguage: str
    citations: list = []
    processingTimeMs: int


class VoicePreferenceRequest(BaseModel):
    """Request to set voice preferences"""
    preferredLanguage: str = "en"
    preferredVoice: str = "narrator_en"
    playbackSpeed: float = 1.0
    microphoneEnabled: bool = True
    autoSubmitVoice: bool = True


class VoicePreferenceResponse(BaseModel):
    """Response with saved preferences"""
    preferredLanguage: str
    preferredVoice: str
    playbackSpeed: float
    microphoneEnabled: bool
    autoSubmitVoice: bool
    saved: bool


# ============================================================================
# Helper Functions
# ============================================================================

def get_user_preferences(user_id: str, db: Session) -> dict:
    """Get user's voice preferences or defaults"""
    pref = db.query(VoicePreference).filter(
        VoicePreference.user_id == user_id
    ).first()
    
    if pref:
        return {
            "language": pref.preferred_language,
            "voice": pref.preferred_voice,
            "speed": float(pref.playback_speed),
            "microphone": pref.microphone_enabled == 'true',
            "auto_submit": pref.auto_submit_voice == 'true'
        }
    else:
        # Return defaults
        return {
            "language": "en",
            "voice": "narrator_en",
            "speed": 1.0,
            "microphone": True,
            "auto_submit": True
        }


def save_voice_session(
    db: Session,
    user_id: str,
    chapter_id: str,
    transcript: str,
    response_text: str,
    language: str = "en",
    confidence: float = 0.0,
    audio_duration: float = 0.0,
    audio_cached: bool = False
) -> str:
    """Save voice session to database"""
    session_id = str(uuid.uuid4())
    
    session = VoiceSession(
        id=session_id,
        user_id=user_id,
        chapter_id=chapter_id,
        language=language,
        input_transcript=transcript,
        input_audio_duration_seconds=str(audio_duration),
        stt_confidence=str(confidence),
        output_text=response_text,
        output_audio_cached='true' if audio_cached else 'false',
        created_at=datetime.utcnow()
    )
    
    db.add(session)
    db.commit()
    return session_id


# ============================================================================
# API Endpoints
# ============================================================================

@router.post("/transcribe", response_model=TranscribeResponse)
async def transcribe_audio(
    file: UploadFile = File(...),
    language: str = "en",
    db: Session = Depends(get_db)
):
    """
    Transcribe audio file to text using local Whisper model.
    
    Args:
        file: Audio file (WAV, MP3, etc.)
        language: "en" or "ur"
        
    Returns:
        Transcribed text with confidence score
    """
    try:
        # Read audio file
        audio_bytes = await file.read()
        
        if not audio_bytes:
            raise HTTPException(status_code=400, detail="Empty audio file")
        
        # Get file format from content-type
        audio_format = "wav"  # Default
        if file.content_type:
            if "mp3" in file.content_type:
                audio_format = "mp3"
            elif "wav" in file.content_type:
                audio_format = "wav"
            elif "ogg" in file.content_type:
                audio_format = "ogg"
        
        # Transcribe using VoiceService
        voice_service = get_voice_service()
        result = await voice_service.transcribe_audio(
            audio_blob=audio_bytes,
            language=language,
            audio_format=audio_format
        )
        
        # Check for errors
        if "error" in result:
            raise HTTPException(
                status_code=400,
                detail=result.get("error", "Transcription failed")
            )
        
        return TranscribeResponse(
            transcript=result["transcript"],
            language=result["language"],
            confidence=result["confidence"],
            processingTimeMs=result["processingTimeMs"],
            durationSeconds=result["durationSeconds"]
        )
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Transcription error: {str(e)}")


@router.post("/ask", response_model=VoiceAskResponse)
async def voice_ask(
    request: VoiceAskRequest,
    db: Session = Depends(get_db),
    user_id: str = "guest"
):
    """
    Ask a question via voice and get AI response with audio.
    
    Flow:
    1. User's voice transcript is received
    2. Query RAG service for relevant context
    3. Generate AI response
    4. Synthesize response to speech
    5. Cache audio for reuse
    
    Args:
        request: Question transcript and metadata
        db: Database session
        user_id: User identifier
        
    Returns:
        AI response text + audio URL
    """
    import time
    start_time = time.time()
    
    try:
        # Validate input
        if not request.transcript or not request.transcript.strip():
            raise HTTPException(status_code=400, detail="Empty transcript")
        
        # Get user preferences
        user_prefs = get_user_preferences(user_id, db)
        
        # Query RAG service for relevant context
        rag_service = RAGService()
        
        search_results = await rag_service.search_context(
            query=request.transcript,
            user_level=request.userLevel,
            language=request.language,
            chapter_filter=request.chapterId
        )
        
        # Generate AI response
        response_text = await rag_service.generate_response(
            query=request.transcript,
            context=search_results,
            user_level=request.userLevel
        )
        
        if not response_text:
            raise HTTPException(status_code=500, detail="Failed to generate response")
        
        # Synthesize response to speech
        voice_service = get_voice_service()
        
        # Use user's preferred language and voice
        target_language = user_prefs["language"]
        voice_style = user_prefs["voice"].split("_")[0]  # "narrator" or "professor"
        
        # Check cache first
        cached_audio = await voice_service.get_cached_audio(
            text=response_text,
            language=target_language,
            voice=voice_style,
            speed=user_prefs["speed"]
        )
        
        audio_cached = False
        if cached_audio:
            audio_bytes = cached_audio
            audio_cached = True
        else:
            # Generate new audio
            audio_bytes, metadata = await voice_service.synthesize_response(
                text=response_text,
                language=target_language,
                voice=voice_style,
                speed=user_prefs["speed"]
            )
            
            if not audio_bytes:
                # Continue without audio if synthesis fails
                audio_url = ""
            else:
                # Save to cache for future use
                await voice_service.save_cached_audio(
                    text=response_text,
                    language=target_language,
                    voice=voice_style,
                    speed=user_prefs["speed"],
                    audio_bytes=audio_bytes
                )
        
        # Convert audio bytes to base64 data URL for inline playback
        audio_url = ""
        if audio_bytes:
            import base64
            audio_base64 = base64.b64encode(audio_bytes).decode('utf-8')
            audio_url = f"data:audio/mp3;base64,{audio_base64}"
        
        # Save voice session to database
        session_id = save_voice_session(
            db=db,
            user_id=user_id,
            chapter_id=request.chapterId or "unknown",
            transcript=request.transcript,
            response_text=response_text,
            language=request.language,
            audio_cached=audio_cached
        )
        
        processing_time = int((time.time() - start_time) * 1000)
        
        return VoiceAskResponse(
            answer=response_text,
            audioUrl=audio_url,
            audioAvailable=bool(audio_bytes),
            audioLanguage=target_language,
            citations=[r.get("metadata", {}).get("url", "") for r in search_results[:3]],
            processingTimeMs=processing_time
        )
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Voice ask error: {str(e)}")


@router.get("/preferences", response_model=VoicePreferenceResponse)
async def get_voice_preferences(
    db: Session = Depends(get_db),
    user_id: str = "guest"
):
    """
    Get user's voice preferences.
    
    Returns:
        User's voice settings (language, voice, speed, etc.)
    """
    try:
        prefs = db.query(VoicePreference).filter(
            VoicePreference.user_id == user_id
        ).first()
        
        if prefs:
            return VoicePreferenceResponse(
                preferredLanguage=prefs.preferred_language,
                preferredVoice=prefs.preferred_voice,
                playbackSpeed=float(prefs.playback_speed),
                microphoneEnabled=prefs.microphone_enabled == 'true',
                autoSubmitVoice=prefs.auto_submit_voice == 'true',
                saved=True
            )
        else:
            # Return defaults
            return VoicePreferenceResponse(
                preferredLanguage="en",
                preferredVoice="narrator_en",
                playbackSpeed=1.0,
                microphoneEnabled=True,
                autoSubmitVoice=True,
                saved=False
            )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error fetching preferences: {str(e)}")


@router.post("/preferences", response_model=VoicePreferenceResponse)
async def save_voice_preferences(
    request: VoicePreferenceRequest,
    db: Session = Depends(get_db),
    user_id: str = "guest"
):
    """
    Save user's voice preferences.
    
    Body:
        {
            "preferredLanguage": "ur",
            "preferredVoice": "narrator_ur",
            "playbackSpeed": 1.5,
            "microphoneEnabled": true,
            "autoSubmitVoice": true
        }
        
    Returns:
        Saved preferences confirmation
    """
    try:
        # Validate speed
        if not (0.5 <= request.playbackSpeed <= 2.0):
            raise HTTPException(
                status_code=400,
                detail="Playback speed must be between 0.5 and 2.0"
            )
        
        # Upsert preferences
        prefs = db.query(VoicePreference).filter(
            VoicePreference.user_id == user_id
        ).first()
        
        if prefs:
            # Update existing
            prefs.preferred_language = request.preferredLanguage
            prefs.preferred_voice = request.preferredVoice
            prefs.playback_speed = str(request.playbackSpeed)
            prefs.microphone_enabled = 'true' if request.microphoneEnabled else 'false'
            prefs.auto_submit_voice = 'true' if request.autoSubmitVoice else 'false'
            prefs.updated_at = datetime.utcnow()
        else:
            # Create new
            prefs = VoicePreference(
                user_id=user_id,
                preferred_language=request.preferredLanguage,
                preferred_voice=request.preferredVoice,
                playback_speed=str(request.playbackSpeed),
                microphone_enabled='true' if request.microphoneEnabled else 'false',
                auto_submit_voice='true' if request.autoSubmitVoice else 'false',
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow()
            )
            db.add(prefs)
        
        db.commit()
        
        return VoicePreferenceResponse(
            preferredLanguage=request.preferredLanguage,
            preferredVoice=request.preferredVoice,
            playbackSpeed=request.playbackSpeed,
            microphoneEnabled=request.microphoneEnabled,
            autoSubmitVoice=request.autoSubmitVoice,
            saved=True
        )
        
    except HTTPException:
        raise
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"Error saving preferences: {str(e)}")


@router.delete("/session/{session_id}")
async def delete_voice_session(
    session_id: str,
    db: Session = Depends(get_db),
    user_id: str = "guest"
):
    """
    Soft-delete a voice session (GDPR compliance).
    
    Path Params:
        session_id: Voice session UUID
        
    Returns:
        Confirmation
    """
    try:
        session = db.query(VoiceSession).filter(
            VoiceSession.id == session_id,
            VoiceSession.user_id == user_id  # Only allow user to delete own
        ).first()
        
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")
        
        # Soft delete
        session.deleted_at = datetime.utcnow()
        db.commit()
        
        return {"message": "Session deleted", "sessionId": session_id}
        
    except HTTPException:
        raise
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"Error deleting session: {str(e)}")


@router.get("/history")
async def get_voice_history(
    limit: int = 20,
    db: Session = Depends(get_db),
    user_id: str = "guest"
):
    """
    Get user's recent voice interactions (not deleted).
    
    Query Params:
        limit: Max number of sessions to return (default 20)
        
    Returns:
        List of recent voice sessions
    """
    try:
        sessions = db.query(VoiceSession).filter(
            VoiceSession.user_id == user_id,
            VoiceSession.deleted_at == None  # Exclude soft-deleted
        ).order_by(
            VoiceSession.created_at.desc()
        ).limit(limit).all()
        
        return {
            "sessions": [
                {
                    "id": s.id,
                    "transcript": s.input_transcript,
                    "response": s.output_text,
                    "language": s.language,
                    "createdAt": s.created_at.isoformat(),
                    "confidence": float(s.stt_confidence) if s.stt_confidence else 0.0
                }
                for s in sessions
            ],
            "count": len(sessions)
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error fetching history: {str(e)}")
