"""
Voice service: Speech-to-text (Whisper) and Text-to-speech (Edge-TTS)
Uses free/open-source models only
"""

import os
import io
import json
import hashlib
import logging
import asyncio
from pathlib import Path
from typing import Optional, Tuple
from datetime import datetime, timedelta

import whisper
import edge_tts
from pydub import AudioSegment

from app.config import settings

logger = logging.getLogger(__name__)


class VoiceService:
    """
    Handles voice input (speech-to-text) and voice output (text-to-speech)
    Uses local Whisper model and Edge-TTS for generation
    """

    def __init__(self):
        """Initialize voice service with Whisper model"""
        self.model = None
        self.cache_dir = Path(settings.VOICE_CACHE_DIR)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self._load_whisper_model()

    def _load_whisper_model(self):
        """Load Whisper model on first use"""
        try:
            logger.info(f"Loading Whisper model: {settings.WHISPER_MODEL}")
            self.model = whisper.load_model(settings.WHISPER_MODEL)
            logger.info("Whisper model loaded successfully")
        except Exception as e:
            logger.error(f"Failed to load Whisper model: {e}")
            raise

    async def transcribe_audio(
        self,
        audio_blob: bytes,
        language: str = "en",
        audio_format: str = "wav"
    ) -> dict:
        """
        Convert audio blob to text using Whisper
        
        Args:
            audio_blob: Raw audio bytes
            language: "en" or "ur" (for language-specific processing)
            audio_format: Audio format (wav, mp3, etc.)
            
        Returns:
            {
                "transcript": "What is a humanoid robot?",
                "language": "en",
                "confidence": 0.95,
                "processingTimeMs": 1200
            }
        """
        start_time = datetime.now()
        
        try:
            # Validate audio length
            audio_segment = AudioSegment.from_file(
                io.BytesIO(audio_blob),
                format=audio_format
            )
            
            duration_seconds = len(audio_segment) / 1000.0
            if duration_seconds > settings.VOICE_MAX_AUDIO_LENGTH_SECONDS:
                return {
                    "error": f"Audio too long ({duration_seconds}s). Max: {settings.VOICE_MAX_AUDIO_LENGTH_SECONDS}s",
                    "code": "AUDIO_TOO_LONG"
                }
            
            if duration_seconds < 0.5:
                return {
                    "error": "Audio too short (< 0.5s)",
                    "code": "AUDIO_TOO_SHORT"
                }
            
            # Save audio to temp file for Whisper
            temp_audio_path = self.cache_dir / f"temp_{datetime.now().timestamp()}.wav"
            audio_segment.export(str(temp_audio_path), format="wav")
            
            # Transcribe with Whisper
            logger.info(f"Transcribing audio ({duration_seconds}s) in language: {language}")
            result = self.model.transcribe(
                str(temp_audio_path),
                language="en" if language == "en" else "ur",
                verbose=False
            )
            
            transcript = result.get("text", "").strip()
            confidence = result.get("segments", [{}])[0].get("avg_logprob", 0.0)
            # Normalize confidence to 0-1 range
            confidence = max(0.0, min(1.0, (confidence + 3.5) / 3.5))
            
            # Clean up temp file
            try:
                temp_audio_path.unlink()
            except:
                pass
            
            processing_time = int((datetime.now() - start_time).total_seconds() * 1000)
            
            logger.info(f"Transcription complete: '{transcript}' (confidence: {confidence:.2f})")
            
            return {
                "transcript": transcript,
                "language": language,
                "confidence": confidence,
                "processingTimeMs": processing_time,
                "durationSeconds": duration_seconds
            }
            
        except Exception as e:
            logger.error(f"Transcription failed: {e}")
            return {
                "error": str(e),
                "code": "TRANSCRIPTION_FAILED"
            }

    async def synthesize_response(
        self,
        text: str,
        language: str = "en",
        voice: str = "narrator",
        speed: float = 1.0
    ) -> Tuple[Optional[bytes], Optional[dict]]:
        """
        Generate audio response using Edge-TTS (free)
        
        Args:
            text: Response text to synthesize
            language: "en" or "ur"
            voice: Voice profile (e.g., "narrator_en", "professor_ur")
            speed: Playback speed (0.5 - 2.0)
            
        Returns:
            (audio_bytes, metadata_dict) or (None, error_dict)
        """
        start_time = datetime.now()
        
        try:
            # Map language and voice to Edge-TTS voices
            voice_map = {
                ("en", "narrator"): "en-US-AriaNeural",
                ("en", "professor"): "en-US-GuyNeural",
                ("ur", "narrator"): "ur-PK-UzmaNeural",
                ("ur", "professor"): "ur-PK-AsadNeural",
            }
            
            tts_voice = voice_map.get((language, voice), "en-US-AriaNeural")
            
            # Clamp speed
            speed = max(0.5, min(2.0, speed))
            
            logger.info(f"Synthesizing ({language}, {tts_voice}, speed={speed}x): {text[:100]}...")
            
            # Generate audio with Edge-TTS
            audio_data = io.BytesIO()
            
            async def generate_audio():
                communicate = edge_tts.Communicate(
                    text=text,
                    voice=tts_voice,
                    rate=f"{(speed - 1.0) * 100:+.0f}%"  # Convert speed to rate
                )
                async for chunk in communicate.stream():
                    if chunk["type"] == "audio":
                        audio_data.write(chunk["data"])
            
            # Run async TTS
            asyncio.run(generate_audio())
            
            audio_bytes = audio_data.getvalue()
            
            if not audio_bytes:
                return None, {"error": "No audio generated", "code": "EMPTY_AUDIO"}
            
            processing_time = int((datetime.now() - start_time).total_seconds() * 1000)
            
            metadata = {
                "language": language,
                "voice": tts_voice,
                "speed": speed,
                "textLength": len(text),
                "audioLengthBytes": len(audio_bytes),
                "processingTimeMs": processing_time,
                "format": "mp3"
            }
            
            logger.info(f"Audio synthesized: {len(audio_bytes)} bytes, {processing_time}ms")
            
            return audio_bytes, metadata
            
        except Exception as e:
            logger.error(f"Audio synthesis failed: {e}")
            return None, {"error": str(e), "code": "SYNTHESIS_FAILED"}

    def _get_cache_key(self, text: str, language: str, voice: str, speed: float) -> str:
        """Generate cache key for audio"""
        content = f"{text}_{language}_{voice}_{speed}"
        return hashlib.md5(content.encode()).hexdigest()

    async def get_cached_audio(
        self,
        text: str,
        language: str,
        voice: str,
        speed: float
    ) -> Optional[bytes]:
        """Retrieve cached audio if exists and not expired"""
        cache_key = self._get_cache_key(text, language, voice, speed)
        cache_file = self.cache_dir / f"audio_{cache_key}.mp3"
        meta_file = self.cache_dir / f"audio_{cache_key}.json"
        
        if cache_file.exists() and meta_file.exists():
            try:
                with open(meta_file) as f:
                    meta = json.load(f)
                
                created = datetime.fromisoformat(meta["created_at"])
                age = datetime.now() - created
                
                # Keep cache for 7 days
                if age < timedelta(hours=168):
                    with open(cache_file, "rb") as f:
                        logger.info(f"Audio cache hit (age: {age.total_seconds():.0f}s)")
                        return f.read()
                else:
                    # Expire old cache
                    cache_file.unlink()
                    meta_file.unlink()
            except Exception as e:
                logger.warning(f"Cache read error: {e}")
        
        return None

    async def save_cached_audio(
        self,
        text: str,
        language: str,
        voice: str,
        speed: float,
        audio_bytes: bytes
    ) -> bool:
        """Save audio to cache"""
        try:
            cache_key = self._get_cache_key(text, language, voice, speed)
            cache_file = self.cache_dir / f"audio_{cache_key}.mp3"
            meta_file = self.cache_dir / f"audio_{cache_key}.json"
            
            # Save audio
            with open(cache_file, "wb") as f:
                f.write(audio_bytes)
            
            # Save metadata
            meta = {
                "created_at": datetime.now().isoformat(),
                "text_length": len(text),
                "audio_length_bytes": len(audio_bytes),
                "language": language,
                "voice": voice,
                "speed": speed
            }
            with open(meta_file, "w") as f:
                json.dump(meta, f)
            
            logger.info(f"Audio cached: {cache_key}")
            return True
        except Exception as e:
            logger.error(f"Cache save error: {e}")
            return False

    async def cleanup_old_cache(self, max_age_hours: int = 168):
        """Remove cached files older than max_age_hours"""
        try:
            cutoff_time = datetime.now() - timedelta(hours=max_age_hours)
            
            for meta_file in self.cache_dir.glob("audio_*.json"):
                try:
                    with open(meta_file) as f:
                        meta = json.load(f)
                    
                    created = datetime.fromisoformat(meta["created_at"])
                    if created < cutoff_time:
                        cache_file = meta_file.parent / meta_file.name.replace(".json", ".mp3")
                        meta_file.unlink()
                        if cache_file.exists():
                            cache_file.unlink()
                        logger.info(f"Cleaned old cache: {meta_file.name}")
                except Exception as e:
                    logger.warning(f"Error cleaning cache file {meta_file}: {e}")
        except Exception as e:
            logger.error(f"Cache cleanup error: {e}")


# Global instance
_voice_service: Optional[VoiceService] = None


def get_voice_service() -> VoiceService:
    """Get or create voice service instance"""
    global _voice_service
    if _voice_service is None:
        _voice_service = VoiceService()
    return _voice_service
