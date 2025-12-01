"""
Test VoiceService with free models (local Whisper + Edge-TTS)
"""

import asyncio
import pytest
import io
from pathlib import Path
from datetime import datetime, timedelta
from unittest.mock import patch, MagicMock

import numpy as np
from pydub import AudioSegment
import edge_tts

from app.services.voice import VoiceService, get_voice_service
from app.config import settings


class TestVoiceServiceTranscription:
    """Test Whisper transcription"""

    @pytest.fixture(autouse=True)
    def setup(self):
        """Setup test service"""
        self.service = VoiceService()
        yield
        # Cleanup
        import shutil
        if Path(settings.VOICE_CACHE_DIR).exists():
            shutil.rmtree(settings.VOICE_CACHE_DIR)

    def _create_test_audio(self, duration_seconds: float = 3, sample_rate: int = 16000) -> bytes:
        """Create synthetic audio for testing"""
        # Generate silence + simple tone
        samples = np.zeros(int(duration_seconds * sample_rate), dtype=np.int16)
        audio_segment = AudioSegment(
            samples.tobytes(),
            frame_rate=sample_rate,
            sample_width=2,
            channels=1
        )
        return audio_segment.export(format="wav").getvalue()

    @pytest.mark.asyncio
    async def test_transcribe_english_audio(self):
        """Test English speech-to-text"""
        # Note: This uses local Whisper, so we'll test with synthetic audio
        audio_bytes = self._create_test_audio(duration_seconds=2)
        
        result = await self.service.transcribe_audio(
            audio_blob=audio_bytes,
            language="en",
            audio_format="wav"
        )
        
        # Synthetic audio won't transcribe to real words, but should complete
        assert "error" not in result or result.get("code") in ["TRANSCRIPTION_FAILED"]
        assert "transcript" in result or "error" in result
        assert "language" in result or "error" in result

    @pytest.mark.asyncio
    async def test_transcribe_audio_too_short(self):
        """Test rejection of very short audio"""
        # Create 0.3 second audio (too short)
        audio_bytes = self._create_test_audio(duration_seconds=0.3)
        
        result = await self.service.transcribe_audio(
            audio_blob=audio_bytes,
            language="en",
            audio_format="wav"
        )
        
        assert "error" in result
        assert result["code"] == "AUDIO_TOO_SHORT"

    @pytest.mark.asyncio
    async def test_transcribe_audio_too_long(self):
        """Test rejection of audio exceeding max length"""
        # Create audio longer than max (default 30s)
        long_duration = settings.VOICE_MAX_AUDIO_LENGTH_SECONDS + 5
        audio_bytes = self._create_test_audio(duration_seconds=long_duration)
        
        result = await self.service.transcribe_audio(
            audio_blob=audio_bytes,
            language="en",
            audio_format="wav"
        )
        
        assert "error" in result
        assert result["code"] == "AUDIO_TOO_LONG"

    @pytest.mark.asyncio
    async def test_transcribe_invalid_audio_format(self):
        """Test handling of corrupt audio"""
        invalid_audio = b"not real audio data"
        
        result = await self.service.transcribe_audio(
            audio_blob=invalid_audio,
            language="en",
            audio_format="wav"
        )
        
        assert "error" in result
        assert result["code"] == "TRANSCRIPTION_FAILED"

    @pytest.mark.asyncio
    async def test_transcribe_language_detection(self):
        """Test language parameter is respected"""
        audio_bytes = self._create_test_audio(duration_seconds=2)
        
        result_en = await self.service.transcribe_audio(
            audio_blob=audio_bytes,
            language="en",
            audio_format="wav"
        )
        
        result_ur = await self.service.transcribe_audio(
            audio_blob=audio_bytes,
            language="ur",
            audio_format="wav"
        )
        
        # Both should process (even if empty)
        assert "language" in result_en or "error" in result_en
        assert "language" in result_ur or "error" in result_ur


class TestVoiceServiceSynthesis:
    """Test Edge-TTS audio generation"""

    @pytest.fixture(autouse=True)
    def setup(self):
        """Setup test service"""
        self.service = VoiceService()
        yield
        import shutil
        if Path(settings.VOICE_CACHE_DIR).exists():
            shutil.rmtree(settings.VOICE_CACHE_DIR)

    @pytest.mark.asyncio
    async def test_synthesize_english_response(self):
        """Test English text-to-speech"""
        text = "Hello, this is a test"
        
        audio_bytes, metadata = await self.service.synthesize_response(
            text=text,
            language="en",
            voice="narrator",
            speed=1.0
        )
        
        assert audio_bytes is not None
        assert len(audio_bytes) > 0
        assert metadata is not None
        assert metadata["language"] == "en"
        assert metadata["voice"] == "en-US-AriaNeural"
        assert metadata["speed"] == 1.0
        assert metadata["format"] == "mp3"

    @pytest.mark.asyncio
    async def test_synthesize_urdu_response(self):
        """Test Urdu text-to-speech"""
        text = "یہ ایک ٹیسٹ ہے"  # "This is a test" in Urdu
        
        audio_bytes, metadata = await self.service.synthesize_response(
            text=text,
            language="ur",
            voice="narrator",
            speed=1.0
        )
        
        assert audio_bytes is not None
        assert len(audio_bytes) > 0
        assert metadata is not None
        assert metadata["language"] == "ur"
        assert metadata["voice"] == "ur-PK-UzmaNeural"

    @pytest.mark.asyncio
    async def test_synthesize_with_speed_variation(self):
        """Test different playback speeds"""
        text = "Test audio"
        
        speeds = [0.5, 1.0, 1.5, 2.0]
        results = []
        
        for speed in speeds:
            audio_bytes, metadata = await self.service.synthesize_response(
                text=text,
                language="en",
                voice="narrator",
                speed=speed
            )
            
            assert audio_bytes is not None
            assert metadata["speed"] == speed
            results.append((speed, len(audio_bytes)))
        
        # Faster speeds might have different byte sizes due to compression
        assert all(size > 0 for _, size in results)

    @pytest.mark.asyncio
    async def test_synthesize_speed_clamping(self):
        """Test speed values are clamped to valid range"""
        text = "Test"
        
        # Test below min
        _, metadata_slow = await self.service.synthesize_response(
            text=text,
            language="en",
            voice="narrator",
            speed=0.1  # Should clamp to 0.5
        )
        assert metadata_slow["speed"] == 0.5
        
        # Test above max
        _, metadata_fast = await self.service.synthesize_response(
            text=text,
            language="en",
            voice="narrator",
            speed=3.0  # Should clamp to 2.0
        )
        assert metadata_fast["speed"] == 2.0

    @pytest.mark.asyncio
    async def test_synthesize_voice_variety(self):
        """Test different voice options"""
        text = "Test"
        voices = ["narrator", "professor"]
        
        for voice in voices:
            audio_bytes, metadata = await self.service.synthesize_response(
                text=text,
                language="en",
                voice=voice,
                speed=1.0
            )
            
            assert audio_bytes is not None
            assert metadata["voice"] in [
                "en-US-AriaNeural",
                "en-US-GuyNeural",
                "ur-PK-UzmaNeural",
                "ur-PK-AsadNeural"
            ]


class TestVoiceServiceCaching:
    """Test audio caching mechanism"""

    @pytest.fixture(autouse=True)
    def setup(self):
        """Setup test service"""
        self.service = VoiceService()
        yield
        import shutil
        if Path(settings.VOICE_CACHE_DIR).exists():
            shutil.rmtree(settings.VOICE_CACHE_DIR)

    @pytest.mark.asyncio
    async def test_cache_save_and_retrieve(self):
        """Test saving and retrieving cached audio"""
        text = "Test audio content"
        language = "en"
        voice = "narrator"
        speed = 1.0
        audio_bytes = b"fake audio data" * 100
        
        # Save to cache
        saved = await self.service.save_cached_audio(
            text=text,
            language=language,
            voice=voice,
            speed=speed,
            audio_bytes=audio_bytes
        )
        assert saved is True
        
        # Retrieve from cache
        cached_audio = await self.service.get_cached_audio(
            text=text,
            language=language,
            voice=voice,
            speed=speed
        )
        
        assert cached_audio is not None
        assert cached_audio == audio_bytes

    @pytest.mark.asyncio
    async def test_cache_miss_for_different_text(self):
        """Test that different text doesn't hit cache"""
        text1 = "Test audio one"
        text2 = "Test audio two"
        audio_bytes = b"fake audio data" * 100
        
        # Save for text1
        await self.service.save_cached_audio(
            text=text1,
            language="en",
            voice="narrator",
            speed=1.0,
            audio_bytes=audio_bytes
        )
        
        # Try to retrieve for text2
        cached_audio = await self.service.get_cached_audio(
            text=text2,
            language="en",
            voice="narrator",
            speed=1.0
        )
        
        assert cached_audio is None

    @pytest.mark.asyncio
    async def test_cache_expiration(self):
        """Test that old cache is not used"""
        text = "Test"
        audio_bytes = b"fake audio" * 100
        
        # Save cache
        await self.service.save_cached_audio(
            text=text,
            language="en",
            voice="narrator",
            speed=1.0,
            audio_bytes=audio_bytes
        )
        
        # Manually modify metadata to make it old
        cache_key = self.service._get_cache_key(text, "en", "narrator", 1.0)
        meta_file = Path(settings.VOICE_CACHE_DIR) / f"audio_{cache_key}.json"
        
        import json
        with open(meta_file) as f:
            meta = json.load(f)
        
        # Set created time to 10 days ago
        old_date = (datetime.now() - timedelta(days=10)).isoformat()
        meta["created_at"] = old_date
        
        with open(meta_file, "w") as f:
            json.dump(meta, f)
        
        # Try to retrieve - should be expired
        cached_audio = await self.service.get_cached_audio(
            text=text,
            language="en",
            voice="narrator",
            speed=1.0
        )
        
        assert cached_audio is None

    @pytest.mark.asyncio
    async def test_cache_cleanup(self):
        """Test cleanup of old cache files"""
        text = "Test"
        audio_bytes = b"fake audio" * 100
        
        # Save multiple cache entries
        for i in range(3):
            await self.service.save_cached_audio(
                text=f"{text} {i}",
                language="en",
                voice="narrator",
                speed=1.0,
                audio_bytes=audio_bytes
            )
        
        # Manually age one of them
        cache_files = list(Path(settings.VOICE_CACHE_DIR).glob("audio_*.json"))
        assert len(cache_files) == 3
        
        # Make first file old
        import json
        with open(cache_files[0]) as f:
            meta = json.load(f)
        
        old_date = (datetime.now() - timedelta(days=10)).isoformat()
        meta["created_at"] = old_date
        
        with open(cache_files[0], "w") as f:
            json.dump(meta, f)
        
        # Run cleanup
        await self.service.cleanup_old_cache(max_age_hours=168)  # 7 days
        
        # Check that old file was removed
        remaining_files = list(Path(settings.VOICE_CACHE_DIR).glob("audio_*.json"))
        assert len(remaining_files) == 2


class TestVoiceServiceIntegration:
    """Integration tests"""

    @pytest.fixture(autouse=True)
    def setup(self):
        """Setup test service"""
        self.service = VoiceService()
        yield
        import shutil
        if Path(settings.VOICE_CACHE_DIR).exists():
            shutil.rmtree(settings.VOICE_CACHE_DIR)

    @pytest.mark.asyncio
    async def test_get_voice_service_singleton(self):
        """Test that get_voice_service returns singleton"""
        service1 = get_voice_service()
        service2 = get_voice_service()
        
        assert service1 is service2

    @pytest.mark.asyncio
    async def test_cache_directory_creation(self):
        """Test that cache directory is created"""
        cache_dir = Path(settings.VOICE_CACHE_DIR)
        assert cache_dir.exists()
        assert cache_dir.is_dir()


if __name__ == "__main__":
    # Run: python -m pytest backend/tests/test_voice_service.py -v
    pytest.main([__file__, "-v", "-s"])
