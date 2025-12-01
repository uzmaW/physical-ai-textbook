# OpenVoice Integration Plan - Audiobook & Multilingual Support

## Executive Summary
Integrate **MyShell OpenVoice** to transform the textbook into an **interactive audiobook** with:
- 🎙️ Text-to-speech for English & Urdu content
- 🗣️ Voice cloning for consistent narrator personality
- 🌍 Multilingual narration (EN/UR with proper tone & emotion)
- 📚 Chapter-based audio generation & caching
- 🎧 Audio playback integration in UI
- 🔊 Speaker selection (male/female/custom voices)

---

## Why OpenVoice?

| Feature | Benefit |
|---------|---------|
| **Voice Cloning** | Consistent narrator voice across all chapters |
| **Multilingual** | Native support for Urdu + English |
| **Zero-Shot** | Convert any speech to target speaker style |
| **Emotion Control** | Adjust tone for educational content (calm, enthusiastic, formal) |
| **Fast Inference** | Real-time or near-real-time audio generation |
| **Open Source** | MIT licensed, can be self-hosted |

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│         Frontend (React/TypeScript)                      │
│  PersonalizedChapter → Audio Playback Widget            │
│  ✓ Play/Pause/Speed controls                            │
│  ✓ Language selection (EN/UR)                           │
│  ✓ Speaker selection                                    │
└────────────────┬────────────────────────────────────────┘
                 │
         ┌───────▼──────────┐
         │  API Gateway     │
         │ /api/audio/*     │
         └───────┬──────────┘
                 │
┌────────────────┴──────────────────────────────────────────┐
│         Backend (FastAPI)                                 │
├──────────────────────────────────────────────────────────┤
│  1. Audio Cache Router (/audio/cached)                   │
│  2. Audio Generation Service (OpenVoice)                 │
│  3. Multi-voice Support (speaker registry)               │
│  4. Audio Streaming                                      │
└────────────────┬─────────────────────────────────────────┘
                 │
    ┌────────────┼────────────┐
    │            │            │
┌───▼────┐  ┌───▼────┐  ┌───▼────┐
│OpenVoice│  │PostgreSQL   │ S3/GCS
│(TTS)    │  │Audio Cache  │ Audio
└────────┘  └────────┘   Storage
```

---

## Phase 1: Backend Audio Service

### 1.1 Create Audio Service (`backend/app/services/audio.py`)

```python
from typing import Optional, Dict
from enum import Enum
import asyncio
from pathlib import Path
import tempfile
import subprocess

class Speaker(str, Enum):
    """Available speakers/voices."""
    PROFESSOR_EN = "professor_en"      # Male, formal, English
    NARRATOR_EN = "narrator_en"        # Female, warm, English  
    PROFESSOR_UR = "professor_ur"      # Male, formal, Urdu
    NARRATOR_UR = "narrator_ur"        # Female, warm, Urdu
    FAST = "fast"                       # High energy

class AudioService:
    """OpenVoice-based audio generation service."""
    
    def __init__(self):
        self.model_path = Path("models/openvoice")
        self.speakers_path = Path("models/openvoice/speakers")
        self.temp_dir = Path(tempfile.gettempdir()) / "openvoice"
        self.temp_dir.mkdir(exist_ok=True)
        
    async def generate_audio(
        self,
        text: str,
        language: str = "en",
        speaker: Speaker = Speaker.NARRATOR_EN,
        speed: float = 1.0,
        pitch: float = 1.0
    ) -> bytes:
        """
        Generate audio from text using OpenVoice.
        
        Args:
            text: Content to narrate
            language: "en" or "ur"
            speaker: Voice profile
            speed: 0.5-2.0 (playback speed)
            pitch: 0.5-2.0 (pitch adjustment)
            
        Returns:
            Audio bytes (MP3)
        """
        try:
            # Hash the input to create cache key
            cache_key = self._get_cache_key(text, language, speaker)
            
            # Call OpenVoice TTS
            audio_bytes = await self._call_openvoice(
                text=text,
                language=language,
                speaker=speaker,
                speed=speed,
                pitch=pitch
            )
            
            return audio_bytes
            
        except Exception as e:
            raise AudioGenerationError(f"Failed to generate audio: {e}")
    
    async def _call_openvoice(
        self,
        text: str,
        language: str,
        speaker: Speaker,
        speed: float,
        pitch: float
    ) -> bytes:
        """
        Call OpenVoice inference engine.
        
        Can be:
        - Local inference (CPU/GPU)
        - Remote API call to MyShell/custom server
        """
        # Implementation depends on deployment:
        # Option A: Use openvoice Python library directly
        # Option B: Call external OpenVoice API
        # Option C: Use Azure/Google TTS with voice conversion
        
        pass
    
    def _get_cache_key(self, text: str, language: str, speaker: Speaker) -> str:
        """Generate cache key for audio."""
        import hashlib
        key = f"{language}:{speaker}:{text[:100]}"
        return hashlib.md5(key.encode()).hexdigest()
```

### 1.2 Audio Cache Model (`backend/app/models/user.py`)

```python
from sqlalchemy import Column, String, LargeBinary, DateTime, Float
from datetime import datetime

class AudioCache(Base):
    """Cache for generated audio files."""
    __tablename__ = "audio_cache"
    
    id = Column(String, primary_key=True)  # hash(content+speaker+lang)
    chapter_id = Column(String, index=True)
    user_id = Column(String)
    
    # Content metadata
    text_content = Column(String)
    language = Column(String)  # "en" or "ur"
    speaker = Column(String)   # voice profile
    
    # Audio metadata
    audio_format = Column(String, default="mp3")
    duration_seconds = Column(Float)
    file_size_bytes = Column(Integer)
    
    # S3/GCS reference (store audio in cloud)
    audio_url = Column(String)  # S3 signed URL
    
    # Timestamps
    created_at = Column(DateTime, default=datetime.utcnow)
    accessed_at = Column(DateTime, default=datetime.utcnow)
    
    # Quality metrics
    generation_time_ms = Column(Float)  # How long it took to generate
```

### 1.3 Audio Router (`backend/app/routers/audio.py`)

```python
from fastapi import APIRouter, HTTPException, Depends
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from sqlalchemy.orm import Session
from app.services.audio import AudioService, Speaker
from app.db.neon import get_db

router = APIRouter(prefix="/api/audio", tags=["audio"])
audio_service = AudioService()

class AudioRequest(BaseModel):
    """Request model for audio generation."""
    text: str
    language: str = "en"  # "en" or "ur"
    speaker: Speaker = Speaker.NARRATOR_EN
    speed: float = 1.0
    pitch: float = 1.0
    chapter_id: str = ""
    user_id: str = ""

@router.post("/generate")
async def generate_audio(
    request: AudioRequest,
    db: Session = Depends(get_db)
) -> StreamingResponse:
    """
    Generate audio from text.
    
    Returns:
        MP3 stream for playback
    """
    try:
        # Check cache first
        cache_key = audio_service._get_cache_key(
            request.text, request.language, request.speaker
        )
        
        cached = db.query(AudioCache).filter(
            AudioCache.id == cache_key
        ).first()
        
        if cached and cached.audio_url:
            # Return cached audio URL
            return {
                "url": cached.audio_url,
                "cached": True,
                "duration": cached.duration_seconds
            }
        
        # Generate new audio
        audio_bytes = await audio_service.generate_audio(
            text=request.text,
            language=request.language,
            speaker=request.speaker,
            speed=request.speed,
            pitch=request.pitch
        )
        
        # Cache the result
        cache_entry = AudioCache(
            id=cache_key,
            chapter_id=request.chapter_id,
            user_id=request.user_id,
            text_content=request.text,
            language=request.language,
            speaker=request.speaker.value,
            audio_format="mp3",
            file_size_bytes=len(audio_bytes)
        )
        
        db.add(cache_entry)
        db.commit()
        
        # Stream audio
        return StreamingResponse(
            iter([audio_bytes]),
            media_type="audio/mpeg"
        )
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/speakers")
async def get_speakers():
    """List available speakers/voices."""
    return {
        "english": [
            {"id": "professor_en", "name": "Professor (Male)", "language": "en"},
            {"id": "narrator_en", "name": "Narrator (Female)", "language": "en"}
        ],
        "urdu": [
            {"id": "professor_ur", "name": "استاد (Male)", "language": "ur"},
            {"id": "narrator_ur", "name": "نارریٹر (Female)", "language": "ur"}
        ]
    }
```

---

## Phase 2: Frontend Audio Widget

### 2.1 Audio Playback Component (`src/components/AudioPlayer.tsx`)

```tsx
import React, { useState, useRef } from 'react';
import styles from './AudioPlayer.module.css';

interface AudioPlayerProps {
  content: string;
  language: 'en' | 'ur';
  chapterId: string;
  defaultSpeaker?: string;
}

export function AudioPlayer({ content, language, chapterId, defaultSpeaker }: AudioPlayerProps) {
  const audioRef = useRef<HTMLAudioElement>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [isPlaying, setIsPlaying] = useState(false);
  const [speed, setSpeed] = useState(1.0);
  const [speaker, setSpeaker] = useState(defaultSpeaker || 'narrator_en');
  const [duration, setDuration] = useState(0);
  const [currentTime, setCurrentTime] = useState(0);

  const handleGenerateAudio = async () => {
    setIsLoading(true);
    try {
      const response = await fetch('http://localhost:8000/api/audio/generate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          text: content,
          language,
          speaker,
          chapter_id: chapterId
        })
      });

      if (response.ok) {
        const blob = await response.blob();
        const url = URL.createObjectURL(blob);
        if (audioRef.current) {
          audioRef.current.src = url;
          audioRef.current.play();
          setIsPlaying(true);
        }
      }
    } catch (error) {
      console.error('Audio generation failed:', error);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.audioPlayer}>
      <div className={styles.controls}>
        <button
          onClick={handleGenerateAudio}
          disabled={isLoading}
          className={styles.playBtn}
        >
          {isLoading ? '🎙️ Generating...' : isPlaying ? '⏸️ Pause' : '▶️ Listen'}
        </button>

        <select
          value={speaker}
          onChange={(e) => setSpeaker(e.target.value)}
          className={styles.speakerSelect}
        >
          <option value="narrator_en">Narrator (Female)</option>
          <option value="professor_en">Professor (Male)</option>
          {language === 'ur' && (
            <>
              <option value="narrator_ur">نارریٹر (خاتون)</option>
              <option value="professor_ur">استاد (آدمی)</option>
            </>
          )}
        </select>

        <input
          type="range"
          min="0.5"
          max="2"
          step="0.1"
          value={speed}
          onChange={(e) => {
            setSpeed(parseFloat(e.target.value));
            if (audioRef.current) audioRef.current.playbackRate = parseFloat(e.target.value);
          }}
          className={styles.speedSlider}
        />
        <span className={styles.speedLabel}>{speed.toFixed(1)}x</span>
      </div>

      <audio
        ref={audioRef}
        onTimeUpdate={() => setCurrentTime(audioRef.current?.currentTime || 0)}
        onLoadedMetadata={() => setDuration(audioRef.current?.duration || 0)}
        onPlay={() => setIsPlaying(true)}
        onPause={() => setIsPlaying(false)}
      />

      <div className={styles.progress}>
        <input
          type="range"
          min="0"
          max={duration}
          value={currentTime}
          onChange={(e) => {
            if (audioRef.current) audioRef.current.currentTime = parseFloat(e.target.value);
          }}
          className={styles.progressBar}
        />
        <span className={styles.timeDisplay}>
          {Math.floor(currentTime)}s / {Math.floor(duration)}s
        </span>
      </div>
    </div>
  );
}
```

### 2.2 Integration in PersonalizedChapter

```tsx
// In src/components/PersonalizedChapter.tsx

import { AudioPlayer } from './AudioPlayer';

export function PersonalizedChapter({ id, children }: PersonalizedChapterProps) {
  // ... existing code ...
  
  return (
    <div className="personalized-chapter">
      {/* Existing header */}
      <div className="chapter-header">
        {/* Existing buttons */}
        
        <button
          onClick={() => setShowAudio(!showAudio)}
          className="audio-btn"
          title="Listen to chapter"
        >
          🎧 Audio
        </button>
      </div>

      {/* Audio Player (when toggled) */}
      {showAudio && (
        <AudioPlayer
          content={extractTextContent(children)}
          language={isUrdu ? 'ur' : 'en'}
          chapterId={id}
          defaultSpeaker={isUrdu ? 'narrator_ur' : 'narrator_en'}
        />
      )}

      {/* Existing content */}
      <div className="chapter-content">
        {/* ... */}
      </div>
    </div>
  );
}
```

### 2.3 Audio Player Styles (`src/components/AudioPlayer.module.css`)

```css
.audioPlayer {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  border-radius: 12px;
  padding: 1.5rem;
  margin: 1.5rem 0;
  box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
  color: white;
}

.controls {
  display: flex;
  gap: 1rem;
  align-items: center;
  margin-bottom: 1rem;
  flex-wrap: wrap;
}

.playBtn {
  background: white;
  color: #667eea;
  border: none;
  padding: 0.75rem 1.5rem;
  border-radius: 8px;
  font-weight: bold;
  cursor: pointer;
  transition: transform 0.2s;
}

.playBtn:hover:not(:disabled) {
  transform: scale(1.05);
}

.playBtn:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.speakerSelect {
  padding: 0.5rem 1rem;
  border-radius: 6px;
  border: 2px solid white;
  background: rgba(255, 255, 255, 0.1);
  color: white;
  font-weight: 500;
}

.speakerSelect option {
  background: #667eea;
  color: white;
}

.speedSlider {
  width: 120px;
  cursor: pointer;
}

.speedLabel {
  font-weight: bold;
  min-width: 50px;
}

.progress {
  display: flex;
  gap: 1rem;
  align-items: center;
}

.progressBar {
  flex: 1;
  cursor: pointer;
  height: 6px;
  border-radius: 3px;
  background: rgba(255, 255, 255, 0.3);
}

.progressBar::-webkit-slider-thumb {
  width: 16px;
  height: 16px;
  border-radius: 50%;
  background: white;
  cursor: pointer;
}

.timeDisplay {
  font-size: 0.9rem;
  min-width: 80px;
}

@media (max-width: 768px) {
  .audioPlayer {
    padding: 1rem;
  }

  .controls {
    flex-direction: column;
    align-items: stretch;
  }

  .speedSlider {
    width: 100%;
  }
}
```

---

## Phase 3: Deployment Options

### Option A: Local OpenVoice (GPU)
```bash
# Install
pip install openvoice

# Run inference server
python -m openvoice.server --device cuda
```

### Option B: MyShell API (Cloud)
```python
import openvoice_client

client = openvoice_client.OpenVoiceClient(
    api_key=settings.MYSHELL_API_KEY
)

audio = client.tts(
    text=text,
    language=language,
    speaker_id=speaker_id
)
```

### Option C: Azure Cognitive Services
```python
from azure.cognitiveservices.speech import speechconfig

config = speechconfig.SpeechConfig(
    subscription=settings.AZURE_SPEECH_KEY,
    region=settings.AZURE_REGION
)
synthesizer = speechsynthesizer.SpeechSynthesizer(speech_config=config)
```

---

## Phase 4: RAG Integration

### Enhanced Chat with Audio Context

```python
# In backend/app/services/rag.py

async def ask_with_audio(
    self,
    question: str,
    language: str = "en",
    include_audio: bool = False
) -> Dict:
    """RAG pipeline with optional audio narration."""
    
    # Existing RAG logic
    result = await self.ask(question, language=language)
    
    # Generate audio for answer if requested
    if include_audio:
        audio_service = AudioService()
        audio_bytes = await audio_service.generate_audio(
            text=result["answer"],
            language=language,
            speaker=Speaker.PROFESSOR_EN if language == "en" else Speaker.PROFESSOR_UR
        )
        result["audio_url"] = upload_to_storage(audio_bytes)
    
    return result
```

---

## Implementation Timeline

| Phase | Duration | Deliverable |
|-------|----------|------------|
| **1** | 1 week | Audio service, cache model, router |
| **2** | 1 week | Frontend widget, integration |
| **3** | 3 days | Deploy OpenVoice infrastructure |
| **4** | 2 days | RAG audio integration |
| **Testing** | 1 week | QA, performance tuning |

---

## Dependencies to Add

### Backend (`backend/requirements.txt`)
```
openvoice>=0.2.0          # or use API client
edge-tts>=6.1.0           # Alternative TTS (lighter)
pydub>=0.25.1            # Audio processing
librosa>=0.10.0          # Audio analysis
```

### Frontend (`package.json`)
```json
{
  "dependencies": {
    "react-h5-audio-player": "^3.8.2",
    "wavesurfer.js": "^6.6.4"
  }
}
```

---

## Performance Considerations

| Metric | Target | Strategy |
|--------|--------|----------|
| **First audio load** | < 5 sec | Cache aggressively |
| **Playback quality** | 128 kbps MP3 | Good quality/size ratio |
| **Storage** | < 2MB per chapter | Audio compression |
| **Concurrent users** | 100+ | Async generation + queue |

---

## Testing Checklist

- [ ] Audio generates for English text
- [ ] Audio generates for Urdu text
- [ ] Speaker selection works
- [ ] Speed control works (0.5x - 2.0x)
- [ ] Audio caches correctly
- [ ] Playback smooth on mobile
- [ ] Multiple concurrent requests handled
- [ ] RAG answers have audio available
- [ ] Fallback to text if audio fails
- [ ] Performance within SLA

---

## Success Metrics

- ✅ 95% of users can play chapter audio
- ✅ Audio generation < 3 sec (cached)
- ✅ 90% audio cache hit rate
- ✅ User engagement +40% (with audio)
- ✅ Accessibility score improves (WCAG AA)

---

## Next Steps

1. **Evaluate OpenVoice deployment** - Decide between local, API, or cloud TTS
2. **Implement audio service** - Start with Phase 1 backend
3. **Build frontend widget** - Simple play/pause/speed controls
4. **Add to Urdu chapters** - Test with translated content
5. **Measure impact** - Track user engagement with audio feature
