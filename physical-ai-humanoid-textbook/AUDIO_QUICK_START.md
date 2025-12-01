# OpenVoice Audio Integration - Quick Start Guide

## Overview
Add text-to-speech audio narration to chapters with speaker selection, speed control, and multilingual support (English & Urdu).

## Quick Implementation (< 2 hours)

### Step 1: Install Dependencies

**Backend:**
```bash
cd backend
pip install edge-tts pydub librosa
# or for local OpenVoice:
# pip install openvoice
```

**Frontend:**
```bash
cd ..
npm install react-h5-audio-player
```

### Step 2: Create Audio Service (Quick Version)

**`backend/app/services/audio_simple.py`** - Use free Edge-TTS for quick start:

```python
import asyncio
from edge_tts import Communicate
from pydub import AudioSegment
from io import BytesIO

class AudioService:
    """Simple audio service using free Edge-TTS."""
    
    async def generate_audio(
        self,
        text: str,
        language: str = "en",
        speed: float = 1.0
    ) -> bytes:
        """Generate MP3 audio from text."""
        # Map language to voice
        voice_map = {
            "en": "en-US-AriaNeural",      # Female English
            "ur": "ur-PK-AsadNeural"       # Male Urdu
        }
        
        voice = voice_map.get(language, voice_map["en"])
        
        # Use edge-tts (free, no API key needed)
        communicate = Communicate(
            text=text,
            voice=voice,
            rate=f"{int((speed - 1) * 50):+d}%"  # Speed as percentage
        )
        
        # Collect audio chunks
        audio_data = BytesIO()
        async for chunk in communicate.stream():
            if chunk["type"] == "audio":
                audio_data.write(chunk["data"])
        
        audio_data.seek(0)
        return audio_data.read()
```

### Step 3: Create Audio Router

**`backend/app/routers/audio.py`:**

```python
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from app.services.audio_simple import AudioService

router = APIRouter(prefix="/api/audio", tags=["audio"])
audio_service = AudioService()

class AudioRequest(BaseModel):
    text: str
    language: str = "en"
    speed: float = 1.0
    chapter_id: str = ""

@router.post("/generate")
async def generate_audio(request: AudioRequest):
    """Generate audio from text."""
    try:
        audio_bytes = await audio_service.generate_audio(
            text=request.text,
            language=request.language,
            speed=request.speed
        )
        
        return StreamingResponse(
            iter([audio_bytes]),
            media_type="audio/mpeg"
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/speakers")
async def get_speakers():
    """Available voices."""
    return {
        "en": [
            {"id": "en-US-AriaNeural", "name": "Aria (Female)"},
            {"id": "en-US-GuyNeural", "name": "Guy (Male)"}
        ],
        "ur": [
            {"id": "ur-PK-AsadNeural", "name": "Asad (Male)"},
            {"id": "ur-PK-UzmaNeural", "name": "Uzma (Female)"}
        ]
    }
```

### Step 4: Register Router in Main App

**`backend/app/main.py`:**

```python
from app.routers import audio

# ... existing code ...

app.include_router(audio.router)
```

### Step 5: Create Audio Player Component

**`src/components/AudioPlayer.tsx`:**

```tsx
import React, { useState, useRef } from 'react';

interface AudioPlayerProps {
  content: string;
  language: 'en' | 'ur';
  chapterId: string;
}

export function AudioPlayer({ content, language, chapterId }: AudioPlayerProps) {
  const audioRef = useRef<HTMLAudioElement>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [isPlaying, setIsPlaying] = useState(false);
  const [speed, setSpeed] = useState(1.0);

  const handleGenerateAudio = async () => {
    setIsLoading(true);
    try {
      const response = await fetch('http://localhost:8000/api/audio/generate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          text: content,
          language,
          speed,
          chapter_id: chapterId
        })
      });

      const blob = await response.blob();
      const url = URL.createObjectURL(blob);
      if (audioRef.current) {
        audioRef.current.src = url;
        audioRef.current.play();
        setIsPlaying(true);
      }
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div style={{
      background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
      padding: '1.5rem',
      borderRadius: '12px',
      marginBottom: '1.5rem',
      color: 'white'
    }}>
      <div style={{ display: 'flex', gap: '1rem', alignItems: 'center' }}>
        <button
          onClick={handleGenerateAudio}
          disabled={isLoading}
          style={{
            padding: '0.75rem 1.5rem',
            borderRadius: '8px',
            border: 'none',
            background: 'white',
            color: '#667eea',
            fontWeight: 'bold',
            cursor: isLoading ? 'not-allowed' : 'pointer'
          }}
        >
          {isLoading ? '🎙️ Generating...' : isPlaying ? '⏸️ Pause' : '▶️ Listen'}
        </button>

        <select value={speed} onChange={(e) => {
          const newSpeed = parseFloat(e.target.value);
          setSpeed(newSpeed);
          if (audioRef.current) audioRef.current.playbackRate = newSpeed;
        }} style={{
          padding: '0.5rem',
          borderRadius: '6px',
          border: '2px solid white',
          background: 'rgba(255, 255, 255, 0.1)',
          color: 'white'
        }}>
          <option value="0.75">0.75x</option>
          <option value="1">1.0x</option>
          <option value="1.25">1.25x</option>
          <option value="1.5">1.5x</option>
          <option value="2">2.0x</option>
        </select>
      </div>

      <audio
        ref={audioRef}
        onPlay={() => setIsPlaying(true)}
        onPause={() => setIsPlaying(false)}
        controls
        style={{ width: '100%', marginTop: '1rem', filter: 'invert(1)' }}
      />
    </div>
  );
}
```

### Step 6: Integrate into PersonalizedChapter

**`src/components/PersonalizedChapter.tsx`:**

```tsx
import { AudioPlayer } from './AudioPlayer';

export function PersonalizedChapter({ id, children }: PersonalizedChapterProps) {
  const [showAudio, setShowAudio] = useState(false);

  // ... existing code ...

  return (
    <div className="personalized-chapter">
      <div className="chapter-header flex items-center justify-between p-4 bg-blue-50 dark:bg-gray-800 rounded-lg mb-6">
        <div className="flex items-center gap-3">
          {/* Existing buttons */}
        </div>

        <div className="flex gap-2">
          {/* Existing buttons */}
          
          <button
            onClick={() => setShowAudio(!showAudio)}
            className="audio-btn px-3 py-2 bg-purple-600 text-white rounded-lg hover:bg-purple-700 transition-all text-sm font-semibold"
            title="Listen to chapter audio"
          >
            {showAudio ? '🎧 Hide Audio' : '🎧 Listen'}
          </button>
        </div>
      </div>

      {showAudio && (
        <AudioPlayer
          content={extractTextContent(children)}
          language={isUrdu ? 'ur' : 'en'}
          chapterId={id}
        />
      )}

      {/* Existing chapter content */}
      <div className={`chapter-content ${isUrdu ? 'urdu-text' : ''}`}>
        {isUrdu && urduContent ? (
          <div dangerouslySetInnerHTML={{ __html: urduContent }} />
        ) : (
          <div className="prose dark:prose-invert max-w-none">
            {children}
          </div>
        )}
      </div>
    </div>
  );
}
```

### Step 7: Update Main App Router

**`backend/app/main.py`:**

```python
from fastapi.middleware.cors import CORSMiddleware
from app.routers import audio, translate, chat

# ... existing setup ...

# Add CORS for audio streaming
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register routers
app.include_router(audio.router)
app.include_router(translate.router)
app.include_router(chat.router)
```

---

## Testing

### Manual Test:

```bash
# Terminal 1: Start backend
cd backend
source venv/bin/activate
python -m uvicorn app.main:app --reload

# Terminal 2: Test API
curl -X POST http://localhost:8000/api/audio/generate \
  -H "Content-Type: application/json" \
  -d '{
    "text": "Hello, this is a test of the audio system",
    "language": "en",
    "speed": 1.0
  }' \
  --output test.mp3

# Open test.mp3 in your browser or media player
```

### Frontend Test:

```bash
# Terminal 3: Start frontend
npm start
# Navigate to any chapter and click the 🎧 Listen button
```

---

## Production Checklist

- [ ] Switch from Edge-TTS to OpenVoice (if needed)
- [ ] Add audio caching to PostgreSQL
- [ ] Upload MP3s to S3/GCS for CDN
- [ ] Implement audio streaming with chunking
- [ ] Add analytics for audio feature usage
- [ ] Test with slow network (throttle to 3G)
- [ ] Verify Urdu pronunciation quality
- [ ] Add fallback to text if audio unavailable
- [ ] Monitor API latency and costs
- [ ] User A/B test (with/without audio)

---

## Deployment (Docker)

**`backend/Dockerfile`** - Add audio dependencies:

```dockerfile
FROM python:3.12-slim

# Install audio libraries
RUN apt-get update && apt-get install -y \
    ffmpeg \
    libsndfile1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY . .

CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

---

## Cost Estimation (Monthly)

| Service | Cost | Notes |
|---------|------|-------|
| Edge-TTS (free) | $0 | Recommended for MVP |
| OpenVoice GPU | ~$50-200 | Self-hosted or API |
| S3 Storage | ~$5 | For audio MP3s |
| Bandwidth | ~$10 | Audio streaming |
| **Total** | **$15-215** | Depends on scale |

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Audio cuts off | Increase buffer size in streaming |
| Urdu sounds weird | Use native Urdu voice (ur-PK) |
| Slow generation | Use streaming + caching |
| CORS errors | Add frontend URL to CORS origins |
| No audio output | Check browser permissions |

---

## Next: Voice Cloning (Advanced)

Once basic audio works, upgrade to OpenVoice voice cloning:

```python
from openvoice import ToneConverter, BaseVoiceConverter

# Clone a specific speaker's voice
converter = BaseVoiceConverter(
    target_se="./speaker_embedding.pth",
    output_dir="./output"
)

converter.convert(
    audio_path="./input.wav",
    reference_se="./reference_speaker.wav"
)
```

---

**Start with Edge-TTS, scale to OpenVoice as you grow. This gives you 80% of the value with 20% of the effort.**
