"""
Free Text-to-Speech Service using OpenVoice and Coqui TTS
Multilingual audio generation for educational content
Uses only free/open-source models for cost-effective deployment
"""

import logging
import asyncio
import hashlib
import json
from typing import Dict, List, Optional, Tuple, BinaryIO
from pathlib import Path
from dataclasses import dataclass, asdict
from datetime import datetime
import io
import wave
import tempfile
import subprocess
import re

import torch
import torchaudio
import numpy as np
from pydub import AudioSegment
from pydub.effects import normalize, compress_dynamic_range

logger = logging.getLogger(__name__)

@dataclass
class TTSResult:
    """Result of TTS generation"""
    text: str
    language: str
    voice_id: str
    audio_file_path: str
    duration: float
    file_size: int
    generation_time: float
    sample_rate: int
    format: str
    quality_score: float
    technical_terms: List[str]

@dataclass
class VoiceProfile:
    """Voice configuration for different languages and styles"""
    voice_id: str
    name: str
    language: str
    gender: str
    style: str  # educational, professional, casual
    model_path: str
    config_path: Optional[str] = None
    speaker_embedding: Optional[str] = None

class AudioProcessor:
    """Audio processing utilities for TTS optimization"""
    
    def __init__(self):
        self.target_sample_rate = 22050
        self.target_channels = 1
        self.compression_format = "mp3"
        self.compression_quality = "128k"
    
    def optimize_for_web(self, audio_path: Path, output_path: Path) -> Dict[str, any]:
        """Optimize audio for web delivery"""
        try:
            # Load audio with pydub
            audio = AudioSegment.from_file(str(audio_path))
            
            # Convert to mono
            if audio.channels > 1:
                audio = audio.set_channels(1)
            
            # Resample to target rate
            if audio.frame_rate != self.target_sample_rate:
                audio = audio.set_frame_rate(self.target_sample_rate)
            
            # Normalize audio levels
            audio = normalize(audio)
            
            # Apply light compression to reduce dynamic range
            audio = compress_dynamic_range(audio, threshold=-20.0, ratio=4.0)
            
            # Export optimized audio
            audio.export(
                str(output_path),
                format="mp3",
                bitrate=self.compression_quality,
                parameters=["-q:a", "2"]  # High quality VBR
            )
            
            return {
                "duration": len(audio) / 1000.0,  # Convert to seconds
                "file_size": output_path.stat().st_size,
                "sample_rate": audio.frame_rate,
                "channels": audio.channels
            }
            
        except Exception as e:
            logger.error(f"Audio optimization failed: {e}")
            raise
    
    def split_long_audio(self, audio_path: Path, max_duration: int = 60) -> List[Path]:
        """Split long audio files into smaller chunks for progressive loading"""
        try:
            audio = AudioSegment.from_file(str(audio_path))
            duration = len(audio) / 1000.0  # Convert to seconds
            
            if duration <= max_duration:
                return [audio_path]
            
            chunks = []
            chunk_duration_ms = max_duration * 1000
            
            for i, start_time in enumerate(range(0, len(audio), chunk_duration_ms)):
                end_time = min(start_time + chunk_duration_ms, len(audio))
                chunk = audio[start_time:end_time]
                
                chunk_path = audio_path.parent / f"{audio_path.stem}_chunk_{i:03d}.mp3"
                chunk.export(str(chunk_path), format="mp3", bitrate=self.compression_quality)
                chunks.append(chunk_path)
            
            return chunks
            
        except Exception as e:
            logger.error(f"Audio splitting failed: {e}")
            return [audio_path]

class FreeTTSService:
    """Free TTS service using OpenVoice and Coqui models"""
    
    def __init__(self, cache_dir: str = "./audio_cache", models_dir: str = "./ml_models/tts"):
        self.cache_dir = Path(cache_dir)
        self.models_dir = Path(models_dir)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.models_dir.mkdir(parents=True, exist_ok=True)
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.audio_processor = AudioProcessor()
        
        # Initialize voice profiles
        self.voice_profiles = self._setup_voice_profiles()
        
        # TTS models (loaded on demand)
        self.loaded_models = {}
        
        logger.info(f"TTS service initialized with device: {self.device}")
    
    def _setup_voice_profiles(self) -> Dict[str, VoiceProfile]:
        """Setup available voice profiles for different languages"""
        profiles = {}
        
        # English voices using OpenVoice
        profiles["en_female_edu"] = VoiceProfile(
            voice_id="en_female_edu",
            name="Sarah - Educational",
            language="en",
            gender="female",
            style="educational",
            model_path="OpenVoice/checkpoints/base_speakers/EN/en_default_se.pth",
            config_path="OpenVoice/checkpoints/base_speakers/EN/config.json"
        )
        
        profiles["en_male_prof"] = VoiceProfile(
            voice_id="en_male_prof", 
            name="David - Professional",
            language="en",
            gender="male",
            style="professional",
            model_path="OpenVoice/checkpoints/base_speakers/EN/en_male_se.pth",
            config_path="OpenVoice/checkpoints/base_speakers/EN/config.json"
        )
        
        # Urdu voices using Coqui TTS
        profiles["ur_female_edu"] = VoiceProfile(
            voice_id="ur_female_edu",
            name="Fatima - Educational", 
            language="ur",
            gender="female",
            style="educational",
            model_path="tts_models/ur/mai/tacotron2-DDC",
            config_path=None
        )
        
        profiles["ur_male_prof"] = VoiceProfile(
            voice_id="ur_male_prof",
            name="Ahmed - Professional",
            language="ur", 
            gender="male",
            style="professional",
            model_path="tts_models/ur/mai/tacotron2-DDC",
            config_path=None
        )
        
        return profiles
    
    async def _load_model(self, voice_profile: VoiceProfile):
        """Load TTS model for specific voice profile"""
        if voice_profile.voice_id in self.loaded_models:
            return self.loaded_models[voice_profile.voice_id]
        
        try:
            logger.info(f"Loading TTS model for {voice_profile.voice_id}...")
            
            if voice_profile.language == "en":
                # Load OpenVoice model
                model = await self._load_openvoice_model(voice_profile)
            elif voice_profile.language == "ur":
                # Load Coqui TTS model
                model = await self._load_coqui_model(voice_profile)
            else:
                raise ValueError(f"Unsupported language: {voice_profile.language}")
            
            self.loaded_models[voice_profile.voice_id] = model
            logger.info(f"Model loaded successfully: {voice_profile.voice_id}")
            return model
            
        except Exception as e:
            logger.error(f"Failed to load model {voice_profile.voice_id}: {e}")
            raise
    
    async def _load_openvoice_model(self, voice_profile: VoiceProfile):
        """Load OpenVoice model for English TTS"""
        try:
            # Import OpenVoice components
            from openvoice import se_extractor
            from openvoice.api import BaseSpeakerTTS, ToneColorConverter
            
            # Initialize base speaker TTS
            base_speaker_tts = BaseSpeakerTTS(
                f"{self.models_dir}/OpenVoice/checkpoints/base_speakers/EN/config.json",
                device=self.device
            )
            base_speaker_tts.load_ckpt(f"{self.models_dir}/OpenVoice/checkpoints/base_speakers/EN/checkpoint.pth")
            
            # Initialize tone color converter
            tone_color_converter = ToneColorConverter(
                f"{self.models_dir}/OpenVoice/checkpoints/converter/config.json",
                device=self.device
            )
            tone_color_converter.load_ckpt(f"{self.models_dir}/OpenVoice/checkpoints/converter/checkpoint.pth")
            
            return {
                "base_tts": base_speaker_tts,
                "converter": tone_color_converter,
                "se_extractor": se_extractor,
                "type": "openvoice"
            }
            
        except Exception as e:
            logger.error(f"OpenVoice model loading failed: {e}")
            # Fallback to basic TTS if OpenVoice fails
            return await self._load_fallback_english_tts()
    
    async def _load_coqui_model(self, voice_profile: VoiceProfile):
        """Load Coqui TTS model for Urdu"""
        try:
            from TTS.api import TTS
            
            # Initialize Coqui TTS
            tts = TTS(
                model_name=voice_profile.model_path,
                progress_bar=False,
                gpu=torch.cuda.is_available()
            )
            
            return {
                "model": tts,
                "type": "coqui"
            }
            
        except Exception as e:
            logger.error(f"Coqui TTS model loading failed: {e}")
            # Fallback to basic Urdu TTS
            return await self._load_fallback_urdu_tts()
    
    async def _load_fallback_english_tts(self):
        """Fallback English TTS using system TTS"""
        try:
            import pyttsx3
            engine = pyttsx3.init()
            
            # Configure voice settings
            voices = engine.getProperty('voices')
            if voices:
                engine.setProperty('voice', voices[0].id)
            
            engine.setProperty('rate', 150)  # Speech rate
            engine.setProperty('volume', 1.0)  # Volume level
            
            return {
                "engine": engine,
                "type": "pyttsx3"
            }
            
        except Exception as e:
            logger.error(f"Fallback English TTS failed: {e}")
            raise RuntimeError("No English TTS models available")
    
    async def _load_fallback_urdu_tts(self):
        """Fallback Urdu TTS using gTTS"""
        try:
            from gtts import gTTS
            
            return {
                "gtts": gTTS,
                "type": "gtts",
                "language": "ur"
            }
            
        except Exception as e:
            logger.error(f"Fallback Urdu TTS failed: {e}")
            raise RuntimeError("No Urdu TTS models available")
    
    def _preprocess_text_for_tts(self, text: str, language: str) -> Tuple[str, List[str]]:
        """Preprocess text for better TTS pronunciation"""
        processed_text = text
        technical_terms = []
        
        # Extract and handle technical terms
        if language == "en":
            # English technical term pronunciations
            replacements = {
                "ROS2": "ross two",
                "URDF": "u r d f",
                "SLAM": "slam",
                "LiDAR": "lie dar",
                "IMU": "i m u",
                "API": "a p i",
                "JSON": "jay son",
                "YAML": "yam ul",
                "C++": "c plus plus",
                "tf2": "t f two",
                "NVIDIA": "en-vidia"
            }
        else:  # Urdu
            replacements = {
                "ROS2": "آر او ایس ٹو",
                "SLAM": "سلیم",
                "LiDAR": "لائیڈار",
                "API": "اے پی آئی",
                "JSON": "جے ایس او این"
            }
        
        for term, pronunciation in replacements.items():
            if term in processed_text:
                technical_terms.append(term)
                processed_text = processed_text.replace(term, pronunciation)
        
        # Clean up markdown and special characters
        processed_text = re.sub(r'[*_`#]', '', processed_text)
        processed_text = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', processed_text)
        processed_text = re.sub(r'```[^`]*```', '[code example]', processed_text)
        
        # Handle URLs and email addresses
        processed_text = re.sub(r'http[s]?://\S+', '[website link]', processed_text)
        processed_text = re.sub(r'\S+@\S+', '[email address]', processed_text)
        
        # Clean up extra whitespace
        processed_text = ' '.join(processed_text.split())
        
        return processed_text, technical_terms
    
    def _get_cache_key(self, text: str, voice_id: str) -> str:
        """Generate cache key for TTS request"""
        content = f"{text}_{voice_id}"
        return hashlib.md5(content.encode('utf-8')).hexdigest()
    
    def _load_from_cache(self, cache_key: str) -> Optional[TTSResult]:
        """Load TTS result from cache"""
        cache_file = self.cache_dir / f"{cache_key}_metadata.json"
        if cache_file.exists():
            try:
                with open(cache_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                # Check if audio file still exists
                audio_path = Path(data['audio_file_path'])
                if audio_path.exists():
                    return TTSResult(**data)
                    
            except Exception as e:
                logger.warning(f"Failed to load cache {cache_key}: {e}")
        return None
    
    def _save_to_cache(self, cache_key: str, result: TTSResult):
        """Save TTS result to cache"""
        cache_file = self.cache_dir / f"{cache_key}_metadata.json"
        try:
            with open(cache_file, 'w', encoding='utf-8') as f:
                json.dump(asdict(result), f, ensure_ascii=False, indent=2)
        except Exception as e:
            logger.warning(f"Failed to save cache {cache_key}: {e}")
    
    async def synthesize_text(self, 
                            text: str, 
                            voice_id: str = "en_female_edu",
                            optimize_for_web: bool = True) -> TTSResult:
        """Synthesize text to speech"""
        start_time = datetime.now()
        
        # Validate voice profile
        if voice_id not in self.voice_profiles:
            raise ValueError(f"Unknown voice ID: {voice_id}")
        
        voice_profile = self.voice_profiles[voice_id]
        
        # Check cache
        cache_key = self._get_cache_key(text, voice_id)
        cached_result = self._load_from_cache(cache_key)
        if cached_result:
            logger.info(f"TTS served from cache: {cache_key}")
            return cached_result
        
        try:
            # Preprocess text
            processed_text, technical_terms = self._preprocess_text_for_tts(
                text, voice_profile.language
            )
            
            # Load appropriate model
            model = await self._load_model(voice_profile)
            
            # Generate audio
            temp_audio_path = self.cache_dir / f"temp_{cache_key}.wav"
            await self._generate_audio(processed_text, model, voice_profile, temp_audio_path)
            
            # Optimize audio for web if requested
            if optimize_for_web:
                final_audio_path = self.cache_dir / f"{cache_key}.mp3"
                audio_info = self.audio_processor.optimize_for_web(temp_audio_path, final_audio_path)
                temp_audio_path.unlink()  # Remove temporary file
            else:
                final_audio_path = self.cache_dir / f"{cache_key}.wav"
                temp_audio_path.rename(final_audio_path)
                audio_info = {
                    "duration": self._get_audio_duration(final_audio_path),
                    "file_size": final_audio_path.stat().st_size,
                    "sample_rate": 22050,
                    "channels": 1
                }
            
            # Calculate quality score
            quality_score = self._assess_audio_quality(final_audio_path, processed_text)
            
            # Calculate generation time
            generation_time = (datetime.now() - start_time).total_seconds()
            
            # Create result
            result = TTSResult(
                text=text,
                language=voice_profile.language,
                voice_id=voice_id,
                audio_file_path=str(final_audio_path),
                duration=audio_info["duration"],
                file_size=audio_info["file_size"],
                generation_time=generation_time,
                sample_rate=audio_info["sample_rate"],
                format="mp3" if optimize_for_web else "wav",
                quality_score=quality_score,
                technical_terms=technical_terms
            )
            
            # Save to cache
            self._save_to_cache(cache_key, result)
            
            logger.info(f"TTS completed in {generation_time:.2f}s, duration: {audio_info['duration']:.1f}s")
            return result
            
        except Exception as e:
            logger.error(f"TTS synthesis failed: {e}")
            raise
    
    async def _generate_audio(self, text: str, model: Dict, voice_profile: VoiceProfile, output_path: Path):
        """Generate audio using loaded model"""
        try:
            if model["type"] == "openvoice":
                await self._generate_openvoice_audio(text, model, output_path)
            elif model["type"] == "coqui":
                await self._generate_coqui_audio(text, model, output_path)
            elif model["type"] == "pyttsx3":
                await self._generate_pyttsx3_audio(text, model, output_path)
            elif model["type"] == "gtts":
                await self._generate_gtts_audio(text, model, output_path)
            else:
                raise ValueError(f"Unknown model type: {model['type']}")
                
        except Exception as e:
            logger.error(f"Audio generation failed with {model['type']}: {e}")
            raise
    
    async def _generate_openvoice_audio(self, text: str, model: Dict, output_path: Path):
        """Generate audio using OpenVoice"""
        try:
            base_tts = model["base_tts"]
            
            # Generate audio
            src_path = str(output_path.with_suffix('.tmp.wav'))
            base_tts.tts(text, src_path, speaker='default', language='English', speed=1.0)
            
            # Convert to target format
            audio = AudioSegment.from_wav(src_path)
            audio.export(str(output_path), format="wav")
            
            # Clean up temporary file
            Path(src_path).unlink(missing_ok=True)
            
        except Exception as e:
            logger.error(f"OpenVoice generation failed: {e}")
            raise
    
    async def _generate_coqui_audio(self, text: str, model: Dict, output_path: Path):
        """Generate audio using Coqui TTS"""
        try:
            tts = model["model"]
            tts.tts_to_file(text=text, file_path=str(output_path))
            
        except Exception as e:
            logger.error(f"Coqui TTS generation failed: {e}")
            raise
    
    async def _generate_pyttsx3_audio(self, text: str, model: Dict, output_path: Path):
        """Generate audio using pyttsx3 fallback"""
        try:
            engine = model["engine"]
            engine.save_to_file(text, str(output_path))
            engine.runAndWait()
            
        except Exception as e:
            logger.error(f"pyttsx3 generation failed: {e}")
            raise
    
    async def _generate_gtts_audio(self, text: str, model: Dict, output_path: Path):
        """Generate audio using gTTS fallback"""
        try:
            from gtts import gTTS
            
            tts = gTTS(text=text, lang=model["language"], slow=False)
            
            # Save to temporary file then convert
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as tmp_file:
                tts.save(tmp_file.name)
                
                # Convert to wav
                audio = AudioSegment.from_mp3(tmp_file.name)
                audio.export(str(output_path), format="wav")
                
                Path(tmp_file.name).unlink()
                
        except Exception as e:
            logger.error(f"gTTS generation failed: {e}")
            raise
    
    def _get_audio_duration(self, audio_path: Path) -> float:
        """Get audio duration in seconds"""
        try:
            if audio_path.suffix.lower() == '.wav':
                with wave.open(str(audio_path), 'rb') as wav_file:
                    frames = wav_file.getnframes()
                    sample_rate = wav_file.getframerate()
                    return frames / sample_rate
            else:
                audio = AudioSegment.from_file(str(audio_path))
                return len(audio) / 1000.0
                
        except Exception as e:
            logger.error(f"Failed to get audio duration: {e}")
            return 0.0
    
    def _assess_audio_quality(self, audio_path: Path, original_text: str) -> float:
        """Assess audio quality score (0-1)"""
        quality_factors = []
        
        # Check file size (reasonable for content length)
        file_size = audio_path.stat().st_size
        duration = self._get_audio_duration(audio_path)
        
        if duration > 0:
            # Expect roughly 1MB per minute for good quality
            expected_size = duration * 1024 * 1024 / 60
            size_ratio = min(file_size / expected_size, expected_size / file_size)
            quality_factors.append(min(1.0, size_ratio))
        
        # Check duration vs text length (rough speech rate check)
        words = len(original_text.split())
        if words > 0 and duration > 0:
            words_per_minute = (words / duration) * 60
            # Expect 120-180 WPM for educational content
            if 80 <= words_per_minute <= 200:
                quality_factors.append(1.0)
            else:
                quality_factors.append(0.7)
        
        return sum(quality_factors) / len(quality_factors) if quality_factors else 0.8
    
    async def synthesize_chapter(self, 
                               chapter_text: str, 
                               voice_id: str = "en_female_edu",
                               max_chunk_duration: int = 60) -> List[TTSResult]:
        """Synthesize entire chapter with automatic chunking"""
        # Split chapter into paragraphs
        paragraphs = [p.strip() for p in chapter_text.split('\n\n') if p.strip()]
        
        results = []
        current_chunk = ""
        
        for paragraph in paragraphs:
            # Skip markdown headers initially
            if paragraph.startswith('#'):
                if current_chunk:
                    result = await self.synthesize_text(current_chunk, voice_id)
                    results.append(result)
                    current_chunk = ""
                continue
            
            # Add paragraph to current chunk
            test_chunk = f"{current_chunk}\n\n{paragraph}" if current_chunk else paragraph
            
            # Estimate duration (rough: 150 words per minute)
            words = len(test_chunk.split())
            estimated_duration = (words / 150) * 60
            
            if estimated_duration <= max_chunk_duration:
                current_chunk = test_chunk
            else:
                # Process current chunk and start new one
                if current_chunk:
                    result = await self.synthesize_text(current_chunk, voice_id)
                    results.append(result)
                current_chunk = paragraph
        
        # Process final chunk
        if current_chunk:
            result = await self.synthesize_text(current_chunk, voice_id)
            results.append(result)
        
        return results
    
    def get_available_voices(self) -> Dict[str, VoiceProfile]:
        """Get all available voice profiles"""
        return self.voice_profiles