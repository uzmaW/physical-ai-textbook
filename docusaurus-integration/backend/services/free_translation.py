"""
Free Translation Service using Helsinki-NLP Models
English to Urdu translation with technical term preservation
Uses only free/open-source models for cost-effective deployment
"""

import logging
import re
import json
import asyncio
from typing import Dict, List, Optional, Tuple
from pathlib import Path
from dataclasses import dataclass, asdict
from datetime import datetime
import hashlib

import torch
from transformers import MarianMTModel, MarianTokenizer
import nltk
from nltk.tokenize import sent_tokenize, word_tokenize

logger = logging.getLogger(__name__)

@dataclass
class TranslationResult:
    """Result of translation operation"""
    original_text: str
    translated_text: str
    language_pair: str
    model_used: str
    translation_time: float
    quality_score: float
    technical_terms_preserved: List[str]
    warnings: List[str]

@dataclass
class TechnicalTerm:
    """Technical terminology with preservation rules"""
    term: str
    category: str
    preserve_original: bool
    urdu_equivalent: Optional[str] = None
    pronunciation_guide: Optional[str] = None

class RoboticsGlossary:
    """Robotics technical terms dictionary for preservation"""
    
    def __init__(self):
        self.terms = self._load_robotics_terms()
        self.patterns = self._compile_patterns()
    
    def _load_robotics_terms(self) -> Dict[str, TechnicalTerm]:
        """Load robotics technical terms with translation rules"""
        terms = {
            # Core Robotics Terms
            "ROS2": TechnicalTerm("ROS2", "framework", True, "آر او ایس 2"),
            "SLAM": TechnicalTerm("SLAM", "algorithm", True, "سلام"),
            "LiDAR": TechnicalTerm("LiDAR", "sensor", True, "لائیڈار"),
            "IMU": TechnicalTerm("IMU", "sensor", True, "آئی ایم یو"),
            "URDF": TechnicalTerm("URDF", "format", True, "یو آر ڈی ایف"),
            "Gazebo": TechnicalTerm("Gazebo", "simulation", True, "گزیبو"),
            "NVIDIA Isaac Sim": TechnicalTerm("NVIDIA Isaac Sim", "simulation", True, "انویڈیا آئزک سم"),
            
            # Programming Terms
            "Python": TechnicalTerm("Python", "language", True, "پائیتھن"),
            "C++": TechnicalTerm("C++", "language", True, "سی پلس پلس"),
            "JavaScript": TechnicalTerm("JavaScript", "language", True, "جاوا اسکرپٹ"),
            "API": TechnicalTerm("API", "interface", True, "اے پی آئی"),
            "JSON": TechnicalTerm("JSON", "format", True, "جے ایس او این"),
            "YAML": TechnicalTerm("YAML", "format", True, "یامل"),
            
            # Robotics Concepts
            "kinematics": TechnicalTerm("kinematics", "concept", False, "حرکیات"),
            "dynamics": TechnicalTerm("dynamics", "concept", False, "حرکی علم"),
            "trajectory": TechnicalTerm("trajectory", "concept", False, "مسار"),
            "localization": TechnicalTerm("localization", "concept", False, "مقام کا تعین"),
            "navigation": TechnicalTerm("navigation", "concept", False, "رہنمائی"),
            "perception": TechnicalTerm("perception", "concept", False, "ادراک"),
            "manipulation": TechnicalTerm("manipulation", "concept", False, "ہاتھ کا کام"),
            
            # Hardware Terms
            "servo motor": TechnicalTerm("servo motor", "hardware", False, "سروو موٹر"),
            "encoder": TechnicalTerm("encoder", "hardware", False, "انکوڈر"),
            "actuator": TechnicalTerm("actuator", "hardware", False, "محرک"),
            "end-effector": TechnicalTerm("end-effector", "hardware", False, "اختتامی اثر کار"),
            
            # AI/ML Terms
            "machine learning": TechnicalTerm("machine learning", "ai", False, "مشین لرننگ"),
            "neural network": TechnicalTerm("neural network", "ai", False, "عصبی نیٹ ورک"),
            "deep learning": TechnicalTerm("deep learning", "ai", False, "گہری یادگیری"),
            "CNN": TechnicalTerm("CNN", "ai", True, "سی این این"),
            "RNN": TechnicalTerm("RNN", "ai", True, "آر این این"),
        }
        return terms
    
    def _compile_patterns(self) -> Dict[str, re.Pattern]:
        """Compile regex patterns for term detection"""
        patterns = {}
        for term, info in self.terms.items():
            # Create case-insensitive pattern with word boundaries
            pattern = re.compile(rf'\b{re.escape(term)}\b', re.IGNORECASE)
            patterns[term] = pattern
        return patterns
    
    def extract_technical_terms(self, text: str) -> List[Tuple[str, TechnicalTerm]]:
        """Extract technical terms from text"""
        found_terms = []
        for term, pattern in self.patterns.items():
            matches = pattern.finditer(text)
            for match in matches:
                found_terms.append((match.group(), self.terms[term]))
        return found_terms

class FreeTranslationService:
    """Translation service using free Helsinki-NLP models"""
    
    def __init__(self, cache_dir: str = "./translation_cache"):
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(exist_ok=True)
        
        self.model_name = "Helsinki-NLP/opus-mt-en-ur"
        self.model = None
        self.tokenizer = None
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        self.glossary = RoboticsGlossary()
        self.translation_cache = {}
        
        # Download NLTK data if needed
        self._ensure_nltk_data()
        
        logger.info(f"Translation service initialized with device: {self.device}")
    
    def _ensure_nltk_data(self):
        """Ensure required NLTK data is available"""
        try:
            nltk.data.find('tokenizers/punkt')
        except LookupError:
            nltk.download('punkt')
    
    async def initialize_model(self):
        """Load translation model (async to avoid blocking)"""
        if self.model is not None:
            return
        
        try:
            logger.info("Loading Helsinki-NLP translation model...")
            
            # Load model and tokenizer
            self.tokenizer = MarianTokenizer.from_pretrained(
                self.model_name,
                cache_dir=str(self.cache_dir / "models")
            )
            self.model = MarianMTModel.from_pretrained(
                self.model_name,
                cache_dir=str(self.cache_dir / "models")
            )
            
            # Move to appropriate device
            self.model.to(self.device)
            self.model.eval()
            
            logger.info("Translation model loaded successfully")
            
        except Exception as e:
            logger.error(f"Failed to load translation model: {e}")
            raise
    
    def _get_cache_key(self, text: str) -> str:
        """Generate cache key for text"""
        return hashlib.md5(text.encode('utf-8')).hexdigest()
    
    def _load_cache(self, cache_key: str) -> Optional[TranslationResult]:
        """Load translation from cache"""
        cache_file = self.cache_dir / f"{cache_key}.json"
        if cache_file.exists():
            try:
                with open(cache_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                return TranslationResult(**data)
            except Exception as e:
                logger.warning(f"Failed to load cache {cache_key}: {e}")
        return None
    
    def _save_cache(self, cache_key: str, result: TranslationResult):
        """Save translation to cache"""
        cache_file = self.cache_dir / f"{cache_key}.json"
        try:
            with open(cache_file, 'w', encoding='utf-8') as f:
                json.dump(asdict(result), f, ensure_ascii=False, indent=2)
        except Exception as e:
            logger.warning(f"Failed to save cache {cache_key}: {e}")
    
    async def translate_text(self, text: str, preserve_formatting: bool = True) -> TranslationResult:
        """Translate English text to Urdu with technical term preservation"""
        start_time = datetime.now()
        
        # Check cache first
        cache_key = self._get_cache_key(text)
        cached_result = self._load_cache(cache_key)
        if cached_result:
            logger.info(f"Translation served from cache: {cache_key}")
            return cached_result
        
        # Ensure model is loaded
        await self.initialize_model()
        
        try:
            # Extract technical terms before translation
            technical_terms = self.glossary.extract_technical_terms(text)
            
            # Prepare text for translation
            processed_text = self._preprocess_text(text, technical_terms)
            
            # Translate using Helsinki-NLP model
            translated_text = await self._translate_with_model(processed_text)
            
            # Post-process translation
            final_translation = self._postprocess_translation(
                translated_text, technical_terms, preserve_formatting
            )
            
            # Calculate quality score
            quality_score = self._assess_translation_quality(text, final_translation, technical_terms)
            
            # Calculate processing time
            translation_time = (datetime.now() - start_time).total_seconds()
            
            # Create result
            result = TranslationResult(
                original_text=text,
                translated_text=final_translation,
                language_pair="en-ur",
                model_used=self.model_name,
                translation_time=translation_time,
                quality_score=quality_score,
                technical_terms_preserved=[term[0] for term in technical_terms],
                warnings=self._generate_warnings(text, final_translation, technical_terms)
            )
            
            # Save to cache
            self._save_cache(cache_key, result)
            
            logger.info(f"Translation completed in {translation_time:.2f}s, quality: {quality_score:.2f}")
            return result
            
        except Exception as e:
            logger.error(f"Translation failed: {e}")
            raise
    
    def _preprocess_text(self, text: str, technical_terms: List[Tuple[str, TechnicalTerm]]) -> str:
        """Preprocess text before translation"""
        processed_text = text
        
        # Replace technical terms with placeholders
        term_placeholders = {}
        for i, (term_text, term_info) in enumerate(technical_terms):
            if term_info.preserve_original:
                placeholder = f"__TECH_TERM_{i}__"
                processed_text = processed_text.replace(term_text, placeholder)
                term_placeholders[placeholder] = term_text
        
        return processed_text
    
    async def _translate_with_model(self, text: str) -> str:
        """Perform actual translation using the model"""
        try:
            # Tokenize input
            inputs = self.tokenizer(text, return_tensors="pt", padding=True, truncation=True)
            inputs = {k: v.to(self.device) for k, v in inputs.items()}
            
            # Generate translation
            with torch.no_grad():
                translated = self.model.generate(**inputs, max_length=512, num_beams=4, length_penalty=0.6)
            
            # Decode translation
            translated_text = self.tokenizer.decode(translated[0], skip_special_tokens=True)
            
            return translated_text
            
        except Exception as e:
            logger.error(f"Model translation failed: {e}")
            raise
    
    def _postprocess_translation(self, 
                                translated_text: str, 
                                technical_terms: List[Tuple[str, TechnicalTerm]],
                                preserve_formatting: bool) -> str:
        """Post-process translation with term restoration"""
        processed_text = translated_text
        
        # Restore technical terms
        for i, (term_text, term_info) in enumerate(technical_terms):
            placeholder = f"__TECH_TERM_{i}__"
            if placeholder in processed_text:
                if term_info.preserve_original:
                    # Use original English term
                    replacement = term_text
                    if term_info.urdu_equivalent:
                        replacement = f"{term_text} ({term_info.urdu_equivalent})"
                elif term_info.urdu_equivalent:
                    # Use Urdu equivalent
                    replacement = term_info.urdu_equivalent
                else:
                    # Keep original as fallback
                    replacement = term_text
                
                processed_text = processed_text.replace(placeholder, replacement)
        
        # Handle any remaining technical terms that weren't replaced
        for term_text, term_info in technical_terms:
            if term_info.preserve_original and term_text in processed_text:
                if term_info.urdu_equivalent and term_info.urdu_equivalent not in processed_text:
                    processed_text = processed_text.replace(
                        term_text, 
                        f"{term_text} ({term_info.urdu_equivalent})"
                    )
        
        return processed_text
    
    def _assess_translation_quality(self, 
                                  original: str, 
                                  translation: str, 
                                  technical_terms: List[Tuple[str, TechnicalTerm]]) -> float:
        """Assess translation quality score (0-1)"""
        quality_factors = []
        
        # Check technical term preservation
        preserved_terms = 0
        total_terms = len(technical_terms)
        
        if total_terms > 0:
            for term_text, term_info in technical_terms:
                if term_info.preserve_original and term_text in translation:
                    preserved_terms += 1
                elif term_info.urdu_equivalent and term_info.urdu_equivalent in translation:
                    preserved_terms += 1
            
            term_preservation_score = preserved_terms / total_terms
            quality_factors.append(term_preservation_score)
        
        # Check length ratio (translated text should be reasonable length)
        length_ratio = len(translation) / len(original) if original else 0
        length_score = 1.0 if 0.5 <= length_ratio <= 2.0 else 0.5
        quality_factors.append(length_score)
        
        # Check for obvious translation artifacts
        artifact_score = 1.0
        artifacts = ["__TECH_TERM_", "```", "{{", "}}", "<>"]
        for artifact in artifacts:
            if artifact in translation:
                artifact_score -= 0.2
        
        quality_factors.append(max(0, artifact_score))
        
        # Average all quality factors
        return sum(quality_factors) / len(quality_factors) if quality_factors else 0.5
    
    def _generate_warnings(self, 
                          original: str, 
                          translation: str, 
                          technical_terms: List[Tuple[str, TechnicalTerm]]) -> List[str]:
        """Generate warnings about potential translation issues"""
        warnings = []
        
        # Check for missing technical terms
        missing_terms = []
        for term_text, term_info in technical_terms:
            if term_info.preserve_original and term_text not in translation:
                if not term_info.urdu_equivalent or term_info.urdu_equivalent not in translation:
                    missing_terms.append(term_text)
        
        if missing_terms:
            warnings.append(f"Technical terms may be missing: {', '.join(missing_terms)}")
        
        # Check translation length
        length_ratio = len(translation) / len(original) if original else 0
        if length_ratio < 0.3:
            warnings.append("Translation appears too short - possible content loss")
        elif length_ratio > 3.0:
            warnings.append("Translation appears too long - possible duplication")
        
        # Check for untranslated placeholders
        if "__TECH_TERM_" in translation:
            warnings.append("Some technical term placeholders were not properly replaced")
        
        return warnings
    
    async def translate_chapter(self, chapter_content: str) -> TranslationResult:
        """Translate a full chapter with paragraph-level processing"""
        # Split chapter into paragraphs for better translation
        paragraphs = [p.strip() for p in chapter_content.split('\n\n') if p.strip()]
        
        translated_paragraphs = []
        all_technical_terms = []
        total_warnings = []
        total_time = 0
        
        for paragraph in paragraphs:
            if paragraph.startswith('#'):  # Skip headers initially
                translated_paragraphs.append(paragraph)
                continue
            
            result = await self.translate_text(paragraph)
            translated_paragraphs.append(result.translated_text)
            all_technical_terms.extend(result.technical_terms_preserved)
            total_warnings.extend(result.warnings)
            total_time += result.translation_time
        
        final_translation = '\n\n'.join(translated_paragraphs)
        
        # Calculate overall quality
        overall_quality = self._assess_translation_quality(
            chapter_content, final_translation, []
        )
        
        return TranslationResult(
            original_text=chapter_content,
            translated_text=final_translation,
            language_pair="en-ur",
            model_used=self.model_name,
            translation_time=total_time,
            quality_score=overall_quality,
            technical_terms_preserved=list(set(all_technical_terms)),
            warnings=list(set(total_warnings))
        )