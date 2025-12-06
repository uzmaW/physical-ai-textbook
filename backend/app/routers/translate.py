"""
Translation router for Urdu translation with caching
Uses free Helsinki-NLP translation models from Hugging Face
Indexes translations in Qdrant for RAG search
"""

import os
# IMPORTANT: Set HF_HOME before importing transformers to avoid deprecation warning
if 'HF_HOME' not in os.environ:
    os.environ['HF_HOME'] = '/tmp/huggingface_cache'

from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from sqlalchemy.orm import Session
from typing import Optional
import re
from datetime import datetime

# Try to use transformers for free translation
try:
    from transformers import MarianMTModel, MarianTokenizer
    TRANSFORMERS_AVAILABLE = True
except ImportError:
    TRANSFORMERS_AVAILABLE = False
    print("Warning: transformers not installed. Translation features disabled.")
    print("Install with: pip install transformers torch")

# Try HuggingFace API as fallback
try:
    from huggingface_hub import InferenceClient
    HF_API_AVAILABLE = True
except ImportError:
    HF_API_AVAILABLE = False
    print("Warning: huggingface_hub not installed for API mode.")

from app.db.neon import get_db

try:
    from app.models.user import TranslationCache
    DB_AVAILABLE = True
except ImportError:
    DB_AVAILABLE = False
    print("Warning: Database models not available. Translation caching disabled.")
    TranslationCache = None

try:
    from app.services.rag import RAGService
    RAG_AVAILABLE = True
except ImportError:
    RAG_AVAILABLE = False
    print("Warning: RAG service not available. Translation indexing disabled.")

router = APIRouter()

# Initialize free translation model (Helsinki-NLP)
translator = None
tokenizer = None

if TRANSFORMERS_AVAILABLE:
    try:
        # Load English to Urdu translation model (lazy load on first use)
        model_name = "Helsinki-NLP/opus-mt-en-ur"
        # Skip loading at startup to avoid hanging
        # print(f"Loading translation model: {model_name}")
        # tokenizer = MarianTokenizer.from_pretrained(model_name)
        # translator = MarianMTModel.from_pretrained(model_name)
        print("✅ Translation model will load on first request")
    except Exception as e:
        print(f"Warning: Could not load translation model: {e}")
        print("Translation features will be disabled.")


class TranslateRequest(BaseModel):
    """Request model for translation."""
    userId: str
    chapterId: str
    content: str
    targetLang: str = "ur"


class TranslateResponse(BaseModel):
    """Response model for translation."""
    translatedContent: str
    cached: bool
    chapterId: str
    characterCount: int
    indexed: bool = False  # Whether content was successfully indexed in Qdrant


async def index_translated_content(
    chapter_id: str,
    translated_text: str,
    source_lang: str = "en",
    target_lang: str = "ur",
    chapter_title: str = "Unknown",
    chapter_url: str = "#"
):
    """
    Index translated content in Qdrant for RAG search.
    
    Args:
        chapter_id: Chapter identifier
        translated_text: Translated content
        source_lang: Source language code
        target_lang: Target language code
        chapter_title: Display title of the chapter
        chapter_url: URL to the chapter
    """
    if not RAG_AVAILABLE:
        print("Warning: RAG service not available, skipping vector indexing")
        return False
    
    try:
        rag_service = RAGService()
        
        # Generate embedding for translated text (await the async call)
        try:
            embedding = await rag_service.embed_query(translated_text)
        except Exception as embed_error:
            print(f"⚠️  Could not generate embedding (OpenAI): {embed_error}")
            print("   Translation cached but NOT indexed in Qdrant")
            return False
        
        # Create point ID for the translated content
        point_id = f"{chapter_id}_{target_lang}".replace("-", "_").replace(".", "_")
        
        # Ensure we have a numeric ID (hash the string if needed)
        point_id_numeric = abs(hash(point_id)) % (10 ** 8)
        
        # Store in Qdrant with language metadata
        rag_service.qdrant_client.upsert(
            collection_name=rag_service.collection_name,
            points=[{
                "id": point_id_numeric,
                "vector": embedding,
                "payload": {
                    "chapter_id": chapter_id,
                    "content": translated_text,
                    "language": target_lang,
                    "source_language": source_lang,
                    "chapter_title": chapter_title,
                    "url": chapter_url,
                    "translation_date": datetime.utcnow().isoformat(),
                    "is_translation": True
                }
            }]
        )
        
        print(f"✅ Indexed {target_lang.upper()} translation for chapter {chapter_id} (ID: {point_id_numeric})")
        return True
        
    except Exception as e:
        print(f"⚠️  Warning: Could not index translated content in Qdrant: {e}")
        import traceback
        traceback.print_exc()
        return False


@router.get("/cached")
async def get_cached_translation(
    userId: str,
    chapterId: str,
    db: Session = Depends(get_db)
):
    """
    Check if translation exists in cache.

    Query Params:
        userId: User ID
        chapterId: Chapter ID (e.g., "week-01")

    Returns:
        Cached translation if exists, 404 otherwise
    """
    if not DB_AVAILABLE or not db:
        raise HTTPException(status_code=503, detail="Database not configured")
    
    cache_key = f"{userId}:{chapterId}"

    cached = db.query(TranslationCache).filter(
        TranslationCache.id == cache_key
    ).first()

    if cached:
        # Index cached translation in Qdrant (if not already indexed)
        indexed = await index_translated_content(
            chapter_id=chapterId,
            translated_text=cached.translated_content,
            source_lang=cached.source_lang or "en",
            target_lang=cached.target_lang or "ur",
            chapter_title=chapterId.replace("-", " ").title(),
            chapter_url=f"/docs/{chapterId}"
        )
        
        return {
            "content": cached.translated_content,
            "cached": True,
            "chapterId": chapterId,
            "indexed": indexed,
            "createdAt": cached.created_at.isoformat()
        }
    else:
        raise HTTPException(status_code=404, detail="Translation not cached")


async def translate_with_libretranslate(text: str, source_lang: str = "en", target_lang: str = "ur") -> str:
    """Translate using LibreTranslate API (FREE, no API key needed)"""
    import httpx

    try:
        # Use public LibreTranslate instance (FREE)
        url = "https://libretranslate.com/translate"

        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(url, json={
                "q": text,
                "source": source_lang,
                "target": target_lang,
                "format": "text"
            })

            if response.status_code == 200:
                result = response.json()
                return result.get("translatedText", text)
            else:
                raise Exception(f"LibreTranslate API returned {response.status_code}")

    except Exception as e:
        print(f"LibreTranslate API failed: {e}")
        raise HTTPException(status_code=503, detail=f"Translation failed: {str(e)}")


async def translate_with_hf_api(text: str, model_name: str = "Helsinki-NLP/opus-mt-en-ur") -> str:
    """Translate using HuggingFace Inference API (FREE tier with rate limits)"""
    from app.config import settings

    if not HF_API_AVAILABLE:
        raise HTTPException(status_code=503, detail="HuggingFace API not available")

    try:
        client = InferenceClient(token=settings.HUGGINGFACE_API_KEY if settings.HUGGINGFACE_API_KEY else None)

        # HuggingFace Inference API for translation
        response = client.translation(text, model=model_name)
        return response['translation_text']
    except Exception as e:
        print(f"⚠️  HF API translation failed: {e}")
        # Don't raise, let caller handle fallback
        return None


async def translate_with_local_model(text: str) -> str:
    """Translate using locally loaded model"""
    global translator, tokenizer

    # Lazy load model on first request
    if not translator or not tokenizer:
        if not TRANSFORMERS_AVAILABLE:
            raise HTTPException(
                status_code=503,
                detail="Translation service not available. Install transformers: pip install transformers torch"
            )

        try:
            model_name = "Helsinki-NLP/opus-mt-en-ur"
            print(f"Loading translation model: {model_name}")
            tokenizer = MarianTokenizer.from_pretrained(model_name)
            translator = MarianMTModel.from_pretrained(model_name)

            # Optimize model for CPU inference
            translator.eval()  # Set to evaluation mode
            translator = translator.to('cpu')  # Explicitly use CPU

            print("✅ Free translation model loaded successfully!")
        except Exception as e:
            raise HTTPException(
                status_code=503,
                detail=f"Could not load translation model: {str(e)}"
            )

    # Split content into sentences for better translation
    sentences = re.split(r'([.!?]+)', text)
    translated_sentences = []

    for sentence in sentences:
        if sentence.strip() and not re.match(r'^[.!?]+$', sentence):
            # Tokenize and translate
            inputs = tokenizer(sentence, return_tensors="pt", padding=True, truncation=True, max_length=512)

            # Use torch.no_grad() for faster inference (no gradient computation)
            import torch
            with torch.no_grad():
                outputs = translator.generate(**inputs, max_length=512, num_beams=4)

            translated = tokenizer.decode(outputs[0], skip_special_tokens=True)
            translated_sentences.append(translated)
        else:
            translated_sentences.append(sentence)

    return ''.join(translated_sentences)


@router.post("/", response_model=TranslateResponse)
async def translate_chapter(
    request: TranslateRequest,
    db: Session = Depends(get_db)
):
    """
    Translate chapter content and cache result using free Helsinki-NLP model.

    Supports two modes:
    - API mode (USE_HF_API=true): Uses HuggingFace Inference API (small Docker image)
    - Local mode (USE_HF_API=false): Downloads and runs model locally (large Docker image)

    Body:
        {
            "userId": "user-uuid",
            "chapterId": "week-01",
            "content": "Chapter content in English...",
            "targetLang": "ur"
        }

    Returns:
        Translated content with caching metadata
    """
    from app.config import settings

    try:
        translated_text = None
        translation_method = "unknown"

        # Three-tier fallback system for maximum reliability
        if settings.USE_HF_API:
            # Tier 1: Try HuggingFace API first (fast, good quality, has rate limits)
            print("📡 [1/3] Trying HuggingFace API for translation...")
            try:
                translated_text = await translate_with_hf_api(request.content)
                if translated_text:
                    translation_method = "huggingface-api"
                    print("✅ Translated with HuggingFace API")
            except Exception as e:
                print(f"⚠️  HuggingFace API failed: {e}")

            # Tier 2: Fallback to LibreTranslate if HF API fails
            if translated_text is None:
                print("📡 [2/3] Falling back to LibreTranslate API...")
                try:
                    translated_text = await translate_with_libretranslate(
                        request.content,
                        source_lang="en",
                        target_lang=request.targetLang
                    )
                    translation_method = "libretranslate-api"
                    print("✅ Translated with LibreTranslate API")
                except Exception as e:
                    print(f"⚠️  LibreTranslate API failed: {e}")

            # Tier 3: Final fallback to local model if both APIs fail
            if translated_text is None:
                print("💻 [3/3] Falling back to local model...")
                try:
                    translated_text = await translate_with_local_model(request.content)
                    translation_method = "local-model"
                    print("✅ Translated with local model")
                except Exception as e:
                    print(f"❌ Local model failed: {e}")
                    raise HTTPException(
                        status_code=503,
                        detail="All translation methods failed. Please try again later."
                    )
        else:
            # Direct local model usage (skip APIs)
            print("💻 Using local model for translation")
            translated_text = await translate_with_local_model(request.content)
            translation_method = "local-model"

        if translated_text is None:
            raise HTTPException(
                status_code=503,
                detail="Translation failed. Please try again later."
            )

        char_count = len(request.content)
        print(f"📊 Translation completed using: {translation_method}")

        # Cache in database if available
        if DB_AVAILABLE and db:
            try:
                cache_key = f"{request.userId}:{request.chapterId}"

                cache_entry = TranslationCache(
                    id=cache_key,
                    user_id=request.userId,
                    chapter_id=request.chapterId,
                    source_lang="en",
                    target_lang=request.targetLang,
                    translated_content=translated_text,
                    translation_service=translation_method,  # Track which method was used
                    character_count={"source": char_count, "translated": len(translated_text)}
                )

                # Upsert (merge handles insert or update)
                db.merge(cache_entry)
                db.commit()
            except Exception as cache_error:
                print(f"Warning: Could not cache translation: {cache_error}")
                if db:
                    db.rollback()

        # Index translated content in Qdrant for RAG search
        indexed = await index_translated_content(
            chapter_id=request.chapterId,
            translated_text=translated_text,
            source_lang="en",
            target_lang=request.targetLang,
            chapter_title=request.chapterId.replace("-", " ").title(),
            chapter_url=f"/docs/{request.chapterId}"
        )

        return TranslateResponse(
            translatedContent=translated_text,
            cached=False,
            chapterId=request.chapterId,
            characterCount=char_count,
            indexed=indexed
        )

    except Exception as e:
        if DB_AVAILABLE and db:
            db.rollback()
        raise HTTPException(
            status_code=500,
            detail=f"Translation failed: {str(e)}"
        )


@router.delete("/cache/{userId}/{chapterId}")
async def clear_translation_cache(
    userId: str,
    chapterId: str,
    db: Session = Depends(get_db)
):
    """
    Clear cached translation for a user+chapter.

    Path Params:
        userId: User ID
        chapterId: Chapter ID

    Returns:
        Success message
    """
    if not DB_AVAILABLE or not db:
        raise HTTPException(status_code=503, detail="Database not configured")
    
    cache_key = f"{userId}:{chapterId}"

    deleted = db.query(TranslationCache).filter(
        TranslationCache.id == cache_key
    ).delete()

    db.commit()

    if deleted:
        return {"message": "Cache cleared", "chapterId": chapterId}
    else:
        raise HTTPException(status_code=404, detail="No cached translation found")
