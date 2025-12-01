"""
Translation router for Urdu translation with caching
Uses free Helsinki-NLP translation models from Hugging Face
Indexes translations in Qdrant for RAG search
"""

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

try:
    from app.db.neon import get_db
    from app.models.user import TranslationCache
    DB_AVAILABLE = True
except ImportError:
    DB_AVAILABLE = False
    print("Warning: Database models not available. Translation caching disabled.")
    def get_db():
        raise HTTPException(status_code=503, detail="Database not configured")

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
        # Load English to Urdu translation model
        model_name = "Helsinki-NLP/opus-mt-en-ur"
        print(f"Loading translation model: {model_name}")
        tokenizer = MarianTokenizer.from_pretrained(model_name)
        translator = MarianMTModel.from_pretrained(model_name)
        print("✅ Free translation model loaded successfully!")
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


@router.post("/", response_model=TranslateResponse)
async def translate_chapter(
    request: TranslateRequest,
    db: Session = Depends(get_db) if DB_AVAILABLE else None
):
    """
    Translate chapter content and cache result using free Helsinki-NLP model.

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
    if not translator or not tokenizer:
        raise HTTPException(
            status_code=503,
            detail="Translation service not available. Install transformers: pip install transformers torch"
        )

    try:
        # Split content into sentences for better translation
        sentences = re.split(r'([.!?]+)', request.content)
        translated_sentences = []

        for sentence in sentences:
            if sentence.strip() and not re.match(r'^[.!?]+$', sentence):
                # Tokenize and translate
                inputs = tokenizer(sentence, return_tensors="pt", padding=True, truncation=True, max_length=512)
                outputs = translator.generate(**inputs)
                translated = tokenizer.decode(outputs[0], skip_special_tokens=True)
                translated_sentences.append(translated)
            else:
                translated_sentences.append(sentence)

        translated_text = ''.join(translated_sentences)
        char_count = len(request.content)

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
                    translation_service="helsinki-nlp",
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
    cache_key = f"{userId}:{chapterId}"

    deleted = db.query(TranslationCache).filter(
        TranslationCache.id == cache_key
    ).delete()

    db.commit()

    if deleted:
        return {"message": "Cache cleared", "chapterId": chapterId}
    else:
        raise HTTPException(status_code=404, detail="No cached translation found")
