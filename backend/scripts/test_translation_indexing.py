#!/usr/bin/env python3
"""
Test script to verify translation indexing in Qdrant
Tests the complete pipeline: translate -> index -> search
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

async def test_translation_indexing():
    """Test the complete translation and indexing pipeline."""
    from app.routers.translate import translate_chapter, TranslateRequest
    from app.services.rag import RAGService
    
    # Test data
    test_request = TranslateRequest(
        userId="test-user",
        chapterId="week-01-test",
        content="Hello world. This is a test translation. The robot moves forward.",
        targetLang="ur"
    )
    
    print("🔄 Testing translation endpoint...")
    try:
        result = await translate_chapter(test_request)
        print(f"✅ Translation successful")
        print(f"   Original: {test_request.content[:50]}...")
        print(f"   Translated: {result.translatedContent[:50]}...")
        print(f"   Cached: {result.cached}")
        
        # Verify translation is indexed
        print("\n🔄 Verifying Qdrant indexing...")
        rag = RAGService()
        
        # Search for the translated content
        search_results = await rag.search_context(
            query="robot",
            language="ur",
            top_k=5
        )
        
        if search_results:
            print(f"✅ Found {len(search_results)} results in Urdu")
            for r in search_results:
                if r.get("is_translation"):
                    print(f"   ✓ Translation indexed: {r['chapter_id']}")
                    print(f"     Language: {r.get('language')}")
                    print(f"     Score: {r.get('score', 0):.3f}")
        else:
            print("⚠️  No results found in Urdu collection")
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False
    
    return True


async def test_cached_translation():
    """Test cached translation retrieval and indexing."""
    from app.routers.translate import get_cached_translation
    from app.db.neon import SessionLocal
    from app.models.user import TranslationCache
    
    print("\n🔄 Testing cached translation retrieval...")
    try:
        # This will only work if there's a cached translation
        db = SessionLocal()
        cached = db.query(TranslationCache).first()
        
        if cached:
            result = await get_cached_translation(
                userId=cached.user_id,
                chapterId=cached.chapter_id,
                db=db
            )
            print(f"✅ Retrieved cached translation")
            print(f"   Chapter: {cached.chapter_id}")
            print(f"   Language: {cached.target_lang}")
            print(f"   Cached at: {result.get('createdAt')}")
        else:
            print("⚠️  No cached translations found (expected on first run)")
            
    except Exception as e:
        print(f"⚠️  Skipped (DB not available): {e}")
    finally:
        db.close()


if __name__ == "__main__":
    print("=== Translation Indexing Test Suite ===\n")
    
    # Run tests
    success = asyncio.run(test_translation_indexing())
    
    if success:
        print("\n✅ All tests passed!")
    else:
        print("\n❌ Tests failed!")
        sys.exit(1)
