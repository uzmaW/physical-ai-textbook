#!/usr/bin/env python3
"""Test translation endpoint and indexing"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import asyncio
from app.routers.translate import translate_with_libretranslate, index_translated_content


async def test_translation():
    """Test translation and indexing"""

    print("🧪 Testing Translation System\n")

    # Test content
    test_content = "Hello, this is a test of the translation system. How many chapters are in the book?"

    print(f"Original text: {test_content}\n")

    # Test translation
    print("1️⃣ Testing LibreTranslate API...")
    try:
        translated = await translate_with_libretranslate(
            text=test_content,
            source_lang="en",
            target_lang="ur"
        )
        print(f"   ✅ Translation successful")
        print(f"   Translated: {translated}\n")

        # Test indexing
        print("2️⃣ Testing Qdrant indexing...")
        indexed = await index_translated_content(
            chapter_id="test-chapter",
            translated_text=translated,
            source_lang="en",
            target_lang="ur",
            chapter_title="Test Chapter",
            chapter_url="/test"
        )

        if indexed:
            print("   ✅ Indexing successful\n")
        else:
            print("   ⚠️  Indexing failed\n")

        # Test retrieval
        print("3️⃣ Testing Qdrant retrieval...")
        from app.services.rag import RAGService
        rag = RAGService()

        results = await rag.search_context(
            query="test translation",
            language="ur",
            top_k=3
        )

        if results:
            print(f"   ✅ Found {len(results)} Urdu results:")
            for r in results:
                print(f"      - {r['chapter_id']} (lang={r['language']}, is_translation={r.get('is_translation', False)})")
        else:
            print("   ⚠️  No Urdu results found")

    except Exception as e:
        print(f"   ❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(test_translation())
