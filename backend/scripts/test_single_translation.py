#!/usr/bin/env python3
"""Test translation on a single chapter"""

import sys
import os
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import asyncio
import httpx


async def test_translation():
    """Test HuggingFace Translation API using InferenceClient"""

    print("🧪 Testing HuggingFace Translation API\n")

    test_text = "Hello, this is a test. How many chapters are in the book?"

    print(f"Original: {test_text}\n")

    try:
        from app.config import settings
        from huggingface_hub import InferenceClient

        client = InferenceClient(token=settings.HUGGINGFACE_API_KEY if settings.HUGGINGFACE_API_KEY else None)

        if settings.HUGGINGFACE_API_KEY:
            print("Using HuggingFace API key")
        else:
            print("Using free tier (no API key)")

        model = "Helsinki-NLP/opus-mt-en-ur"

        # Use the InferenceClient for translation
        result = client.translation(test_text, model=model)

        if result and 'translation_text' in result:
            translated = result['translation_text']
            print(f"✅ Translation successful!")
            print(f"Translated: {translated}\n")
            return True
        else:
            print(f"❌ Unexpected response format: {result}")
            return False

    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    result = asyncio.run(test_translation())
    if result:
        print("✅ Translation API is working! Ready to translate chapters.")
    else:
        print("❌ Translation API test failed. Check your internet connection.")
        sys.exit(1)
