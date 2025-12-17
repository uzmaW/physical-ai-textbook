#!/usr/bin/env python3
"""Test RAG with Urdu translations"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import asyncio
from app.services.rag import RAGService


async def test_urdu_rag():
    """Test RAG queries with Urdu content"""

    print("🧪 Testing RAG with Urdu Translations\n")

    rag = RAGService()

    # Test queries
    queries = [
        ("How many chapters are in the book?", "en"),
        ("کتاب میں کتنے ابواب ہیں؟", "ur"),  # Same question in Urdu
        ("Tell me about ROS2", "en"),
        ("Tell me about ROS2", "ur"),  # English query but search Urdu content
    ]

    for query, language in queries:
        print(f"\n{'='*60}")
        print(f"Query: {query}")
        print(f"Language: {language}")
        print(f"{'='*60}")

        # Search for context
        results = await rag.search_context(
            query=query,
            language=language,
            top_k=3
        )

        print(f"\nFound {len(results)} results:")
        for i, result in enumerate(results, 1):
            print(f"\n{i}. Chapter: {result['chapter_title']}")
            print(f"   Language: {result['language']}")
            print(f"   Is Translation: {result.get('is_translation', False)}")
            print(f"   Score: {result['score']:.4f}")
            print(f"   Content preview: {result['content'][:100]}...")


if __name__ == "__main__":
    asyncio.run(test_urdu_rag())
