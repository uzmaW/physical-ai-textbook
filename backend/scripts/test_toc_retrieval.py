#!/usr/bin/env python3
"""Test script to verify TOC indexing and retrieval"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from qdrant_client import QdrantClient
from app.config import settings
from openai import OpenAI
import asyncio


async def test_toc():
    """Test TOC retrieval from Qdrant"""

    print("🔍 Testing TOC Retrieval...\n")

    # Initialize clients
    qdrant_client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY
    )
    openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)

    # 1. Check if TOC exists
    print("1️⃣ Searching for TOC directly...")
    try:
        result = qdrant_client.scroll(
            collection_name="textbook_chapters",
            scroll_filter={
                "must": [
                    {
                        "key": "is_toc",
                        "match": {"value": True}
                    }
                ]
            },
            limit=1,
            with_payload=True,
            with_vectors=False
        )

        if result[0]:
            toc_point = result[0][0]
            print(f"   ✅ TOC found!")
            print(f"   ID: {toc_point.id}")
            print(f"   Total chapters in TOC: {toc_point.payload.get('total_chapters', 'N/A')}")
            print(f"   Content preview:\n   {toc_point.payload.get('content', '')[:300]}...\n")
        else:
            print("   ❌ TOC not found in database!\n")
            return
    except Exception as e:
        print(f"   ❌ Error searching for TOC: {e}\n")
        return

    # 2. Test semantic search for "how many chapters"
    print("2️⃣ Testing semantic search for 'how many chapters'...")
    query = "how many chapters are there in the book"

    try:
        # Generate embedding
        response = openai_client.embeddings.create(
            model="text-embedding-3-large",
            input=query
        )
        query_vector = response.data[0].embedding

        # Search without filters (to see if TOC appears)
        search_results = qdrant_client.query_points(
            collection_name="textbook_chapters",
            query=query_vector,
            limit=5,
            with_payload=True,
            with_vectors=False
        ).points

        print(f"   Found {len(search_results)} results:\n")
        for idx, result in enumerate(search_results, 1):
            is_toc = result.payload.get("is_toc", False)
            chapter_title = result.payload.get("chapter_title", "Unknown")
            score = result.score
            total_chapters = result.payload.get("total_chapters", "N/A")

            print(f"   {idx}. {'[TOC]' if is_toc else ''} {chapter_title}")
            print(f"      Score: {score:.4f}")
            if is_toc:
                print(f"      Total chapters: {total_chapters}")
            print()

    except Exception as e:
        print(f"   ❌ Error in semantic search: {e}\n")

    # 3. Test with difficulty filter (intermediate)
    print("3️⃣ Testing with 'intermediate' difficulty filter...")
    try:
        search_results_filtered = qdrant_client.query_points(
            collection_name="textbook_chapters",
            query=query_vector,
            limit=5,
            query_filter={
                "must": [
                    {
                        "key": "difficulty",
                        "match": {"value": "intermediate"}
                    }
                ]
            },
            with_payload=True,
            with_vectors=False
        ).points

        print(f"   Found {len(search_results_filtered)} results with intermediate filter:")
        toc_found = False
        for result in search_results_filtered:
            if result.payload.get("is_toc"):
                toc_found = True
                break

        if toc_found:
            print("   ✅ TOC included in filtered results")
        else:
            print("   ⚠️  TOC NOT included (filtered out by difficulty=intermediate)")

    except Exception as e:
        print(f"   ❌ Error in filtered search: {e}\n")


if __name__ == "__main__":
    asyncio.run(test_toc())
