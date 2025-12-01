#!/usr/bin/env python3
"""
Qdrant Vector Store Setup
Creates collection for textbook chapter embeddings
"""

import sys
import os

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PayloadSchemaType
from app.config import settings


def setup_qdrant_collection():
    """
    Create Qdrant collection for textbook chapters.

    Collection Spec:
    - Name: textbook_chapters
    - Vector size: 3072 (text-embedding-3-large)
    - Distance: Cosine similarity
    - Indexes: chapter_id, difficulty
    """
    print("🔧 Setting up Qdrant collection...")

    # Connect to Qdrant
    client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY
    )

    collection_name = "textbook_chapters"

    # Check if collection exists
    collections = client.get_collections().collections
    collection_names = [c.name for c in collections]

    if collection_name in collection_names:
        print(f"⚠️  Collection '{collection_name}' already exists")
        response = input("Delete and recreate? (y/N): ")
        if response.lower() == 'y':
            client.delete_collection(collection_name)
            print(f"🗑️  Deleted existing collection")
        else:
            print("Keeping existing collection")
            return

    # Create collection
    client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(
            size=3072,  # OpenAI text-embedding-3-large dimension
            distance=Distance.COSINE
        )
    )

    print(f"✅ Created collection: {collection_name}")

    # Create payload indexes for efficient filtering
    client.create_payload_index(
        collection_name=collection_name,
        field_name="chapter_id",
        field_schema=PayloadSchemaType.KEYWORD
    )

    print("📑 Created index: chapter_id")

    client.create_payload_index(
        collection_name=collection_name,
        field_name="difficulty",
        field_schema=PayloadSchemaType.KEYWORD
    )

    print("📑 Created index: difficulty")

    # Get collection info
    info = client.get_collection(collection_name)
    print(f"\n📊 Collection Info:")
    print(f"   Name: {info.config.params.vectors.size}")
    print(f"   Vector size: {info.config.params.vectors.size}")
    print(f"   Distance: {info.config.params.vectors.distance}")
    print(f"   Points count: {info.points_count}")

    print("\n🎉 Qdrant setup complete!")
    print(f"   Collection '{collection_name}' ready for embeddings")


if __name__ == "__main__":
    setup_qdrant_collection()
