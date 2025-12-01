#!/usr/bin/env python3
"""
Generate embeddings for all textbook chapters
Chunks content and uploads to Qdrant vector store
"""

import sys
import os
import re
from pathlib import Path

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import frontmatter
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
import hashlib
import tiktoken
from app.config import settings


def count_tokens(text: str, model: str = "gpt-4") -> int:
    """Count tokens in text."""
    encoding = tiktoken.encoding_for_model(model)
    return len(encoding.encode(text))


def chunk_markdown(content: str, max_tokens: int = 1000, overlap_tokens: int = 200) -> list[str]:
    """
    Split markdown into semantic chunks.

    Args:
        content: Markdown content
        max_tokens: Maximum tokens per chunk
        overlap_tokens: Overlap between chunks

    Returns:
        List of text chunks
    """
    # Split by paragraphs (double newline)
    paragraphs = content.split('\n\n')

    chunks = []
    current_chunk = ""
    current_tokens = 0

    for para in paragraphs:
        para_tokens = count_tokens(para)

        # If adding this paragraph exceeds max, start new chunk
        if current_tokens + para_tokens > max_tokens and current_chunk:
            chunks.append(current_chunk.strip())

            # Start new chunk with overlap (last paragraph)
            current_chunk = para + "\n\n"
            current_tokens = para_tokens
        else:
            current_chunk += para + "\n\n"
            current_tokens += para_tokens

    # Add final chunk
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


async def embed_chapter(chapter_path: Path, openai_client: OpenAI, qdrant_client: QdrantClient):
    """
    Generate embeddings for a single chapter.

    Args:
        chapter_path: Path to .mdx file
        openai_client: OpenAI client
        qdrant_client: Qdrant client
    """
    print(f"\n📄 Processing: {chapter_path.name}")

    # Parse frontmatter
    post = frontmatter.load(chapter_path)
    metadata = post.metadata
    content = post.content

    # Remove MDX-specific syntax
    content = re.sub(r"import .+ from .+;?\n", "", content)  # Remove imports
    content = re.sub(r"<PersonalizedChapter[^>]*>", "", content)  # Remove wrapper tags
    content = re.sub(r"</PersonalizedChapter>", "", content)

    # Chunk content
    chunks = chunk_markdown(content, max_tokens=1000, overlap_tokens=200)

    print(f"   Chunks: {len(chunks)}")

    # Generate embeddings
    points = []

    for idx, chunk in enumerate(chunks):
        # Generate embedding
        response = openai_client.embeddings.create(
            model="text-embedding-3-large",
            input=chunk
        )
        vector = response.data[0].embedding

        # Create unique ID
        chunk_id = hashlib.md5(f"{metadata['id']}-chunk-{idx}".encode()).hexdigest()

        # Create point
        point = PointStruct(
            id=chunk_id,
            vector=vector,
            payload={
                "content": chunk,
                "chapter_id": metadata['id'],
                "chapter_title": metadata.get('title', ''),
                "sidebar_label": metadata.get('sidebar_label', ''),
                "url": f"/chapters/{metadata['id']}",
                "chunk_index": idx,
                "total_chunks": len(chunks),
                "difficulty": "intermediate",  # Default - can be enhanced
                "tokens": count_tokens(chunk)
            }
        )
        points.append(point)

        print(f"   Chunk {idx+1}/{len(chunks)}: {count_tokens(chunk)} tokens")

    # Upload to Qdrant
    qdrant_client.upsert(
        collection_name="textbook_chapters",
        points=points
    )

    print(f"   ✅ Uploaded {len(chunks)} chunks to Qdrant")


async def embed_all_chapters():
    """
    Embed all textbook chapters.

    Reads all week-*.mdx files from docs directory and generates embeddings.
    """
    print("🚀 Starting embedding generation for all chapters...")

    # Initialize clients
    openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)
    qdrant_client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY
    )

    # Find docs directory (relative to script location)
    script_dir = Path(__file__).parent
    docs_dir = script_dir.parent.parent / "physical-ai-humanoid-textbook" / "docs"

    if not docs_dir.exists():
        print(f"❌ Docs directory not found: {docs_dir}")
        print("   Please run this script from the project root")
        return

    # Process all week-*.mdx files
    chapter_files = sorted(docs_dir.glob("week-*.mdx"))

    print(f"\n📚 Found {len(chapter_files)} chapters to process\n")

    total_chunks = 0

    for chapter_file in chapter_files:
        await embed_chapter(chapter_file, openai_client, qdrant_client)
        # Get chunk count from Qdrant
        collection_info = qdrant_client.get_collection("textbook_chapters")
        total_chunks = collection_info.points_count

    print(f"\n\n🎉 Embedding generation complete!")
    print(f"   Chapters processed: {len(chapter_files)}")
    print(f"   Total chunks: {total_chunks}")
    print(f"   Collection: textbook_chapters")
    print(f"\n✅ Ready for RAG queries!")


if __name__ == "__main__":
    import asyncio
    asyncio.run(embed_all_chapters())
