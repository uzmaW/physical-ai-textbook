#!/usr/bin/env python3
"""
Complete Textbook Indexing Script
Sets up Qdrant and indexes all textbook content in one command
"""

import sys
import os
import argparse
import asyncio
from pathlib import Path

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import frontmatter
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance, VectorParams, PayloadSchemaType, PointStruct
)
import hashlib
import tiktoken
from app.config import settings


def count_tokens(text: str, model: str = "gpt-4") -> int:
    """Count tokens in text."""
    try:
        encoding = tiktoken.encoding_for_model(model)
        return len(encoding.encode(text))
    except Exception:
        # Fallback to rough estimate
        return len(text) // 4


def chunk_markdown(content: str, max_tokens: int = 1000, overlap_tokens: int = 200) -> list[str]:
    """
    Split markdown into semantic chunks.

    Args:
        content: Markdown content
        max_tokens: Maximum tokens per chunk
        overlap_tokens: Overlap between chunks for context

    Returns:
        List of text chunks
    """
    import re

    # First, try to split by headers
    sections = re.split(r'\n(#{1,3}\s+.+)\n', content)

    chunks = []
    current_chunk = ""
    current_tokens = 0

    for i, section in enumerate(sections):
        section_tokens = count_tokens(section)

        # If this is a header and current chunk is too big, save it
        if section.startswith('#') and current_tokens > max_tokens // 2:
            if current_chunk.strip():
                chunks.append(current_chunk.strip())
            current_chunk = section + "\n"
            current_tokens = section_tokens
        # If adding would exceed max, save current and start new
        elif current_tokens + section_tokens > max_tokens and current_chunk:
            chunks.append(current_chunk.strip())
            # Add overlap from previous chunk
            overlap = ' '.join(current_chunk.split()[-overlap_tokens:])
            current_chunk = overlap + "\n" + section + "\n"
            current_tokens = count_tokens(current_chunk)
        else:
            current_chunk += section + "\n"
            current_tokens += section_tokens

    # Add final chunk
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks if chunks else [content]


def setup_collection(client: QdrantClient, collection_name: str, force: bool = False):
    """
    Create or verify Qdrant collection.

    Args:
        client: Qdrant client
        collection_name: Collection name
        force: If True, delete and recreate existing collection
    """
    print(f"\n🔧 Setting up collection '{collection_name}'...")

    # Check if exists
    collections = client.get_collections().collections
    collection_names = [c.name for c in collections]

    if collection_name in collection_names:
        if force:
            print(f"🗑️  Deleting existing collection...")
            client.delete_collection(collection_name)
        else:
            print(f"✅ Collection already exists")
            return

    # Create collection
    client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(
            size=3072,  # text-embedding-3-large
            distance=Distance.COSINE
        )
    )

    # Create indexes
    client.create_payload_index(
        collection_name=collection_name,
        field_name="chapter_id",
        field_schema=PayloadSchemaType.KEYWORD
    )

    client.create_payload_index(
        collection_name=collection_name,
        field_name="difficulty",
        field_schema=PayloadSchemaType.KEYWORD
    )

    print(f"✅ Collection '{collection_name}' created with indexes")


def get_chapter_chunks(
    chapter_path: Path,
    verbose: bool = False
) -> tuple[list[str], list[dict]]:
    """
    Process a chapter file and return its chunks and metadata.

    Returns:
        A tuple containing a list of chunks and a list of metadata dicts.
    """
    if verbose:
        print(f"\n📄 Processing: {chapter_path.name}")

    chunks = []
    metadata_list = []

    try:
        # Parse frontmatter
        post = frontmatter.load(chapter_path)
        metadata = post.metadata
        content = post.content

        # Clean MDX syntax
        import re
        content = re.sub(r"import .+ from .+;?\n", "", content)
        content = re.sub(r"<[A-Z][^>]*>", "", content)
        content = re.sub(r"</[A-Z][^>]*>", "", content)

        if not content.strip():
            print(f"⚠️  {chapter_path.name}: No content found")
            return [], []

        # Chunk content
        chunks = chunk_markdown(content, max_tokens=1000, overlap_tokens=200)

        if verbose:
            print(f"   Generated {len(chunks)} chunks")

        for idx, chunk in enumerate(chunks):
            meta = {
                "content": chunk,
                "chapter_id": metadata.get('id', chapter_path.stem),
                "chapter_title": metadata.get('title', metadata.get('sidebar_label', chapter_path.stem)),
                "sidebar_label": metadata.get('sidebar_label', ''),
                "url": f"/chapters/{metadata.get('id', chapter_path.stem)}",
                "chunk_index": idx,
                "total_chunks": len(chunks),
                "difficulty": "intermediate",
                "tokens": count_tokens(chunk),
                "file_path": str(chapter_path)
            }
            metadata_list.append(meta)

        return chunks, metadata_list

    except Exception as e:
        print(f"❌ {chapter_path.name}: Error - {str(e)}")
        if verbose:
            import traceback
            traceback.print_exc()
        return [], []


def generate_embeddings(client: OpenAI, texts: list[str]) -> list[list[float]]:
    """Generate embeddings for a list of texts using OpenAI API."""
    response = client.embeddings.create(
        input=texts,
        model="text-embedding-3-large"
    )
    return [item.embedding for item in response.data]

def index_all_chapters(
    docs_dir: Path,
    force: bool = False,
    verbose: bool = False,
    pattern: str = "week-*.mdx"
):
    """
    Index all textbook chapters.

    Args:
        docs_dir: Path to docs directory
        force: Recreate collection if exists
        verbose: Print detailed progress
        pattern: Glob pattern for chapter files
    """
    print("🚀 Textbook Indexing Pipeline")
    print("=" * 60)

    # Validate docs directory
    if not docs_dir.exists():
        print(f"❌ Error: Docs directory not found: {docs_dir}")
        print("\n💡 Tip: Run from project root or specify --docs-dir")
        sys.exit(1)

    # Initialize clients
    print("\n🔌 Connecting to services...")
    try:
        openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)
        qdrant_client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY
        )
        print("✅ Connected to OpenAI and Qdrant")
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        print("\n💡 Check your .env configuration:")
        print("   - OPENAI_API_KEY")
        print("   - QDRANT_URL")
        print("   - QDRANT_API_KEY")
        sys.exit(1)

    # Setup collection
    collection_name = "textbook_chapters"
    setup_collection(qdrant_client, collection_name, force=force)

    # Find chapter files
    chapter_files = sorted(docs_dir.glob(pattern))

    if not chapter_files:
        print(f"\n❌ No files found matching '{pattern}' in {docs_dir}")
        sys.exit(1)

    print(f"\n📚 Found {len(chapter_files)} chapters to index")
    print("=" * 60)

    # Process chapters
    all_chunks = []
    all_metadata = []
    successful_chapters = 0

    for chapter_file in chapter_files:
        chunks, metadata_list = get_chapter_chunks(chapter_file, verbose=verbose)
        if chunks:
            all_chunks.extend(chunks)
            all_metadata.extend(metadata_list)
            successful_chapters += 1

    print(f"\n🔄 Processing {len(all_chunks)} chunks from {successful_chapters} chapters...")

    # Process in batches
    BATCH_SIZE = 32
    total_points = 0

    for i in range(0, len(all_chunks), BATCH_SIZE):
        batch_chunks = all_chunks[i:i + BATCH_SIZE]
        batch_metadata = all_metadata[i:i + BATCH_SIZE]

        print(f"\n🔄 Processing batch {i // BATCH_SIZE + 1}/{(len(all_chunks) + BATCH_SIZE - 1) // BATCH_SIZE}...")

        try:
            # Generate embeddings for the batch
            print(f"   Generating {len(batch_chunks)} embeddings...")
            embeddings = generate_embeddings(openai_client, batch_chunks)

            # Create points for the batch
            points = []
            for j, embedding in enumerate(embeddings):
                # Create unique ID
                chunk_id = hashlib.md5(
                    f"{batch_metadata[j]['chapter_id']}-chunk-{batch_metadata[j]['chunk_index']}".encode()
                ).hexdigest()

                point = PointStruct(
                    id=chunk_id,
                    vector=embedding,
                    payload=batch_metadata[j]
                )
                points.append(point)

            # Upsert batch to Qdrant
            if points:
                qdrant_client.upsert(
                    collection_name=collection_name,
                    points=points
                )
                total_points += len(points)
                print(f"   ✅ Upserted {len(points)} points.")

        except Exception as e:
            print(f"   ❌ Error processing batch: {e}")
            continue

    # Final report
    print("\n" + "=" * 60)
    print("🎉 Indexing Complete!")
    print(f"   Chapters processed: {successful_chapters}/{len(chapter_files)}")
    print(f"   Total chunks indexed: {total_points}")
    print(f"   Collection: {collection_name}")

    # Verify collection
    info = qdrant_client.get_collection(collection_name)
    print(f"   Qdrant points: {info.points_count}")
    print("\n✅ Ready for RAG queries!")



def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Index Physical AI textbook chapters into Qdrant"
    )
    parser.add_argument(
        "--docs-dir",
        type=Path,
        help="Path to docs directory (default: auto-detect)"
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Force recreate collection (deletes existing)"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Print detailed progress"
    )
    parser.add_argument(
        "--pattern",
        default="week-*.mdx",
        help="Glob pattern for chapter files (default: week-*.mdx)"
    )

    args = parser.parse_args()

    # Determine docs directory
    if args.docs_dir:
        docs_dir = args.docs_dir
    else:
        # Auto-detect from script location
        script_dir = Path(__file__).parent
        docs_dir = script_dir.parent.parent / "physical-ai-humanoid-textbook" / "docs"

    # Run indexing
    index_all_chapters(
        docs_dir=docs_dir,
        force=args.force,
        verbose=args.verbose,
        pattern=args.pattern
    )


if __name__ == "__main__":
    main()
