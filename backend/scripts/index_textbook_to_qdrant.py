#!/usr/bin/env python3
"""
Index all textbook chapters (week 1-13) into Qdrant vector database
Extracts content from .mdx files, creates embeddings, and stores in Qdrant
"""

import sys
import os
import re
import hashlib
from pathlib import Path
from typing import List, Dict, Tuple

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import frontmatter
import tiktoken
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, Distance, VectorParams, PayloadSchemaType
from app.config import settings

try:
    from openai import OpenAI
    HAS_OPENAI = True
except ImportError:
    HAS_OPENAI = False
    print("⚠️  OpenAI not available. Install with: pip install openai")


# Chapter metadata
CHAPTERS_DIR = "/mnt/workingdir/piaic_projects/humanoid_ai/physical-ai-humanoid-textbook/docs"
CHAPTERS = [
    ("week-01.mdx", "Foundations of Physical AI"),
    ("week-02.mdx", "Realistic Interaction & Emotional AI"),
    ("week-03.mdx", "ROS 2 Fundamentals"),
    ("week-04.mdx", "Services, Actions & Launch Files"),
    ("week-05.mdx", "tf2, Control & Multi-Robot"),
    ("week-06.mdx", "Gazebo & Unity Simulation"),
    ("week-07.mdx", "Advanced Gazebo & Domain Randomization"),
    ("week-08.mdx", "NVIDIA Isaac Sim & Isaac ROS"),
    ("week-09.mdx", "Deep RL & Nav2"),
    ("week-10.mdx", "Advanced RL & Imitation Learning"),
    ("week-11.mdx", "Humanoid Kinematics & Locomotion"),
    ("week-12.mdx", "VLA Models & Applications"),
    ("week-13.mdx", "Capstone Project - The Conversational Humanoid"),
]


def count_tokens(text: str, model: str = "gpt-4") -> int:
    """Count tokens in text."""
    try:
        encoding = tiktoken.encoding_for_model(model)
        return len(encoding.encode(text))
    except Exception:
        # Fallback to rough estimate
        return len(text) // 4


def extract_mdx_content(filepath: str) -> Tuple[str, str, str]:
    """
    Extract frontmatter and content from .mdx file using frontmatter library.
    
    Returns:
        (title, chapter_id, content)
    """
    try:
        post = frontmatter.load(filepath)
        metadata = post.metadata
        content = post.content
        
        # Extract metadata
        title = metadata.get('title', metadata.get('sidebar_label', 'Unknown'))
        chapter_id = metadata.get('id', 'unknown')
        
        # Remove MDX-specific syntax
        content = re.sub(r"import .+ from .+;?\n", "", content)  # Remove imports
        content = re.sub(r"<PersonalizedChapter[^>]*>", "", content)  # Remove wrapper tags
        content = re.sub(r"</PersonalizedChapter>", "", content)
        
        return title, chapter_id, content
    
    except Exception as e:
        print(f"❌ Error extracting {filepath}: {e}")
        return "Unknown", "unknown", ""


def chunk_markdown(content: str, max_tokens: int = 1000, overlap_tokens: int = 200) -> List[str]:
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

            # Start new chunk with overlap
            current_chunk = para + "\n\n"
            current_tokens = para_tokens
        else:
            current_chunk += para + "\n\n"
            current_tokens += para_tokens

    # Add final chunk
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


def generate_embeddings(client: OpenAI, texts: List[str]) -> List[List[float]]:
    """Generate embeddings for a list of texts using OpenAI API."""
    try:
        response = client.embeddings.create(
            input=texts,
            model="text-embedding-3-large"
        )
        return [item.embedding for item in response.data]
    except Exception as e:
        print(f"❌ Embedding error: {e}")
        raise


def create_toc(chapter_files: List[str], openai_client: OpenAI, qdrant_client: QdrantClient, collection_name: str):
    """
    Generate and embed a table of contents chunk.
    
    This ensures factual queries like "how many chapters" work correctly.
    """
    print("\n📑 Generating Table of Contents...")
    
    # Build TOC from chapter metadata
    total_chapters = len(chapter_files)
    
    toc_lines = [
        "# Table of Contents - Complete Textbook Structure",
        "",
        f"TOTAL NUMBER OF CHAPTERS: {total_chapters}",
        "",
        f"This textbook contains exactly {total_chapters} chapters covering Physical AI and Humanoid Robotics:",
        ""
    ]
    
    for idx, (filename, display_title) in enumerate(chapter_files, 1):
        filepath = os.path.join(CHAPTERS_DIR, filename)
        try:
            post = frontmatter.load(filepath)
            title = post.metadata.get('title', post.metadata.get('sidebar_label', display_title))
            chapter_id = post.metadata.get('id', filename.replace('.mdx', ''))
            toc_lines.append(f"Chapter {idx}: {title}")
        except Exception as e:
            print(f"   ⚠️  Error reading {filename}: {e}")
            toc_lines.append(f"Chapter {idx}: {display_title}")
    
    toc_content = "\n".join(toc_lines)
    
    # Generate embedding for TOC
    response = openai_client.embeddings.create(
        model="text-embedding-3-large",
        input=toc_content
    )
    vector = response.data[0].embedding
    
    # Create unique ID for TOC (must be integer)
    toc_id = int(hashlib.md5("TABLE_OF_CONTENTS".encode()).hexdigest(), 16) % (2**63)
    
    # Create TOC point
    toc_point = PointStruct(
        id=toc_id,
        vector=vector,
        payload={
            "content": toc_content,
            "chapter_id": "TABLE_OF_CONTENTS",
            "chapter_title": "Table of Contents",
            "sidebar_label": "Table of Contents",
            "url": "/toc",
            "chunk_index": 0,
            "total_chunks": 1,
            "is_toc": True,
            "total_chapters": len(chapter_files),
            "chapter_count": len(chapter_files),
            "difficulty": "beginner",
            "tokens": count_tokens(toc_content)
        }
    )
    
    # Upload TOC to Qdrant
    qdrant_client.upsert(
        collection_name=collection_name,
        points=[toc_point]
    )
    
    print(f"   ✅ TOC uploaded with {len(chapter_files)} chapters listed")


def index_all_chapters():
    """Index all textbook chapters into Qdrant."""
    
    if not HAS_OPENAI:
        print("❌ OpenAI library required. Install: pip install openai")
        return
    
    print("="*70)
    print("📚 Indexing All Textbook Chapters to Qdrant")
    print("="*70)
    
    # Initialize clients
    print("\n🔗 Connecting to services...")
    openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)
    
    qdrant_client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY
    )
    
    collection_name = "textbook_chapters"
    vector_size = 3072  # text-embedding-3-large
    BATCH_SIZE = 32
    
    # Create or recreate collection
    print(f"📦 Setting up collection '{collection_name}'...")
    try:
        # Delete existing collection
        qdrant_client.delete_collection(collection_name)
        print("   ✓ Removed existing collection")
    except:
        pass
    
    # Create new collection
    qdrant_client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE)
    )
    print("   ✓ Created new collection")

    # Create payload indexes for filtering
    print("   📑 Creating payload indexes...")
    try:
        qdrant_client.create_payload_index(
            collection_name=collection_name,
            field_name="difficulty",
            field_schema=PayloadSchemaType.KEYWORD
        )
        print("      ✓ Created index: difficulty")
    except Exception as e:
        print(f"      ⚠ difficulty index: {e}")

    try:
        qdrant_client.create_payload_index(
            collection_name=collection_name,
            field_name="chapter_id",
            field_schema=PayloadSchemaType.KEYWORD
        )
        print("      ✓ Created index: chapter_id")
    except Exception as e:
        print(f"      ⚠ chapter_id index: {e}")

    # Create TOC first
    create_toc(CHAPTERS, openai_client, qdrant_client, collection_name)

    # Index each chapter
    all_chunks = []
    all_metadata = []
    
    for filename, display_title in CHAPTERS:
        filepath = os.path.join(CHAPTERS_DIR, filename)
        
        if not os.path.exists(filepath):
            print(f"⚠️  Skipping {filename} - not found")
            continue
        
        print(f"\n📄 Processing: {filename}")
        
        # Extract content
        title, chapter_id, content = extract_mdx_content(filepath)
        print(f"   Title: {title}")
        print(f"   ID: {chapter_id}")
        
        # Chunk content
        chunks = chunk_markdown(content, max_tokens=1000, overlap_tokens=200)
        print(f"   Chunks: {len(chunks)}")
        
        for chunk_idx, chunk in enumerate(chunks):
            all_chunks.append(chunk)
            all_metadata.append({
                "chapter_id": chapter_id,
                "chapter_title": title,
                "chunk_index": chunk_idx,
                "total_chunks": len(chunks),
                "content": chunk,
                "url": f"/docs/{chapter_id}",
                "difficulty": "intermediate",
                "tokens": count_tokens(chunk)
            })

    # Process chunks in batches
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
                chunk_id = int(hashlib.md5(
                    f"{batch_metadata[j]['chapter_id']}-chunk-{batch_metadata[j]['chunk_index']}".encode()
                ).hexdigest(), 16) % (2**63)
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

    # Print summary
    print(f"\n{'='*70}")
    print(f"✅ Indexing Complete!")
    print(f"{'='*70}")
    print(f"Chapters indexed: {len(CHAPTERS)}")
    print(f"Total chunks: {total_points + 1}")  # +1 for TOC
    print(f"Vector size: {vector_size} dimensions")
    print(f"Collection: {collection_name}")
    print(f"\n📊 Collection Info:")
    
    try:
        info = qdrant_client.get_collection(collection_name)
        print(f"   Points: {info.points_count}")
        print(f"   Vectors indexed: {info.points_count}")
    except Exception as e:
        print(f"   Could not retrieve stats: {e}")
    
    print(f"\n✨ All chapters now searchable in Qdrant!")
    print(f"   The RAG chat API will return detailed responses from the indexed textbook.")


if __name__ == "__main__":
    try:
        index_all_chapters()
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
