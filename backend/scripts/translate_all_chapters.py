#!/usr/bin/env python3
"""
Pre-translate and index all textbook chapters to Urdu
Reads all week-*.mdx files, translates them, and indexes in Qdrant
"""

import sys
import os
from pathlib import Path
import asyncio
import re

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import frontmatter
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
import hashlib
import tiktoken
from app.config import settings

# Translation imports
try:
    import httpx
    HTTPX_AVAILABLE = True
except ImportError:
    HTTPX_AVAILABLE = False
    print("❌ httpx not installed. Install with: pip install httpx")
    sys.exit(1)


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


def translate_text_sync(text: str, source_lang: str = "en", target_lang: str = "ur", max_retries: int = 3) -> str:
    """
    Translate text using HuggingFace Inference API (synchronous).
    Splits long text into sentences to avoid token limit issues.

    Args:
        text: Text to translate
        source_lang: Source language code
        target_lang: Target language code
        max_retries: Maximum retry attempts

    Returns:
        Translated text
    """
    from huggingface_hub import InferenceClient
    import time
    import re

    client = InferenceClient(token=settings.HUGGINGFACE_API_KEY if settings.HUGGINGFACE_API_KEY else None)
    model = "Helsinki-NLP/opus-mt-en-ur"

    # Split text into sentences to avoid token limit (512 tokens for this model)
    # Split by period, exclamation, question mark followed by space
    sentences = re.split(r'([.!?]+\s+|\n\n)', text)

    # Translate in batches of ~200 words (well under 512 token limit)
    translated_parts = []
    current_batch = ""

    for sent in sentences:
        if not sent.strip():
            translated_parts.append(sent)
            continue

        # If adding this sentence exceeds ~200 words, translate current batch
        word_count = len((current_batch + sent).split())
        if word_count > 200 and current_batch:
            # Translate current batch
            for attempt in range(max_retries):
                try:
                    result = client.translation(current_batch.strip(), model=model)
                    if result and 'translation_text' in result:
                        translated_parts.append(result['translation_text'])
                        break
                    elif attempt == max_retries - 1:
                        translated_parts.append(current_batch)  # Fallback to original
                except Exception as e:
                    if attempt == max_retries - 1:
                        translated_parts.append(current_batch)  # Fallback to original
                    time.sleep(2)

            # Start new batch
            current_batch = sent
        else:
            current_batch += sent

    # Translate remaining batch
    if current_batch.strip():
        for attempt in range(max_retries):
            try:
                result = client.translation(current_batch.strip(), model=model)
                if result and 'translation_text' in result:
                    translated_parts.append(result['translation_text'])
                    break
                elif attempt == max_retries - 1:
                    translated_parts.append(current_batch)
            except Exception as e:
                if attempt == max_retries - 1:
                    translated_parts.append(current_batch)
                time.sleep(2)

    return ' '.join(translated_parts)


async def translate_text(text: str, source_lang: str = "en", target_lang: str = "ur", max_retries: int = 3) -> str:
    """Async wrapper for translate_text_sync."""
    import asyncio
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, translate_text_sync, text, source_lang, target_lang, max_retries)


async def translate_and_index_chapter(
    chapter_path: Path,
    openai_client: OpenAI,
    qdrant_client: QdrantClient,
    target_lang: str = "ur",
    batch_delay: float = 2.0
):
    """
    Translate a chapter to Urdu and index it in Qdrant.

    Args:
        chapter_path: Path to .mdx file
        openai_client: OpenAI client for embeddings
        qdrant_client: Qdrant client
        target_lang: Target language code
        batch_delay: Delay between chunks to avoid rate limits
    """
    print(f"\n📄 Processing: {chapter_path.name}")

    # Parse frontmatter
    post = frontmatter.load(chapter_path)
    metadata = post.metadata
    content = post.content

    # Remove MDX-specific syntax
    content = re.sub(r"import .+ from .+;?\n", "", content)
    content = re.sub(r"<PersonalizedChapter[^>]*>", "", content)
    content = re.sub(r"</PersonalizedChapter>", "", content)

    # Chunk content
    chunks = chunk_markdown(content, max_tokens=800, overlap_tokens=150)  # Smaller chunks for translation

    print(f"   Chunks: {len(chunks)}")
    print(f"   Starting translation to {target_lang.upper()}...")

    # Translate and generate embeddings
    points = []
    successful_translations = 0

    for idx, chunk in enumerate(chunks):
        try:
            # Translate chunk
            print(f"   Translating chunk {idx+1}/{len(chunks)}...", end=" ")
            translated_chunk = await translate_text(chunk, source_lang="en", target_lang=target_lang)

            if translated_chunk == chunk:
                print("⚠️  (translation failed, using original)")
            else:
                print("✅")
                successful_translations += 1

            # Generate embedding for translated text
            response = openai_client.embeddings.create(
                model="text-embedding-3-large",
                input=translated_chunk
            )
            vector = response.data[0].embedding

            # Create unique ID
            chunk_id = int(hashlib.md5(f"{metadata['id']}-{target_lang}-chunk-{idx}".encode()).hexdigest(), 16) % (2**63)

            # Create point with translation metadata
            point = PointStruct(
                id=chunk_id,
                vector=vector,
                payload={
                    "content": translated_chunk,
                    "chapter_id": metadata['id'],
                    "chapter_title": metadata.get('title', ''),
                    "sidebar_label": metadata.get('sidebar_label', ''),
                    "url": f"/chapters/{metadata['id']}",
                    "chunk_index": idx,
                    "total_chunks": len(chunks),
                    "difficulty": "intermediate",
                    "tokens": count_tokens(translated_chunk),
                    "language": target_lang,
                    "source_language": "en",
                    "is_translation": True,
                    "translation_service": "libretranslate-api"
                }
            )
            points.append(point)

            # Small delay to avoid overwhelming the API
            if idx < len(chunks) - 1:
                await asyncio.sleep(batch_delay)

        except Exception as e:
            print(f"\n   ❌ Error processing chunk {idx+1}: {e}")
            continue

    if not points:
        print(f"   ❌ No chunks translated successfully")
        return False

    # Upload to Qdrant
    try:
        qdrant_client.upsert(
            collection_name="textbook_chapters",
            points=points
        )
        print(f"   ✅ Uploaded {len(points)} {target_lang.upper()} chunks to Qdrant ({successful_translations}/{len(chunks)} translated)")
        return True
    except Exception as e:
        print(f"   ❌ Failed to upload to Qdrant: {e}")
        return False


async def translate_toc(
    chapter_files: list[Path],
    openai_client: OpenAI,
    qdrant_client: QdrantClient,
    target_lang: str = "ur"
):
    """
    Translate and index the Table of Contents.

    Args:
        chapter_files: List of chapter files
        openai_client: OpenAI client
        qdrant_client: Qdrant client
        target_lang: Target language code
    """
    print("📑 Translating Table of Contents...")

    # Build TOC
    total_chapters = len(chapter_files)

    toc_lines = [
        "# Table of Contents - Complete Textbook Structure",
        "",
        f"TOTAL NUMBER OF CHAPTERS: {total_chapters}",
        "",
        f"This textbook contains exactly {total_chapters} chapters covering Physical AI and Humanoid Robotics:",
        ""
    ]

    for idx, chapter_file in enumerate(chapter_files, 1):
        try:
            post = frontmatter.load(chapter_file)
            title = post.metadata.get('title', post.metadata.get('sidebar_label', chapter_file.stem))
            toc_lines.append(f"Chapter {idx}: {title}")
        except Exception as e:
            print(f"   ⚠️  Error reading {chapter_file.name}: {e}")
            toc_lines.append(f"Chapter {idx}: {chapter_file.stem}")

    toc_content = "\n".join(toc_lines)

    # Translate TOC
    print(f"   Translating TOC to {target_lang.upper()}...")
    translated_toc = await translate_text(toc_content, source_lang="en", target_lang=target_lang)

    # Generate embedding
    response = openai_client.embeddings.create(
        model="text-embedding-3-large",
        input=translated_toc
    )
    vector = response.data[0].embedding

    # Create unique ID for translated TOC
    toc_id = int(hashlib.md5(f"TABLE_OF_CONTENTS_{target_lang}".encode()).hexdigest(), 16) % (2**63)

    # Create TOC point
    toc_point = PointStruct(
        id=toc_id,
        vector=vector,
        payload={
            "content": translated_toc,
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
            "tokens": count_tokens(translated_toc),
            "language": target_lang,
            "source_language": "en",
            "is_translation": True,
            "translation_service": "libretranslate-api"
        }
    )

    # Upload TOC to Qdrant
    qdrant_client.upsert(
        collection_name="textbook_chapters",
        points=[toc_point]
    )

    print(f"   ✅ Translated TOC uploaded")


async def translate_all_chapters(target_lang: str = "ur", start_from: int = 0, batch_delay: float = 2.0):
    """
    Translate all textbook chapters to target language.

    Args:
        target_lang: Target language code (default: "ur" for Urdu)
        start_from: Chapter index to start from (for resuming)
        batch_delay: Delay between chunks in seconds
    """
    print(f"🌍 Starting translation to {target_lang.upper()} for all chapters...")
    print(f"⏱️  Batch delay: {batch_delay}s between chunks\n")

    # Initialize clients
    openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)
    qdrant_client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY
    )

    # Find docs directory
    script_dir = Path(__file__).parent
    docs_dir = script_dir.parent.parent / "physical-ai-humanoid-textbook" / "docs"

    if not docs_dir.exists():
        print(f"❌ Docs directory not found: {docs_dir}")
        return

    # Process all week-*.mdx files
    chapter_files = sorted(docs_dir.glob("week-*.mdx"))

    print(f"📚 Found {len(chapter_files)} chapters to translate\n")

    if start_from > 0:
        print(f"▶️  Resuming from chapter {start_from + 1}\n")
        chapter_files = chapter_files[start_from:]

    # Translate TOC first
    await translate_toc(chapter_files, openai_client, qdrant_client, target_lang)

    # Translate chapters
    successful = 0
    failed = 0

    for idx, chapter_file in enumerate(chapter_files, start=start_from):
        try:
            result = await translate_and_index_chapter(
                chapter_file,
                openai_client,
                qdrant_client,
                target_lang,
                batch_delay
            )
            if result:
                successful += 1
            else:
                failed += 1
        except Exception as e:
            print(f"   ❌ Failed to process chapter: {e}")
            failed += 1
            continue

        # Progress update
        print(f"   Progress: {idx + 1}/{len(chapter_files) + start_from} chapters")

    # Final summary
    print(f"\n\n{'='*60}")
    print(f"🎉 Translation complete!")
    print(f"{'='*60}")
    print(f"   Language: {target_lang.upper()}")
    print(f"   Successful: {successful}")
    print(f"   Failed: {failed}")
    print(f"   Total chapters: {len(chapter_files)}")
    print(f"\n✅ Translations indexed in Qdrant!")
    print(f"   Collection: textbook_chapters")
    print(f"   Filter: language='{target_lang}'")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Translate all textbook chapters to Urdu")
    parser.add_argument("--lang", default="ur", help="Target language code (default: ur)")
    parser.add_argument("--start-from", type=int, default=0, help="Resume from chapter index")
    parser.add_argument("--delay", type=float, default=2.0, help="Delay between chunks in seconds (default: 2.0)")

    args = parser.parse_args()

    asyncio.run(translate_all_chapters(
        target_lang=args.lang,
        start_from=args.start_from,
        batch_delay=args.delay
    ))
