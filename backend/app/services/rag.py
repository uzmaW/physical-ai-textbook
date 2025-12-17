"""
RAG (Retrieval-Augmented Generation) Service
Semantic search in Qdrant + GPT-4o-mini for intelligent Q&A
Includes local capstone knowledge base for quick fallback search
"""

from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue
from app.config import settings
from typing import List, Dict, Optional
import json
import os


class RAGService:
    """Retrieval-Augmented Generation service for textbook Q&A."""

    def __init__(self):
        self.openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)
        self.qdrant_client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY
        )
        self.collection_name = "textbook_chapters"
        
        # Load capstone knowledge base for local search
        self.capstone_kb = self._load_capstone_kb()
    
    def _load_capstone_kb(self) -> dict:
        """Load capstone knowledge base from JSON file."""
        kb_path = "/mnt/workingdir/piaic_projects/humanoid_ai/backend/data/capstone_knowledge_base.json"
        try:
            if os.path.exists(kb_path):
                with open(kb_path, 'r') as f:
                    return json.load(f)
        except Exception as e:
            print(f"⚠️  Failed to load capstone KB: {e}")
        return {"sections": []}
    
    def _search_capstone_kb(self, query: str, top_k: int = 5) -> List[Dict]:
        """Search capstone knowledge base using keyword matching."""
        if not self.capstone_kb.get("sections"):
            return []
        
        query_lower = query.lower()
        results = []
        
        for section in self.capstone_kb["sections"]:
            score = 0
            
            # Check keywords (weight: 10)
            for keyword in section.get("keywords", []):
                if keyword.lower() in query_lower:
                    score += 10
            
            # Check section title (weight: 5)
            score += query_lower.count(section.get("section", "").lower()) * 5
            
            # Check content (weight: 1)
            query_words = [w for w in query_lower.split() if len(w) > 3]
            for word in query_words:
                if word in section.get("content", "").lower():
                    score += 1
            
            if score > 0:
                results.append({
                    "content": section.get("content", ""),
                    "chapter_id": "capstone",
                    "chapter_title": section.get("section", "Capstone"),
                    "url": "#capstone",
                    "score": score,
                    "chunk_index": 0,
                    "language": "en",
                    "is_translation": False
                })
        
        results.sort(key=lambda x: x["score"], reverse=True)
        return results[:top_k]

    async def embed_query(self, query: str) -> List[float]:
        """
        Generate embedding vector for user query.

        Args:
            query: User's question

        Returns:
            3072-dimensional embedding vector
        """
        response = self.openai_client.embeddings.create(
            model="text-embedding-3-large",  # Must match indexed data
            input=query
        )
        return response.data[0].embedding

    async def search_context(
        self,
        query: str,
        user_level: str = "intermediate",
        language: str = "en",
        top_k: int = 5,
        chapter_filter: Optional[str] = None
    ) -> List[Dict]:
        """
        Search for relevant context in vector store with language support.

        Args:
            query: User's question
            user_level: Difficulty filter (beginner/intermediate/advanced)
            language: Search language (en/ur) - searches both original and translations
            top_k: Number of results to return
            chapter_filter: Optional specific chapter ID

        Returns:
            List of relevant text chunks with metadata
        """
        # Generate query embedding
        query_vector = await self.embed_query(query)

        # Build filters
        filters = []

        # For structural queries (chapters, TOC, book info), don't apply difficulty filter
        # These queries need access to TOC which has beginner difficulty
        structural_keywords = ["chapter", "chapters", "total", "how many", "structure", "organization", "toc", "table", "book"]
        query_lower = query.lower()
        is_structural_query = any(keyword in query_lower for keyword in structural_keywords)

        # Only apply difficulty filter for non-structural queries
        if user_level and user_level != "all" and not is_structural_query:
            filters.append(
                FieldCondition(
                    key="difficulty",
                    match=MatchValue(value=user_level)
                )
            )

        if chapter_filter:
            filters.append(
                FieldCondition(
                    key="chapter_id",
                    match=MatchValue(value=chapter_filter)
                )
            )

        # If searching in non-English, also filter by language
        if language and language != "en":
            filters.append(
                FieldCondition(
                    key="language",
                    match=MatchValue(value=language)
                )
            )

        # Search Qdrant - retrieve more results to ensure TOC is included
        search_results = self.qdrant_client.query_points(
            collection_name=self.collection_name,
            query=query_vector,
            limit=top_k,
            query_filter=Filter(must=filters) if filters else None,
            with_payload=True,
            with_vectors=False
        ).points

        # If no difficulty filter (or beginner level), also search without difficulty filter
        # to ensure we get structural questions like TOC
        if (not filters or user_level == "beginner") and len(search_results) < top_k:
            search_results_unfiltered = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=top_k * 2,
                with_payload=True,
                with_vectors=False
            ).points
            
            # Merge and deduplicate
            result_ids = {r.id for r in search_results}
            for r in search_results_unfiltered:
                if r.id not in result_ids and len(search_results) < top_k:
                    search_results.append(r)

        # Format results
        context_chunks = []
        for result in search_results:
            context_chunks.append({
                "content": result.payload.get("content", ""),
                "chapter_id": result.payload.get("chapter_id", "unknown"),
                "chapter_title": result.payload.get("chapter_title", result.payload.get("title", "Unknown")),
                "url": result.payload.get("url", "#"),
                "score": result.score,
                "chunk_index": result.payload.get("chunk_index", 0),
                "language": result.payload.get("language", "en"),
                "is_translation": result.payload.get("is_translation", False),
                "is_toc": result.payload.get("is_toc", False)
            })

        return context_chunks

    async def generate_answer(
        self,
        question: str,
        context_chunks: List[Dict],
        user_level: str = "intermediate",
        language: str = "en"
    ) -> Dict:
        """
        Generate answer using GPT-4o-mini with retrieved context.

        Args:
            question: User's question
            context_chunks: Retrieved relevant chunks from Qdrant
            user_level: User's proficiency level
            language: Response language (en/ur)

        Returns:
            Dict with answer and citations
        """
        # Build context string
        context_text = "\n\n---\n\n".join([
            f"[{chunk['chapter_title']}]\n{chunk['content']}"
            for chunk in context_chunks
        ])

        # System prompt tailored to user level
        level_prompts = {
            "beginner": "Explain concepts simply with analogies. Avoid jargon.",
            "intermediate": "Provide balanced explanations with some technical detail.",
            "advanced": "Be concise. Focus on advanced concepts and research details."
        }

        system_prompt = f"""You are an expert AI tutor for Physical AI and Humanoid Robotics.

User Level: {user_level}
Style: {level_prompts.get(user_level, level_prompts['intermediate'])}
Language: {'Urdu' if language == 'ur' else 'English'}

Use the following context from the textbook to answer questions accurately.
Always cite sources by mentioning the chapter title.
If the answer isn't in the context, say so honestly.

Context from textbook:
{context_text}
"""

        # Generate streaming response
        chat_response = self.openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": question}
            ],
            temperature=0.7,
            max_tokens=500,  # Reduced for faster responses
            stream=True  # Enable streaming
        )

        # Collect streamed response
        answer = ""
        for chunk in chat_response:
            if chunk.choices[0].delta.content:
                answer += chunk.choices[0].delta.content

        # Extract top 3 citations
        citations = [
            {
                "chapter": chunk["chapter_title"],
                "url": chunk["url"],
                "relevance": chunk["score"]
            }
            for chunk in context_chunks[:3]
        ]

        return {
            "answer": answer,
            "citations": citations,
            "sourcesCount": len(context_chunks),
            "model": "gpt-4o-mini"
        }

    async def ask(
        self,
        question: str,
        selected_text: Optional[str] = None,
        user_level: str = "intermediate",
        language: str = "en"
    ) -> Dict:
        """
        Main RAG pipeline: search + generate.

        Args:
            question: User's question
            selected_text: Optional context from page
            user_level: Difficulty level
            language: Response language

        Returns:
            Answer with citations
        """
        # Augment query with selected text context
        enhanced_query = question
        if selected_text:
            enhanced_query = f"Context: '{selected_text[:200]}...'\n\nQuestion: {question}"

        # Search for relevant context - retrieve 5 to ensure TOC is included if relevant
        context_chunks = await self.search_context(
            query=enhanced_query,
            user_level=user_level,
            language=language,
            top_k=5  # Retrieve more results to ensure TOC is included
        )

        # Prioritize TOC chunks for structural questions (chapters, structure, organization)
        structural_keywords = ["chapter", "chapters", "total", "how many", "structure", "organization", "toc", "table"]
        question_lower = question.lower()
        has_structural_query = any(keyword in question_lower for keyword in structural_keywords)
        
        # For structural questions, ensure TOC is included
        toc_chunk = None
        non_toc_chunks = []
        
        for chunk in context_chunks:
            if chunk.get("is_toc"):
                toc_chunk = chunk
            else:
                non_toc_chunks.append(chunk)
        
        # Arrange chunks: TOC first for structural questions, then others
        if has_structural_query and toc_chunk:
            # Put TOC first for structural questions
            context_to_use = [toc_chunk] + non_toc_chunks[:2]
        elif toc_chunk and len(non_toc_chunks) < 2:
            # Include TOC if we don't have enough other chunks
            context_to_use = [toc_chunk] + non_toc_chunks[:2]
        else:
            # Otherwise use top 3 results
            context_to_use = context_chunks[:3]

        # Generate answer - pass selected context chunks
        result = await self.generate_answer(
            question=question,
            context_chunks=context_to_use,
            user_level=user_level,
            language=language
        )

        return result
