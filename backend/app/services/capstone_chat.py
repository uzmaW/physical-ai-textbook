"""
Capstone Chat Service
Local knowledge base search + LLM response generation
No API keys required for search - uses JSON-indexed knowledge base
"""

import json
import os
from typing import List, Dict, Optional
from openai import OpenAI
from app.config import settings

# Path to capstone knowledge base
KB_PATH = "/mnt/workingdir/piaic_projects/humanoid_ai/backend/data/capstone_knowledge_base.json"


class CapstoneChat:
    """Chat service for capstone project questions using local knowledge base."""

    def __init__(self):
        self.kb_path = KB_PATH
        self.database = self._load_database()
        
        # Optional: Initialize OpenAI client if API key is available
        self.openai_client = None
        if settings.OPENAI_API_KEY:
            try:
                self.openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)
            except Exception as e:
                print(f"⚠️  OpenAI client initialization failed: {e}")

    def _load_database(self) -> dict:
        """Load capstone knowledge base from JSON file."""
        try:
            with open(self.kb_path, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"⚠️  Knowledge base not found at {self.kb_path}")
            return {"metadata": {}, "sections": []}
        except json.JSONDecodeError as e:
            print(f"⚠️  Failed to parse knowledge base: {e}")
            return {"metadata": {}, "sections": []}

    def search_knowledge_base(
        self,
        query: str,
        top_k: int = 3,
        min_score: int = 1
    ) -> List[Dict]:
        """
        Search knowledge base using keyword matching.

        Args:
            query: User's question
            top_k: Number of results to return
            min_score: Minimum relevance score threshold

        Returns:
            List of relevant sections with scores
        """
        if not self.database.get("sections"):
            return []

        query_lower = query.lower()
        results = []

        for section in self.database["sections"]:
            score = 0

            # Check keywords (weight: 10)
            for keyword in section.get("keywords", []):
                if keyword.lower() in query_lower:
                    score += 10

            # Check section title (weight: 5)
            score += query_lower.count(section.get("section", "").lower()) * 5

            # Check content (weight: 1 per word match)
            query_words = [w for w in query_lower.split() if len(w) > 3]
            for word in query_words:
                if word in section.get("content", "").lower():
                    score += 1

            if score >= min_score:
                results.append({
                    "id": section.get("id"),
                    "section": section.get("section"),
                    "difficulty": section.get("difficulty"),
                    "keywords": section.get("keywords", []),
                    "content": section.get("content", ""),
                    "score": score,
                    "preview": section.get("preview", "")
                })

        # Sort by score and return top-k
        results.sort(key=lambda x: x["score"], reverse=True)
        return results[:top_k]

    async def generate_response(
        self,
        question: str,
        context: List[Dict]
    ) -> Dict:
        """
        Generate response using OpenAI (if available) or fallback to context.

        Args:
            question: User's question
            context: Retrieved relevant sections

        Returns:
            Response with answer and sources
        """
        # Build context string
        context_text = "\n\n---\n\n".join([
            f"[{c['section']}]\n{c['content']}"
            for c in context
        ])

        # If OpenAI client is available, use it
        if self.openai_client:
            try:
                system_prompt = """You are an expert AI tutor specializing in the Capstone Project "The Autonomous Humanoid".
                
You have access to the capstone project documentation. Answer questions accurately and cite the source sections.
If the answer isn't in the documentation, say so honestly.

Be concise but thorough. Use technical language appropriate for advanced students."""

                response = self.openai_client.chat.completions.create(
                    model="gpt-4o-mini",
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": f"Documentation:\n\n{context_text}\n\nQuestion: {question}"}
                    ],
                    temperature=0.7,
                    max_tokens=500,
                    stream=False
                )

                answer = response.choices[0].message.content
            except Exception as e:
                print(f"⚠️  OpenAI generation failed: {e}")
                answer = self._generate_fallback_answer(question, context)
        else:
            # Fallback: Use context directly
            answer = self._generate_fallback_answer(question, context)

        # Build citations
        citations = [
            {
                "section": c["section"],
                "difficulty": c["difficulty"],
                "relevance_score": c["score"]
            }
            for c in context[:3]
        ]

        return {
            "answer": answer,
            "citations": citations,
            "sources_count": len(context),
            "knowledge_base": "capstone_project"
        }

    def _generate_fallback_answer(self, question: str, context: List[Dict]) -> str:
        """
        Generate answer from context without LLM (fallback).

        Args:
            question: User's question
            context: Retrieved sections

        Returns:
            Answer text
        """
        if not context:
            return "I couldn't find information about that in the capstone knowledge base. Try asking about: VLA models, project phases, evaluation metrics, or the tech stack."

        # Find most relevant section
        top_section = context[0]

        # Try to extract relevant portion
        content = top_section["content"]
        query_words = [w.lower() for w in question.split() if len(w) > 3]

        # Find paragraph containing query words
        paragraphs = content.split("\n\n")
        relevant_para = None

        for para in paragraphs:
            para_lower = para.lower()
            matching_words = sum(1 for w in query_words if w in para_lower)
            if matching_words > 0:
                relevant_para = para
                break

        if relevant_para:
            answer = relevant_para
        else:
            # Use first 300 chars of content
            answer = content[:300] + "..."

        # Add attribution
        answer += f"\n\n**Source**: {top_section['section']} ({top_section['difficulty']})"

        return answer

    async def ask(
        self,
        question: str,
        user_level: str = "intermediate"
    ) -> Dict:
        """
        Main chat pipeline: search + generate.

        Args:
            question: User's question
            user_level: Difficulty level (beginner/intermediate/advanced)

        Returns:
            Complete response with answer and citations
        """
        # Search knowledge base
        context = self.search_knowledge_base(query=question, top_k=3)

        # Generate response
        response = await self.generate_response(question, context)

        # Add metadata
        response["question"] = question
        response["user_level"] = user_level
        response["knowledge_base_sections"] = len(self.database.get("sections", []))

        return response


# Singleton instance
_capstone_chat = None


def get_capstone_chat() -> CapstoneChat:
    """Get or create capstone chat service singleton."""
    global _capstone_chat
    if _capstone_chat is None:
        _capstone_chat = CapstoneChat()
    return _capstone_chat
