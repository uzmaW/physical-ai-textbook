"""
Capstone Chat Endpoint
Provides /api/capstone/chat/ for capstone-specific Q&A
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, List
from app.services.capstone_chat import get_capstone_chat

router = APIRouter(prefix="/api/capstone", tags=["capstone"])


class CapstoneQuestion(BaseModel):
    """Request model for capstone questions."""
    question: str
    user_level: Optional[str] = "intermediate"  # beginner/intermediate/advanced
    conversation_id: Optional[str] = None


class Citation(BaseModel):
    """Citation metadata."""
    section: str
    difficulty: str
    relevance_score: int


class CapstoneResponse(BaseModel):
    """Response model for capstone answers."""
    question: str
    answer: str
    citations: List[Citation]
    sources_count: int
    user_level: str
    knowledge_base: str
    knowledge_base_sections: int


@router.post("/chat/", response_model=CapstoneResponse)
async def chat_capstone(request: CapstoneQuestion) -> CapstoneResponse:
    """
    Answer questions about the capstone project.

    Args:
        request: Question and metadata

    Returns:
        Answer with citations and source information

    Example:
        POST /api/capstone/chat/
        {
            "question": "What is the VLA task planner?",
            "user_level": "intermediate"
        }

    Response:
        {
            "question": "What is the VLA task planner?",
            "answer": "The VLA Task Planner is the brain of the system...",
            "citations": [
                {
                    "section": "VLA Task Planner",
                    "difficulty": "advanced",
                    "relevance_score": 10
                }
            ],
            "sources_count": 1,
            "user_level": "intermediate",
            "knowledge_base": "capstone_project",
            "knowledge_base_sections": 11
        }
    """
    try:
        # Validate question
        if not request.question or len(request.question.strip()) == 0:
            raise HTTPException(status_code=400, detail="Question cannot be empty")

        if len(request.question) > 1000:
            raise HTTPException(status_code=400, detail="Question too long (max 1000 chars)")

        # Get capstone chat service
        capstone_chat = get_capstone_chat()

        # Check if knowledge base is loaded
        if not capstone_chat.database.get("sections"):
            raise HTTPException(
                status_code=503,
                detail="Capstone knowledge base not loaded. Run 'python scripts/index_capstone_json.py'"
            )

        # Get response from capstone chat
        response = await capstone_chat.ask(
            question=request.question,
            user_level=request.user_level or "intermediate"
        )

        # Convert to response model
        citations = [
            Citation(
                section=c["section"],
                difficulty=c["difficulty"],
                relevance_score=c["relevance_score"]
            )
            for c in response["citations"]
        ]

        return CapstoneResponse(
            question=response["question"],
            answer=response["answer"],
            citations=citations,
            sources_count=response["sources_count"],
            user_level=response["user_level"],
            knowledge_base=response["knowledge_base"],
            knowledge_base_sections=response["knowledge_base_sections"]
        )

    except HTTPException:
        raise
    except Exception as e:
        print(f"❌ Capstone chat error: {e}")
        raise HTTPException(status_code=500, detail=f"Chat error: {str(e)}")


@router.get("/knowledge-base/stats")
async def get_kb_stats():
    """
    Get capstone knowledge base statistics.

    Returns:
        Metadata about the indexed knowledge base
    """
    try:
        capstone_chat = get_capstone_chat()
        db = capstone_chat.database

        sections = db.get("sections", [])
        total_content = sum(s.get("content_length", 0) for s in sections)

        return {
            "title": db.get("metadata", {}).get("title"),
            "total_sections": len(sections),
            "total_content_length": total_content,
            "sections": [
                {
                    "id": s.get("id"),
                    "section": s.get("section"),
                    "difficulty": s.get("difficulty"),
                    "keywords": s.get("keywords", []),
                    "content_length": s.get("content_length", 0)
                }
                for s in sections
            ],
            "created_at": db.get("metadata", {}).get("created_at")
        }

    except Exception as e:
        print(f"❌ Knowledge base stats error: {e}")
        raise HTTPException(status_code=500, detail=f"Stats error: {str(e)}")


@router.post("/search")
async def search_knowledge_base(request: CapstoneQuestion):
    """
    Search knowledge base without generating LLM response.

    Args:
        request: Question to search for

    Returns:
        List of matching sections with scores
    """
    try:
        capstone_chat = get_capstone_chat()

        results = capstone_chat.search_knowledge_base(
            query=request.question,
            top_k=5
        )

        return {
            "query": request.question,
            "results": [
                {
                    "id": r["id"],
                    "section": r["section"],
                    "difficulty": r["difficulty"],
                    "relevance_score": r["score"],
                    "keywords": r["keywords"],
                    "preview": r["preview"][:200] + "..."
                }
                for r in results
            ],
            "total_results": len(results)
        }

    except Exception as e:
        print(f"❌ Search error: {e}")
        raise HTTPException(status_code=500, detail=f"Search error: {str(e)}")
