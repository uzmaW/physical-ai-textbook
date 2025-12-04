#!/usr/bin/env python3
"""
Direct test of Capstone Chat service
No dependencies on FastAPI or database connections
"""

import asyncio
import json
from app.services.capstone_chat import CapstoneChat


async def test_capstone_chat():
    """Test capstone chat service directly."""
    
    print("=" * 70)
    print("🚀 Capstone Chat Service - Direct Testing")
    print("=" * 70)
    
    # Initialize chat service
    print("\n📚 Initializing Capstone Chat Service...")
    chat = CapstoneChat()
    
    # Check database
    sections = chat.database.get("sections", [])
    print(f"✅ Knowledge base loaded: {len(sections)} sections")
    print()
    
    # Test queries
    test_queries = [
        "What is VLA?",
        "How do I evaluate the capstone project?",
        "What is the technology stack?",
        "Tell me about the project phases",
        "How do I debug my system?",
        "What are common pitfalls?",
    ]
    
    for i, query in enumerate(test_queries, 1):
        print(f"\n{'='*70}")
        print(f"Test {i}: {query}")
        print(f"{'='*70}")
        
        # Search
        results = chat.search_knowledge_base(query, top_k=3)
        print(f"\n🔍 Search Results: {len(results)} matches")
        for j, r in enumerate(results, 1):
            print(f"   {j}. {r['section']} (score: {r['score']})")
        
        # Generate response
        print(f"\n💬 Generating Response...")
        response = await chat.ask(query)
        
        print(f"\n📝 Answer:")
        answer_preview = response['answer'][:300] + "..." if len(response['answer']) > 300 else response['answer']
        print(f"   {answer_preview}")
        
        print(f"\n📚 Citations:")
        for citation in response['citations']:
            print(f"   - {citation['section']} ({citation['difficulty']})")
        
        print(f"\n📊 Metadata:")
        print(f"   Sources: {response['sources_count']}")
        print(f"   Knowledge base: {response['knowledge_base']}")
        print(f"   User level: {response['user_level']}")


async def main():
    """Run tests."""
    try:
        await test_capstone_chat()
        print(f"\n{'='*70}")
        print("✅ All tests passed!")
        print(f"{'='*70}")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())
