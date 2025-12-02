---
title: Physical AI Textbook Backend
emoji: 📚
colorFrom: blue
colorTo: green
sdk: docker
pinned: false
---

# Physical AI Textbook Backend

FastAPI backend for the Physical AI Textbook application.

## Features

- RAG (Retrieval Augmented Generation)
- Chat with conversation history
- Text translation (10+ languages)
- Voice synthesis and transcription
- OpenAI integration
- Vector search with Qdrant

## Environment Variables

Required:
- `DATABASE_URL` - PostgreSQL connection string
- `OPENAI_API_KEY` - OpenAI API key
- `QDRANT_URL` - Qdrant instance URL
- `QDRANT_API_KEY` - Qdrant API key
- `JWT_SECRET` - JWT signing secret
- `HASURA_ADMIN_SECRET` - Hasura admin secret

## Local Development

```bash
pip install -r requirements.txt
python -m uvicorn app.main:app --reload
```

## Documentation

API docs available at `/docs` when running.
