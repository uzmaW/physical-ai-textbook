---
title: Physical AI Textbook Backend
emoji: 📚
colorFrom: blue
colorTo: green
sdk: docker
app_file: backend/app/main.py
pinned: false
---

# Physical AI & Humanoid Robotics Textbook

An intelligent, interactive learning platform featuring RAG, translation, voice synthesis, and AI-powered content.

## Repository Structure

- **backend/** - FastAPI backend (deployed to Hugging Face)
- **physical-ai-humanoid-textbook/** - Frontend (Docusaurus)
- **nhost/** - Nhost serverless functions
- **terraform/** - Infrastructure as Code

## Backend Deployment

The backend folder is deployed to Hugging Face Spaces as a Docker container. See [backend/README.md](backend/README.md) for details.

## Documentation

- [START_HERE.md](START_HERE.md) - Quick start guide
- [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md) - Deployment instructions
- [backend/README.md](backend/README.md) - Backend API documentation
