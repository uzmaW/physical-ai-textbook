#!/bin/bash
# Backend Run Script
# Starts the FastAPI development server

set -e

echo "🚀 Starting Physical AI Textbook Backend"
echo "========================================"

# Check if venv exists
if [ ! -d "venv" ]; then
    echo "❌ Virtual environment not found"
    echo "💡 Run ./setup.sh first"
    exit 1
fi

# Activate virtual environment
source venv/bin/activate

# Check if .env exists
if [ ! -f ".env" ]; then
    echo "⚠️  No .env file found"
    echo "📝 Creating from .env.example..."
    cp .env.example .env
    echo ""
    echo "⚠️  IMPORTANT: Edit .env and add your API keys before starting"
    echo "Press Ctrl+C to exit and edit .env"
    sleep 5
fi

# Start server
echo ""
echo "🌐 Starting FastAPI server..."
echo "📡 API Documentation: http://localhost:8000/docs"
echo "💚 Health Check: http://localhost:8000/health"
echo ""
echo "Press Ctrl+C to stop"
echo ""

uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
