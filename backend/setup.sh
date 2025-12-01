#!/bin/bash
# Backend Setup Script
# Sets up Python environment and dependencies for the Physical AI textbook backend

set -e  # Exit on error

echo "🚀 Physical AI Textbook Backend Setup"
echo "======================================"

# Check Python version
echo ""
echo "📋 Checking Python version..."
python3 --version || { echo "❌ Python 3 not found. Please install Python 3.8+"; exit 1; }

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo ""
    echo "🐍 Creating Python virtual environment..."
    python3 -m venv venv
    echo "✅ Virtual environment created"
else
    echo "✅ Virtual environment already exists"
fi

# Activate virtual environment
echo ""
echo "🔌 Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo ""
echo "⬆️  Upgrading pip..."
pip install --upgrade pip

# Install dependencies
echo ""
echo "📦 Installing dependencies..."
pip install -r requirements.txt

echo ""
echo "✅ Dependencies installed successfully"

# Check for .env file
echo ""
if [ ! -f ".env" ]; then
    echo "⚠️  No .env file found"
    echo "📝 Creating .env from .env.example..."
    cp .env.example .env
    echo ""
    echo "⚠️  IMPORTANT: Edit .env and add your API keys:"
    echo "   - OPENAI_API_KEY"
    echo "   - QDRANT_URL and QDRANT_API_KEY"
    echo "   - DATABASE_URL (if using PostgreSQL)"
    echo ""
    echo "💡 Run: nano .env (or use your preferred editor)"
else
    echo "✅ .env file exists"
fi

echo ""
echo "======================================"
echo "🎉 Setup Complete!"
echo ""
echo "Next steps:"
echo "  1. Edit .env and add your API keys"
echo "  2. Setup Qdrant: python scripts/setup_qdrant.py"
echo "  3. Index textbook: python scripts/index_textbook.py"
echo "  4. Start server: ./run.sh"
echo ""
