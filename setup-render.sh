#!/bin/bash

# Render Backend Setup Script
# Run this to configure environment variables for Render deployment

echo "🚀 Render Backend Setup"
echo "====================="
echo ""

read -p "Enter OPENAI_API_KEY (sk-...): " openai_key
read -p "Enter QDRANT_URL (https://...): " qdrant_url
read -s -p "Enter QDRANT_API_KEY: " qdrant_key
echo ""
read -p "Enter FRONTEND_URL (e.g., https://xyz.vercel.app) [optional]: " frontend_url

echo ""
echo "📋 Environment Variables Summary:"
echo "================================="
echo "OPENAI_API_KEY: ${openai_key:0:10}..."
echo "QDRANT_URL: $qdrant_url"
echo "QDRANT_API_KEY: ${qdrant_key:0:10}..."
echo "FRONTEND_URL: $frontend_url"
echo ""

read -p "Correct? (y/n): " confirm
if [ "$confirm" != "y" ]; then
    echo "Aborted."
    exit 1
fi

echo ""
echo "📝 Next steps:"
echo "1. Go to https://render.com/dashboard"
echo "2. Select 'physical-ai-backend' service"
echo "3. Go to Environment tab"
echo "4. Click 'Add Environment Variable' for each:"
echo ""
echo "   OPENAI_API_KEY = $openai_key"
echo "   QDRANT_URL = $qdrant_url"
echo "   QDRANT_API_KEY = $qdrant_key"
echo "   FRONTEND_URL = $frontend_url"
echo ""
echo "5. Click 'Save' and service will auto-redeploy"
echo ""
