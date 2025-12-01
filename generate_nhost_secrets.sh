#!/bin/bash

echo "🔐 Generating Nhost Environment Variables"
echo "========================================"
echo ""

echo "✅ Required Secrets (copy these to Nhost dashboard):"
echo ""

echo "JWT_SECRET=$(openssl rand -hex 32)"
echo "HASURA_ADMIN_SECRET=$(openssl rand -hex 32)"
echo "HASURA_WEBHOOK_SECRET=$(openssl rand -hex 32)"

echo ""
echo "✅ Static Values:"
echo ""
echo "JWT_ALGORITHM=HS256"
echo "JWT_EXPIRATION_DAYS=30"
echo "DEBUG=false"
echo "API_VERSION=1.0.0"
echo "FRONTEND_URL=https://physical-ai-textbook.nhost.run"

echo ""
echo "📋 Don't forget to add your existing values:"
echo "- DATABASE_URL (from your Neon database)"
echo "- OPENAI_API_KEY"
echo "- QDRANT_URL" 
echo "- QDRANT_API_KEY"
echo "- OAuth credentials (if using)"
echo ""
echo "🚀 Go to: https://app.nhost.io → Your Project → Settings → Environment Variables"
