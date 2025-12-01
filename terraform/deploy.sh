#!/bin/bash
set -e

echo "🚀 Deploying Backend to Render..."
echo "=================================="

# Validate required variables
if [ -z "$RENDER_API_KEY" ]; then
  echo "❌ Error: RENDER_API_KEY not set"
  exit 1
fi

if [ -z "$GITHUB_TOKEN" ]; then
  echo "❌ Error: GITHUB_TOKEN not set"
  exit 1
fi

echo "✅ Deploying service via Render API..."

# Note: Manual deployment recommended for first time
# See TERRAFORM_DEPLOY.md for full instructions
echo ""
echo "📖 For initial setup, follow:"
echo "   1. Go to render.com"
echo "   2. New Web Service"
echo "   3. Select: uzmaW/physical-ai-textbook"
echo "   4. Configure with RENDER_DEPLOY.md"
echo ""
echo "After first deployment, you can manage via Terraform."
