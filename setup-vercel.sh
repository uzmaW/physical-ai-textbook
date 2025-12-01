#!/bin/bash

# Vercel Frontend Setup Script
# Run this to configure environment variables for Vercel deployment

echo "🚀 Vercel Frontend Setup"
echo "======================="
echo ""

read -p "Enter REACT_APP_API_URL (e.g., https://physical-ai-backend.onrender.com): " api_url

echo ""
echo "📋 Environment Variables Summary:"
echo "================================="
echo "REACT_APP_API_URL: $api_url"
echo "REACT_APP_ENVIRONMENT: production"
echo ""

read -p "Correct? (y/n): " confirm
if [ "$confirm" != "y" ]; then
    echo "Aborted."
    exit 1
fi

echo ""
echo "📝 Next steps:"
echo "1. Go to https://vercel.com/dashboard"
echo "2. Select 'physical-ai-textbook' project"
echo "3. Go to Settings → Environment Variables"
echo "4. Add variables:"
echo ""
echo "   REACT_APP_API_URL = $api_url"
echo "   REACT_APP_ENVIRONMENT = production"
echo ""
echo "5. Save and project will auto-redeploy"
echo ""
echo "✅ Once deployed, visit:"
echo "   https://physical-ai-textbook.vercel.app"
echo ""
