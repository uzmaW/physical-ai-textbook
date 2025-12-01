#!/bin/bash

echo "🧹 Clearing Docusaurus cache..."

# Remove build cache
rm -rf .docusaurus
rm -rf build
rm -rf node_modules/.cache

echo "✅ Cache cleared!"
echo ""
echo "Now run one of these commands:"
echo "  npm start    - Start development server"
echo "  npm run build - Build for production"
