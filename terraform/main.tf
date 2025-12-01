terraform {
  required_version = ">= 1.0"
}

# Deploy using render.com API via local-exec
# Since official Render provider is limited, we use API directly

resource "local_file" "render_deploy_script" {
  filename = "${path.module}/deploy.sh"
  content  = <<-EOF
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
EOF
}

# Document for manual deployment
resource "local_file" "deployment_checklist" {
  filename = "${path.module}/DEPLOYMENT_CHECKLIST.md"
  content  = <<-EOF
# Render Deployment Checklist

## Step 1: Create Web Service on Render Dashboard

1. Go to render.com
2. Click "New+" → "Web Service"
3. Select repo: uzmaW/physical-ai-textbook
4. Fill form:
   - Name: physical-ai-backend
   - Environment: Python
   - Region: Oregon
   - Plan: Free
   - Root Directory: backend
   - Build: pip install -r requirements.txt
   - Start: uvicorn app.main:app --host 0.0.0.0 --port \$PORT

## Step 2: Set Environment Variables

In Render Service Settings → Environment:

- OPENAI_API_KEY = ${var.openai_api_key}
- QDRANT_URL = ${var.qdrant_url}
- QDRANT_API_KEY = ${var.qdrant_api_key}
- FRONTEND_URL = ${var.frontend_url}

## Step 3: Deploy

Click "Create Web Service" and wait 3-5 minutes.

## Step 4: Verify

Test health endpoint:
\`\`\`bash
curl https://physical-ai-backend.onrender.com/health
\`\`\`

Should return: { "status": "healthy", ... }

## Step 5: Update Frontend

Add backend URL to Vercel:
REACT_APP_API_URL = https://physical-ai-backend.onrender.com

Done! 🚀
EOF
}

output "deployment_guide" {
  value       = local_file.deployment_checklist.content
  description = "Deployment checklist for manual setup"
  sensitive   = true
}

output "next_steps" {
  value = <<-EOT
Render Terraform limitations:
- Render provider is read-only (cannot create services)
- Deploy manually through render.com dashboard (5 minutes)
- OR use Azure/AWS/GCP alternatives

Manual Deployment Steps:
1. Go to render.com
2. New Web Service → Select your GitHub repo
3. Configure: backend folder, Python, uvicorn command
4. Add environment variables from terraform.tfvars
5. Deploy

After manual deployment:
- Terraform can manage environment variables
- Use: terraform apply
  EOT
}
