# Deploy Backend to Render Using Terraform

## Prerequisites

1. **Terraform installed**: https://www.terraform.io/downloads
   ```bash
   terraform version
   ```

2. **Render account**: https://render.com

3. **API Keys ready**:
   - Render API Key
   - OpenAI API Key
   - Qdrant URL & API Key

---

## Step 1: Get Render API Key

1. Go to render.com/account/api-tokens
2. Create new API token
3. Copy the token (you'll only see it once)

---

## Step 2: Get Render Owner ID

1. Go to render.com/account/connections
2. Your owner ID is visible in the URL or account settings
3. Or run after login:
   ```bash
   curl -H "Authorization: Bearer YOUR_API_KEY" https://api.render.com/v1/account
   ```

---

## Step 3: Set Up Terraform Variables

1. Go to `terraform/` directory:
   ```bash
   cd terraform
   ```

2. Copy example file:
   ```bash
   cp terraform.tfvars.example terraform.tfvars
   ```

3. Edit `terraform.tfvars` with your values:
   ```hcl
   render_api_key  = "rnd_..."
   render_owner_id = "user_xxxxx"
   openai_api_key  = "sk-..."
   qdrant_url      = "https://xxxxx.qdrant.io"
   qdrant_api_key  = "ey..."
   frontend_url    = "https://physical-ai-textbook.vercel.app"
   ```

   **IMPORTANT**: Never commit `terraform.tfvars` (it's in .gitignore)

---

## Step 4: Initialize Terraform

```bash
terraform init
```

This downloads the Render provider and sets up the working directory.

---

## Step 5: Review Plan

```bash
terraform plan
```

Shows what will be created. Review the output.

---

## Step 6: Deploy Backend

```bash
terraform apply
```

Terraform will:
1. Ask for confirmation (type `yes`)
2. Create the web service on Render
3. Set environment variables
4. Output the backend URL

Wait 3-5 minutes for deployment to complete.

---

## Step 7: Get Backend URL

After `terraform apply` completes:

```bash
terraform output backend_url
```

You'll get something like:
```
https://physical-ai-backend.onrender.com
```

---

## Step 8: Update Frontend (Vercel)

Add this backend URL to Vercel:

1. Go to vercel.com → Project Settings → Environment Variables
2. Add:
   ```
   REACT_APP_API_URL = https://physical-ai-backend.onrender.com
   ```
3. Redeploy (or push to trigger auto-deploy)

---

## Verify Deployment

Test the backend:

```bash
curl https://physical-ai-backend.onrender.com/health
```

Should return:
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "services": {
    "database": "connected",
    "qdrant": "connected",
    "openai": "configured"
  }
}
```

---

## Managing with Terraform

### Update Environment Variables

Edit `terraform.tfvars`:
```hcl
openai_api_key = "sk-new-key"
```

Then:
```bash
terraform apply
```

### View Current State

```bash
terraform state show render_web_service.backend
```

### Destroy Backend (Delete Service)

```bash
terraform destroy
```

Confirm with `yes`. This will delete the Render service.

---

## Troubleshooting

### "Error: Provider not found"

```bash
terraform init
```

### "Invalid API key"

Check `terraform.tfvars`:
```bash
cat terraform.tfvars | grep render_api_key
```

### "Service creation failed"

Check Terraform output:
```bash
terraform apply 2>&1 | tail -20
```

### Render provider not available

Install manually:
```bash
mkdir -p ~/.terraform.plugins
cd terraform
terraform init -upgrade
```

---

## Useful Commands

```bash
# See all resources managed by Terraform
terraform state list

# Show specific resource details
terraform state show render_web_service.backend

# Format Terraform files
terraform fmt -recursive

# Validate syntax
terraform validate

# Plan with output to file
terraform plan -out=tfplan

# Apply from saved plan
terraform apply tfplan

# Unlock state (if stuck)
terraform force-unlock LOCK_ID
```

---

## Monitoring Backend

Once deployed:

1. **Render Dashboard**: https://render.com/dashboard
   - View logs
   - Check CPU/memory
   - See deployment history

2. **Health Endpoint**: `https://physical-ai-backend.onrender.com/health`

3. **API Docs**: `https://physical-ai-backend.onrender.com/docs`

---

## Cost

- **Free tier**: $0/month (with cold starts after 15 min inactivity)
- **Paid tier**: $7/month (always running)

To upgrade:
```bash
# Edit terraform/main.tf:
# Change: plan_id = "free"
# To:     plan_id = "starter"
terraform apply
```

---

**Backend deployed with Terraform!** 🚀

Next: Monitor logs and test chat functionality.
