# Docker Build Instructions

## Step 1: Build the main optimized image

```bash
docker build -t humanoid-ai-runtime:latest -f Dockerfile.base --target runtime .
```

## Step 2: Build the Hugging Face Spaces image

```bash
docker build -t humanoid-ai-spaces:latest -f Dockerfile .
```

## Step 3: Run locally for testing

```bash
docker run -p 7860:7860 humanoid-ai-spaces:latest
```

The application will be available at `http://localhost:7860`

## Notes

- The main `Dockerfile` uses multi-stage builds with a `builder` stage and `runtime` stage
- The `runtime` stage is tagged and used as the base for `Dockerfile.spaces`
- This reduces build time and avoids duplicating dependencies
- The Hugging Face Spaces version runs on port 7860 by default
