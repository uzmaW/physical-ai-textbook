# Use pre-built runtime image from GitHub Container Registry
FROM ghcr.io/uzmaw/physical-ai-textbook:latest

# Override port for Hugging Face Spaces (7860 is standard)
EXPOSE 7860

# Override health check endpoint
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:7860/health')" || exit 1

# Override CMD to use Hugging Face Spaces port
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "7860", "--workers", "1"]
