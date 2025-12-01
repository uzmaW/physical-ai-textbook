import sys
import os
from pathlib import Path

# Add the backend directory to Python path
backend_path = Path(__file__).parent.parent.parent / "backend"
sys.path.insert(0, str(backend_path))

from app.main import app
from nhost_functions import handler

# Wrap FastAPI app for Nhost
@handler
def main(event, context):
    return app
