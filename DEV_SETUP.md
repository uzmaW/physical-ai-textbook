# Development Setup

## Running the Application Locally

This application consists of:
- **Backend**: FastAPI server on `http://localhost:8000`
- **Frontend**: Docusaurus textbook on `http://localhost:3000`

### Prerequisites

- Python 3.10+
- Node.js 18+
- Backend dependencies: `pip install -r backend/requirements.txt`
- Frontend dependencies: `cd physical-ai-humanoid-textbook && npm install`

### Starting Services

#### 1. Backend (FastAPI)

```bash
cd backend
source venv/bin/activate
python -m uvicorn app.main:app --host 0.0.0.0 --port 8000
```

The backend will be available at `http://localhost:8000`

#### 2. Frontend (with API Proxy)

```bash
cd physical-ai-humanoid-textbook
node server-with-proxy.js
```

The frontend will be available at `http://localhost:3000` with:
- All `/api/*` requests proxied to `http://localhost:8000`
- Static files served from `build/` directory

### Features

**Translation Button**:
- Click the "ترجمہ" (Urdu translation) button on any chapter
- Content is translated to Urdu using Helsinki-NLP model
- Translations are cached in the database
- Button toggles between English and Urdu views

**Personalization Button**:
- Click "🎯 Personalize" to cycle through difficulty levels
- Beginner → Intermediate → Advanced

### API Endpoints

- `POST /api/translate/` - Translate content to Urdu
- `GET /api/translate/cached?userId={id}&chapterId={id}` - Get cached translation
- `DELETE /api/translate/cache/{userId}/{chapterId}` - Clear cache

### Troubleshooting

**404 on API calls**: Make sure both services are running and the proxy is configured correctly.

**Translation fails**: Check that `transformers` and `torch` are installed in the backend venv:
```bash
pip install transformers torch
```

**Frontend doesn't load**: Ensure the build directory exists:
```bash
cd physical-ai-humanoid-textbook && npm run build
```
