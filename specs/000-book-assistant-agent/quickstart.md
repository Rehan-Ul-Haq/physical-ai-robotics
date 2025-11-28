# Quickstart: Book Assistant AI Agent

**Feature Branch**: `002-book-assistant-agent`
**Date**: 2025-11-28
**Updated**: 2025-11-28 (Docusaurus Integration)

## Architecture Overview

- **Backend**: FastAPI + OpenAI ChatKit Server (`backend/`)
- **Frontend**: Integrated into existing Docusaurus site (`book-source/`) - NO separate frontend
- **Vector DB**: Qdrant Cloud
- **Database**: Neon Postgres (ThreadStore)

## Prerequisites

### Required Accounts

1. **OpenAI API Key**
   - Create at: https://platform.openai.com/api-keys
   - Required for: Embeddings (text-embedding-3-small) and Chat (GPT-4o-mini)

2. **Qdrant Cloud Account**
   - Create at: https://cloud.qdrant.io/
   - Free tier: 1GB storage, sufficient for initial book content
   - Get: Cluster URL and API Key

3. **Neon Postgres Account**
   - Create at: https://neon.tech/
   - Free tier: 0.5GB storage, sufficient for chat history
   - Get: Connection string (DATABASE_URL)

### Local Development Environment

- Python 3.11+
- Node.js 20+ (for Docusaurus)
- uv package manager
- Git

---

## Project Setup

### 1. Clone and Navigate

```bash
# Already in repo, switch to feature branch
git checkout 002-book-assistant-agent

# Backend directory structure already exists
ls backend/app/  # Should show main.py, models.py, etc.
```

### 2. Backend Setup (Python/FastAPI with uv)

```bash
cd backend

# Install dependencies with uv
uv sync

# Or if starting fresh:
uv init --name book-assistant-backend
uv add "fastapi>=0.114.1,<0.116" "openai>=1.40" "openai-chatkit>=1.1.2,<2" "httpx>=0.28,<0.29" "uvicorn[standard]>=0.36,<0.37" qdrant-client asyncpg python-dotenv pydantic
```

**Create `backend/.env`** (copy from `.env.example`):
```env
# OpenAI
OPENAI_API_KEY=sk-...

# Qdrant Cloud
QDRANT_URL=https://xxx-xxx.us-east4-0.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=...

# Neon Postgres
DATABASE_URL=postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require

# Server
HOST=127.0.0.1
PORT=8001
DEBUG=true
```

### 3. Docusaurus Setup (ChatWidget already integrated)

The ChatWidget is integrated directly into the Docusaurus site. No separate frontend needed.

```bash
cd book-source

# Install dependencies
npm install

# Configure API URL (optional, defaults to localhost:8001)
# Add to .env.local:
echo "BOOK_ASSISTANT_API_URL=http://localhost:8001" >> .env.local
```


---

## Database Setup

### Neon Postgres Schema (ChatKit ThreadStore)

Run the migration script against your Neon database:

```bash
cd backend

# Using psql
psql $DATABASE_URL < migrations/001_initial_schema.sql

# Or using Python
uv run python -c "
import asyncio
import asyncpg
import os
from dotenv import load_dotenv

load_dotenv()

async def migrate():
    conn = await asyncpg.connect(os.getenv('DATABASE_URL'))
    with open('migrations/001_initial_schema.sql') as f:
        await conn.execute(f.read())
    await conn.close()
    print('Migration complete')

asyncio.run(migrate())
"
```

**Schema creates**:
- `threads` table (ChatKit Thread storage)
- `thread_items` table (ChatKit ThreadItem storage)
- `feedback_events` table (user satisfaction signals)

### Qdrant Collection Setup

```bash
cd backend
uv run python scripts/setup_qdrant.py
```

---

## Initial Book Content Indexing

### Index Chapter 1

```bash
cd backend
uv run python scripts/index_book.py
```

This indexes content from `book-source/docs/01-robotic-nervous-system/01-introduction-to-physical-ai/`.

---

## Running the Application

### Start Backend Server

```bash
cd backend
uv run uvicorn app.main:app --host 127.0.0.1 --port 8001 --reload
```

### Start Docusaurus Dev Server

```bash
cd book-source
npm start
# Opens http://localhost:3000 with ChatWidget integrated
```

### Verify Setup

1. **Health Check**:
   ```bash
   curl http://localhost:8001/assistant/health
   ```
   Expected: `{"status": "healthy"}`

2. **Open Docusaurus**: Navigate to http://localhost:3000
   - Click the chat icon (bottom-right corner)
   - Ask: "What is Physical AI?"
   - Verify response with book content


---

## Development Workflow

### Backend Development

```bash
cd backend

# Run server with hot reload
uv run uvicorn app.main:app --reload --port 8001

# Run tests (when implemented)
uv run pytest

# Type checking
uv run mypy app/
```

### Docusaurus Development

```bash
cd book-source

# Start dev server (hot reload)
npm start

# Build for production
npm run build

# Type checking
npm run typecheck
```

---

## Environment Variables Reference

### Backend (`backend/.env`)

| Variable | Required | Description |
|----------|----------|-------------|
| `OPENAI_API_KEY` | Yes | OpenAI API key for embeddings and chat |
| `QDRANT_URL` | Yes | Qdrant Cloud cluster URL |
| `QDRANT_API_KEY` | Yes | Qdrant Cloud API key |
| `DATABASE_URL` | Yes | Neon Postgres connection string |
| `HOST` | No | Server host (default: 127.0.0.1) |
| `PORT` | No | Server port (default: 8001) |
| `DEBUG` | No | Enable debug mode (default: false) |

### Docusaurus (`book-source/.env.local`)

| Variable | Required | Description |
|----------|----------|-------------|
| `BOOK_ASSISTANT_API_URL` | No | Backend API URL (default: http://localhost:8001) |

---

## Troubleshooting

### Common Issues

1. **Qdrant connection timeout**
   - Check QDRANT_URL includes port `:6333`
   - Verify API key is correct

2. **Neon connection refused**
   - Ensure `?sslmode=require` in DATABASE_URL
   - Check Neon project is not paused (free tier pauses after inactivity)

3. **OpenAI rate limit**
   - Add exponential backoff for embedding calls
   - Use batch embedding endpoint for indexing

4. **CORS errors in browser**
   - Verify backend CORS middleware allows `http://localhost:3000`
   - Check `backend/app/main.py` CORS configuration

5. **ChatWidget not appearing**
   - Check browser console for errors
   - Verify `book-source/src/theme/Root.tsx` includes ChatWidget import
   - Check `BOOK_ASSISTANT_API_URL` is accessible


---

## Project Structure

```text
physical-ai-robotics/
├── backend/                      # FastAPI backend
│   ├── app/
│   │   ├── main.py               # FastAPI app + endpoints
│   │   ├── book_assistant_server.py
│   │   ├── book_assistant_agent.py
│   │   ├── postgres_thread_store.py
│   │   ├── qdrant_service.py
│   │   ├── models.py
│   │   ├── tools.py
│   │   └── config.py
│   ├── migrations/
│   │   └── 001_initial_schema.sql
│   ├── scripts/
│   │   ├── setup_qdrant.py
│   │   └── index_book.py
│   ├── pyproject.toml
│   └── .env.example
│
└── book-source/                  # Docusaurus site (ChatWidget integrated here)
    ├── src/
    │   ├── components/
    │   │   └── ChatWidget/
    │   │       ├── index.tsx     # Chat widget component
    │   │       └── styles.module.css
    │   └── theme/
    │       └── Root.tsx          # ChatWidget wrapped here
    ├── docs/                     # Book content (indexed for RAG)
    └── .env.example
```

---

## MVP Complete (Phase 3)

The MVP provides:
- Working `/assistant/chatkit` endpoint
- Chapter 1 indexed in Qdrant
- ChatWidget integrated in Docusaurus (appears on every page)
- SSE streaming for real-time responses
- Dark/light mode support via Docusaurus CSS variables
- Session management with localStorage

## Next Steps (Post-MVP)

1. **Phase 4 (US2)**: Text selection for context-specific queries
2. **Phase 5 (US3)**: Conversation history persistence
3. **Phase 6 (US4)**: Clickable source citations
4. **Phase 7**: Feedback collection, rate limiting, security
