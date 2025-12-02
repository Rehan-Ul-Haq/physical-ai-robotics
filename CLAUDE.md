# Physical AI Robotics  Project Guide

**Version**: 1.0.0  
**Last Updated**: 2025-12-02

---

## Project Overview

A two-part hackathon project:
1. **Book Platform**  Docusaurus site about Physical AI & Humanoid Robotics
2. **Book Assistant**  RAG-powered chatbot using OpenAI Agents SDK

---

## Architecture

```
physical-ai-robotics/
 roboai/                    # Docusaurus book (Part 1)
    docs/                  # Book content (MDX)
    src/components/        # React components (ChatWidget)
    docusaurus.config.ts   # Site configuration

 backend/                   # FastAPI + OpenAI Agents SDK (Part 2)
    app/
       main.py                    # FastAPI application
       book_assistant_agent.py    # OpenAI Agents SDK agent definition
       book_assistant_server.py   # ChatKit server integration
       qdrant_service.py          # Vector search (RAG)
       tools.py                   # Agent tools (search_book_content)
    scripts/
        index_book.py              # Index book content to Qdrant

 .github/workflows/
     deploy.yml             # Deploy Docusaurus to GitHub Pages
     validate-content.yml   # Validate assets
```

---

## Technology Stack

### Frontend (Book Platform)
| Component | Technology |
|-----------|------------|
| Static Site | Docusaurus 3.x |
| Styling | Tailwind CSS |
| Chat Interface | Custom ChatWidget component |
| Hosting | GitHub Pages |

### Backend (Book Assistant)
| Component | Technology |
|-----------|------------|
| API Framework | FastAPI |
| AI Agent | **OpenAI Agents SDK** |
| Chat Protocol | OpenAI ChatKit SDK |
| Vector Database | Qdrant Cloud |
| Embeddings | OpenAI text-embedding-3-small |
| LLM | OpenAI GPT-4o-mini |
| Database | PostgreSQL (thread storage) |

### Key Distinction
- **FastAPI** = HTTP server, routes, middleware, CORS
- **OpenAI Agents SDK** = Agent definition, tools, instructions, conversation logic
- **ChatKit SDK** = Streaming protocol, thread management, SSE responses

---

## Development Commands

### Frontend (Docusaurus)
```powershell
cd roboai
npm install
npm run start          # Dev server at localhost:3000
npm run build          # Production build
```

### Backend (FastAPI + Agent)
```powershell
cd backend
uv sync                # Install dependencies
uv run uvicorn app.main:app --reload  # Dev server at localhost:8000
```

### Index Book Content
```powershell
cd backend
uv run python scripts/index_book.py
```

---

## Environment Variables

### Backend (.env)
```env
# OpenAI
OPENAI_API_KEY=sk-...

# Qdrant
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=...
QDRANT_COLLECTION_NAME=book_chunks

# PostgreSQL
DATABASE_URL=postgresql://user:pass@host:5432/db

# Server
HOST=0.0.0.0
PORT=8000
DEBUG=true
```

---

## RAG Architecture

```
User Question
    
    

  FastAPI (/assistant/chatkit)                           
                                                        
                                                        
  ChatKit Server (book_assistant_server.py)              
                                                        
                                                        
  OpenAI Agent (book_assistant_agent.py)                 
                                                        
     Tool: search_book_content                        
                                                       
                                                       
       Qdrant Vector Search  Relevant Chunks         
                                                        
                                                        
  LLM generates response with citations                  

    
    
Streaming Response to ChatWidget
```

---

## Book Content Structure

```
roboai/docs/
 00-preface/
 01-robotic-nervous-system/     # Part 1: ROS 2
    01-introduction-to-physical-ai/
    02-ros2-fundamentals/
 02-digital-twin/               # Part 2: Gazebo & Unity
 03-ai-robot-brain/             # Part 3: NVIDIA Isaac
 04-vision-language-action/     # Part 4: VLA
```

**Topic**: Physical AI & Humanoid Robotics (ROS 2, simulation, Sim-to-Real)

---

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/assistant/chatkit` | Main chat endpoint (streaming SSE) |
| GET | `/assistant/thread/{id}` | Get thread history |
| DELETE | `/assistant/thread/{id}` | Delete thread (new conversation) |
| GET | `/assistant/health` | Health check (DB, Qdrant, OpenAI) |
| GET | `/assistant/index/status` | Qdrant index stats |

---

## Deployment

### Frontend (GitHub Pages)
- Trigger: Push to `main` with changes in `roboai/**`
- Workflow: `.github/workflows/deploy.yml`
- URL: `https://rehan-ul-haq.github.io/physical-ai-robotics/`

### Backend
- Host on: Railway / Render / Fly.io / Azure Container Apps
- Required: PostgreSQL database, environment variables set
- Health check: `GET /assistant/health`

---

## Development Guidelines

### 1. Default to Action
Implement changes rather than suggesting. Read files before editing, make changes, commit when appropriate.

### 2. Backend Changes
When modifying the backend:
- **Agent behavior**  Edit `book_assistant_agent.py` (instructions, tools)
- **API routes**  Edit `main.py`
- **RAG/search logic**  Edit `qdrant_service.py`, `tools.py`
- **Models**  Edit `models.py`
- Run backend locally and test before committing

### 3. Frontend Changes
When modifying the book platform:
- **Book content**  Edit files in `roboai/docs/`
- **Chat widget**  Edit `roboai/src/components/ChatWidget/`
- **Site config**  Edit `roboai/docusaurus.config.ts`
- Run `npm run build` to verify no build errors

### 4. Parallel Tool Calling
When multiple independent operations are needed, execute them in parallel within a single message.

---

## Quick Reference

### Check Backend Health
```powershell
curl http://localhost:8000/assistant/health
```

### Test Chat Endpoint
```powershell
$body = '{"query": "What is Physical AI?"}'
Invoke-RestMethod -Uri "http://localhost:8000/assistant/chatkit" -Method POST -Body $body -ContentType "application/json"
```

### Rebuild Qdrant Index
```powershell
cd backend
uv run python scripts/index_book.py --force
```

### Check Git Status
```powershell
git status
git log --oneline -5
```

---

## Key Files Reference

| Purpose | File |
|---------|------|
| Agent definition | `backend/app/book_assistant_agent.py` |
| FastAPI routes | `backend/app/main.py` |
| Vector search | `backend/app/qdrant_service.py` |
| Agent tools | `backend/app/tools.py` |
| ChatKit server | `backend/app/book_assistant_server.py` |
| Thread storage | `backend/app/postgres_thread_store.py` |
| Index script | `backend/scripts/index_book.py` |
| Chat widget | `roboai/src/components/ChatWidget/index.tsx` |
| Site config | `roboai/docusaurus.config.ts` |

---

## Success Criteria

- [x] Book is published on GitHub Pages
- [x] Backend serves RAG-powered responses  
- [x] OpenAI Agents SDK agent defined with search tool
- [ ] Chatbot answers questions accurately with citations
- [ ] Context-selection feature works (highlight text  ask about it)
- [ ] Full integration: ChatWidget  FastAPI  Agent  Qdrant
