# Physical AI Robotics Project — Constitution

**Version:** 8.0.0
**Ratified:** 2025-12-02
**Last Amended:** 2025-12-02
**Scope:** Complete project governance (book platform + RAG chatbot + deployment)

---

## Project Overview

This is a **two-part project**:

1. **Book Platform** — Docusaurus site about Physical AI & Humanoid Robotics
2. **Book Assistant** — RAG-powered chatbot using OpenAI Agents SDK

This constitution governs decision-making across ALL project components.

---

## I. Architecture & Technology Stack

### Project Structure

```
physical-ai-robotics/
├── roboai/                    # Part 1: Docusaurus Book Platform
│   ├── docs/                  # Book content (MDX)
│   ├── src/components/        # React components (ChatWidget)
│   └── docusaurus.config.ts   # Site configuration
│
├── backend/                   # Part 2: FastAPI + OpenAI Agents SDK
│   ├── app/
│   │   ├── main.py                    # FastAPI application
│   │   ├── book_assistant_agent.py    # OpenAI Agents SDK agent
│   │   ├── book_assistant_server.py   # ChatKit server
│   │   ├── qdrant_service.py          # Vector search (RAG)
│   │   └── tools.py                   # Agent tools
│   └── scripts/
│       └── index_book.py              # Index content to Qdrant
│
└── .github/workflows/
    ├── deploy.yml             # Deploy to GitHub Pages
    └── validate-content.yml   # Validate assets
```

### Technology Stack

| Component | Technology | Purpose |
|-----------|------------|---------|
| **Book Platform** | Docusaurus 3.x | Static site generator |
| **Styling** | Tailwind CSS | UI styling |
| **API Server** | FastAPI | HTTP endpoints, CORS |
| **AI Agent** | OpenAI Agents SDK | Agent definition, tools, conversation |
| **Chat Protocol** | OpenAI ChatKit SDK | Streaming, thread management |
| **Vector DB** | Qdrant Cloud | Semantic search for RAG |
| **Embeddings** | text-embedding-3-small | Content vectorization |
| **Database** | PostgreSQL | Thread/conversation storage |
| **Hosting** | GitHub Pages | Book platform hosting |

### Key Technology Distinctions

Understanding component responsibilities prevents confusion:

- **FastAPI** = HTTP server, routes, middleware, CORS handling
- **OpenAI Agents SDK** = Agent definition, tool registration, instructions, conversation orchestration
- **ChatKit SDK** = Streaming protocol, SSE responses, thread state management
- **Qdrant** = Vector storage, similarity search, RAG retrieval

---

## II. Decision Frameworks by Project Area

### A. Book Content (roboai/docs/)

**When creating or modifying book content:**

1. **Topic**: Physical AI & Humanoid Robotics (ROS 2, simulation, Sim-to-Real)
2. **Format**: MDX files in `roboai/docs/`
3. **Structure**: 4 modules, 13 weeks progression
4. **Validation**: `npm run build` must pass

**Content Principles:**
- Explain concepts before showing code
- Include practical examples with ROS 2
- Address hardware requirements where relevant
- Connect theory to robot behavior

### B. Backend Development (backend/)

**When modifying the Book Assistant:**

| Change Type | File to Edit | Considerations |
|-------------|--------------|----------------|
| Agent behavior | `book_assistant_agent.py` | Instructions, tool definitions |
| API endpoints | `main.py` | Routes, request/response models |
| RAG logic | `qdrant_service.py`, `tools.py` | Search queries, chunking |
| Data models | `models.py` | Pydantic schemas |
| Thread storage | `postgres_thread_store.py` | Persistence logic |

**Backend Principles:**
- Test locally before committing (`uv run uvicorn app.main:app --reload`)
- Keep agent instructions clear and focused
- Use proper error handling in API endpoints
- Validate against `/assistant/health` endpoint

### C. Frontend Integration (roboai/src/)

**When modifying the chat interface:**

| Change Type | File to Edit |
|-------------|--------------|
| Chat widget | `src/components/ChatWidget/` |
| Site config | `docusaurus.config.ts` |
| Styling | `src/css/` or Tailwind classes |

**Frontend Principles:**
- Build must pass: `npm run build`
- Test ChatWidget → Backend integration
- Handle loading/error states gracefully

---

## III. RAG Architecture Decisions

### How the Book Assistant Works

```
User Question
    │
    ▼
FastAPI (/assistant/chatkit)
    │
    ▼
ChatKit Server (transforms request to ChatKit protocol)
    │
    ▼
OpenAI Agent (book_assistant_agent.py)
    │
    ├─► Tool: search_book_content
    │      │
    │      ▼
    │   Qdrant: semantic search → relevant chunks
    │
    ▼
LLM generates response with citations
    │
    ▼
Streaming SSE response to ChatWidget
```

### RAG Decision Framework

**When modifying search behavior:**

1. **Query formation**: How does the agent formulate search queries?
   - Edit: Agent instructions in `book_assistant_agent.py`
   
2. **Chunk retrieval**: How many chunks? What similarity threshold?
   - Edit: `qdrant_service.py` search parameters
   
3. **Context injection**: How are chunks presented to the LLM?
   - Edit: Tool return format in `tools.py`

4. **Citation accuracy**: How are sources referenced?
   - Edit: Agent instructions for citation format

### Index Maintenance

**When book content changes:**
```powershell
cd backend
uv run python scripts/index_book.py --force
```

This re-indexes all content in `roboai/docs/` to Qdrant.

---

## IV. Book Content Structure

### Module Overview (13 Weeks)

| Module | Focus | Weeks | Content Path |
|--------|-------|-------|--------------|
| **1: Robotic Nervous System** | ROS 2 fundamentals | 1-5 | `01-robotic-nervous-system/` |
| **2: Digital Twin** | Gazebo & Unity | 6-7 | `02-digital-twin/` |
| **3: AI-Robot Brain** | NVIDIA Isaac | 8-10 | `03-ai-robot-brain/` |
| **4: Vision-Language-Action** | LLM + Robotics | 11-13 | `04-vision-language-action/` |

### Content Quality Principles

1. **Explain before demonstrating**: Concepts before code
2. **Production-relevant examples**: Not toy tutorials
3. **Progressive complexity**: A2 → B1 → B2 → C1
4. **Hardware awareness**: State requirements for computational content

---

## V. Deployment & Operations

### Frontend Deployment (GitHub Pages)

- **Trigger**: Push to `main` with changes in `roboai/**`
- **Workflow**: `.github/workflows/deploy.yml`
- **URL**: `https://rehan-ul-haq.github.io/physical-ai-robotics/`

### Backend Deployment

- **Host options**: Railway, Render, Fly.io, Azure Container Apps
- **Requirements**: PostgreSQL database, environment variables
- **Health check**: `GET /assistant/health`

### Environment Variables (Backend)

```env
OPENAI_API_KEY=sk-...
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=...
QDRANT_COLLECTION_NAME=book_chunks
DATABASE_URL=postgresql://user:pass@host:5432/db
```

---

## VI. Development Workflow

### Default Behavior

1. **Default to action**: Implement changes rather than suggesting
2. **Read before editing**: Understand context before modifying
3. **Test before committing**: Verify changes work locally
4. **Parallel operations**: Execute independent tasks simultaneously

### Change Verification

| Component | Verification Command |
|-----------|---------------------|
| Book build | `cd roboai && npm run build` |
| Backend health | `curl http://localhost:8000/assistant/health` |
| Full integration | Send test query to `/assistant/chatkit` |

### Git Workflow

```powershell
# Check status
git status

# Create feature branch for significant changes
git checkout -b feature/description

# Commit with descriptive message
git add -A
git commit -m "type: description"

# Push and create PR
git push -u origin feature/description
```

---

## VII. Success Criteria

### Project Completion Checklist

- [x] Book is published on GitHub Pages
- [x] Backend serves RAG-powered responses
- [x] OpenAI Agents SDK agent defined with search tool
- [ ] Chatbot answers questions accurately with citations
- [ ] Context-selection feature works (highlight → ask)
- [ ] Full integration: ChatWidget ↔ FastAPI ↔ Agent ↔ Qdrant

### Quality Metrics

- **Book**: Builds without errors, content is accurate
- **Backend**: Health check passes, responses stream correctly
- **Integration**: End-to-end chat flow works
- **Citations**: Sources are accurate and clickable

---

## VIII. Quick Reference

### Common Commands

```powershell
# Frontend
cd roboai && npm run start      # Dev server
cd roboai && npm run build      # Production build

# Backend  
cd backend && uv run uvicorn app.main:app --reload  # Dev server
cd backend && uv run python scripts/index_book.py   # Re-index content

# Health check
curl http://localhost:8000/assistant/health
```

### Key Files

| Purpose | Location |
|---------|----------|
| Agent definition | `backend/app/book_assistant_agent.py` |
| FastAPI routes | `backend/app/main.py` |
| Vector search | `backend/app/qdrant_service.py` |
| Chat widget | `roboai/src/components/ChatWidget/` |
| Book content | `roboai/docs/` |
| Deploy workflow | `.github/workflows/deploy.yml` |

---

**This constitution governs the complete Physical AI Robotics project. For book content, apply pedagogical principles. For backend/frontend, apply software engineering principles. All components work together to deliver a RAG-powered book assistant.**
