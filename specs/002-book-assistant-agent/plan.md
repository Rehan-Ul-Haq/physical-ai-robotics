# Implementation Plan: Book Assistant AI Agent

**Branch**: `002-book-assistant-agent` | **Date**: 2025-11-28 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-book-assistant-agent/spec.md`

## Summary

Build a RAG-powered AI chatbot for the Physical AI & Humanoid Robotics book using OpenAI ChatKit Server pattern. The chatbot will:
- Answer questions about book content with source citations
- Support context-specific queries via text selection
- Maintain conversation history via ChatKit ThreadStore (backed by Neon Postgres)
- Use Qdrant Cloud for semantic search over book embeddings
- Integrate directly into existing Docusaurus site (`book-source/`) via custom React component

**Architecture Pattern**: Following [openai-chatkit-advanced-samples/customer-support](https://github.com/openai/openai-chatkit-advanced-samples/tree/main/examples/customer-support) example with `ChatKitServer` base class.

**Architecture Decision**: ChatWidget integrated into existing Docusaurus site instead of separate frontend.
- Single deployment (GitHub Pages already configured)
- Book content IS the frontend - no duplication
- Text selection works naturally on actual book pages
- Better UX - readers stay in context while chatting

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript/React (Docusaurus integration)
**Primary Dependencies**:
- Backend: FastAPI >=0.114.1,<0.116, openai >=1.40, openai-chatkit >=1.1.2,<2, httpx >=0.28,<0.29, uvicorn[standard] >=0.36,<0.37, qdrant-client, asyncpg, pydantic
- Frontend: Existing Docusaurus site with React 18+, CSS Modules

**Storage**:
- Qdrant Cloud (vector embeddings for RAG)
- Neon Postgres (ThreadStore persistence, feedback events)

**Testing**: pytest (backend)
**Target Platform**: Web (Docusaurus embedded, all modern browsers)
**Project Type**: Backend API + Docusaurus component integration

**Performance Goals**:
- Response time < 3 seconds average
- Support 100 concurrent users
- Rate limit: 20 requests/minute per session

**Constraints**:
- Must use OpenAI ChatKit Server pattern (`openai-chatkit` package)
- Must extend `ChatKitServer[TContext]` base class for backend
- Must implement `ThreadStore` interface for conversation persistence
- Must use Qdrant Cloud free tier for vector storage
- Must use Neon Postgres for ThreadStore backend
- Must integrate into existing Docusaurus site (no separate frontend)
- ChatWidget must respect Docusaurus dark/light mode via CSS variables

**Scale/Scope**:
- Initial: Chapter 1 content (~10-50 chunks)
- Target: Full book (~200-500 chunks)
- Users: Educational context, moderate traffic

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Specification Primacy | PASS | Spec defined before implementation planning |
| Progressive Complexity | PASS | P1-P4 user stories provide incremental delivery |
| Factual Accuracy | PASS | RAG retrieves verified book content |
| Intelligence Accumulation | PASS | Conversation history persisted, feedback collected |
| Anti-Convergence | N/A | Infrastructure feature, not educational content |
| Minimal Content | PASS | Only essential components designed |

**Post-Phase 1 Re-Check**: PASS - Design artifacts align with spec requirements

## Project Structure

### Documentation (this feature)

```text
specs/002-book-assistant-agent/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Phase 0: Technology research
├── data-model.md        # Phase 1: Entity schemas
├── quickstart.md        # Phase 1: Development setup guide
├── contracts/           # Phase 1: API contracts
│   └── openapi.yaml     # OpenAPI 3.1 specification
└── tasks.md             # Phase 2: Implementation tasks (via /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── app/
│   ├── __init__.py
│   ├── main.py                  # FastAPI app + ChatKitServer integration
│   ├── book_assistant_server.py # ChatKitServer[BookAssistantContext] implementation
│   ├── book_assistant_agent.py  # Agent definition with RAG tools
│   ├── postgres_thread_store.py # ThreadStore implementation for Neon Postgres
│   ├── qdrant_service.py        # Vector search service
│   ├── thread_item_converter.py # Convert thread items for agent
│   ├── models.py                # Pydantic models
│   ├── tools.py                 # RAG search tool
│   └── config.py                # Environment configuration
├── migrations/
│   └── 001_initial_schema.sql
├── scripts/
│   ├── setup_qdrant.py
│   └── index_book.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── conftest.py
├── pyproject.toml
├── uv.lock
└── .env.example

book-source/src/                  # Docusaurus integration (NOT separate frontend)
├── components/
│   └── ChatWidget/
│       ├── index.tsx            # Chat widget with SSE streaming
│       └── styles.module.css    # CSS module (dark/light mode)
└── theme/
    └── Root.tsx                 # ChatWidget integrated here
```

**Structure Decision**: Following ChatKit advanced samples pattern with `ChatKitServer` base class. Backend uses flat `app/` structure (matching customer-support example). Key files:
- `book_assistant_server.py`: Extends `ChatKitServer[BookAssistantContext]` with `respond()` method
- `postgres_thread_store.py`: Implements ChatKit `Store` protocol for Neon Postgres persistence
- `book_assistant_agent.py`: Defines `Agent[AgentContext]` with RAG search tool

**Frontend Decision**: ChatWidget integrated directly into existing Docusaurus site rather than separate React app.
- Eliminates separate build/deployment pipeline
- Uses CSS modules with Docusaurus CSS variables for theme compatibility
- Appears on every page via Root.tsx wrapper
- SSE streaming built into component (no external ChatKit web component needed)

## Design Artifacts

### Phase 0: Research Complete

See [research.md](./research.md) for:
- OpenAI Agents SDK implementation patterns
- ChatKit server architecture
- RAG pipeline with Qdrant
- Neon Postgres chat history storage
- Text selection implementation
- Security considerations

### Phase 1: Design Complete

**Data Model**: See [data-model.md](./data-model.md)
- BookChunk (Qdrant): Vector embeddings with source metadata
- ThreadItem (ChatKit): Standard chat items stored via ThreadStore
- Thread (Postgres): ChatKit thread with metadata (context_mode, selected_text)
- FeedbackEvent (Postgres): User satisfaction signals

**API Contracts**: See [contracts/openapi.yaml](./contracts/openapi.yaml)
- `POST /assistant/chatkit` - Main ChatKit server endpoint (via `server.process()`)
- `GET /assistant/thread/{thread_id}` - Get thread state and context
- `POST /assistant/feedback` - Submit feedback
- `POST /assistant/reindex` - Trigger content reindexing (admin)
- `GET /assistant/health` - Health check

**Quickstart Guide**: See [quickstart.md](./quickstart.md)
- Prerequisites and account setup
- Environment configuration
- Database and Qdrant setup
- Development workflow

## Implementation Phases

### Phase 1: Core Infrastructure (P1 MVP) ✅ COMPLETE

**Goal**: Basic Q&A functionality without text selection

1. Backend setup (FastAPI, environment, dependencies)
2. Neon Postgres schema migration
3. Qdrant collection setup and indexing script
4. OpenAI Agent with RAG tool
5. ChatKit endpoint implementation
6. ChatWidget integrated in Docusaurus
7. Health check and error handling

**Deliverables**:
- Working `/chatkit` endpoint
- Chapter 1 indexed in Qdrant
- Conversation history in Postgres
- ChatWidget in Docusaurus (every page)

### Phase 2: Enhanced Features (P2-P3)

**Goal**: Text selection and conversation context

1. Text selection event handler (ChatWidget)
2. Context mode switching (selected_text vs full_book)
3. Conversation history loading
4. Source citations in responses
5. "New conversation" functionality

**Deliverables**:
- Text selection triggers context-specific queries
- Context-aware responses
- Clickable source citations

### Phase 3: Quality & Feedback (P4)

**Goal**: Production readiness

1. Feedback collection (thumbs up/down)
2. Rate limiting middleware
3. Input sanitization
4. Comprehensive error handling
5. Logging and monitoring

**Deliverables**:
- Feedback tracking for eval metrics
- Rate limiting (20 req/min)
- Security hardening
- Production deployment ready

## Complexity Tracking

No constitution violations. Design follows standard web application patterns.

## Next Steps

Run `/sp.tasks` to generate actionable tasks from this plan.
