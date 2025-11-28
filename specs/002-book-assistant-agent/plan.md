# Implementation Plan: Book Assistant AI Agent

**Branch**: `002-book-assistant-agent` | **Date**: 2025-11-28 | **Spec**: [spec.md](./spec.md)
**Status**: ✅ MVP COMPLETE (Phase 1-3 Implemented)
**Input**: Feature specification from `/specs/002-book-assistant-agent/spec.md`

## Summary

Build a RAG-powered AI chatbot for the Physical AI & Humanoid Robotics book using OpenAI ChatKit Server pattern. The chatbot:
- ✅ Answers questions about book content with source citations
- ✅ Supports context-specific queries via text selection
- ✅ Maintains conversation history via ChatKit Store (backed by Neon Postgres)
- ✅ Uses Qdrant Cloud for semantic search with multi-query and context boosting
- ✅ Integrates directly into existing Docusaurus site (`book-source/`) via custom React component
- ✅ Streams responses via SSE using ChatKit protocol

**Architecture Pattern**: OpenAI Agents SDK + ChatKit Server pattern
- `ChatKitServer[dict[str, Any]]` for request handling
- `BookAssistantAgentContext(AgentContext)` for custom context passing
- `Runner.run_streamed()` + `stream_agent_response()` for SSE streaming
- `@function_tool` with `RunContextWrapper[Any]` for context-aware RAG

**Architecture Decision**: ChatWidget integrated into existing Docusaurus site instead of separate frontend.
- Single deployment (GitHub Pages already configured)
- Book content IS the frontend - no duplication
- Text selection works naturally on actual book pages (excludes chat window)
- Better UX - readers stay in context while chatting

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript/React (Docusaurus integration)
**Actual Dependencies** (Installed):
- Backend: `agents>=0.0.10`, `openai-chatkit>=0.0.3`, `openai>=1.82.0`, `fastapi>=0.115.12`, `qdrant-client>=1.14.2`, `asyncpg>=0.30.0`, `pydantic-settings>=2.9.1`, `uvicorn[standard]`
- Frontend: Existing Docusaurus 3.9.2 site with React 18+, CSS Modules

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

**Goal**: Basic Q&A functionality

**Implemented**:
1. ✅ Backend setup (FastAPI, environment, dependencies with `uv`)
2. ✅ Neon Postgres schema migration (`threads`, `thread_items` tables)
3. ✅ Qdrant collection setup and indexing script (Chapter 1)
4. ✅ OpenAI Agent with RAG tool (`search_book_content`)
5. ✅ ChatKit endpoint implementation (`POST /assistant/chatkit`)
6. ✅ ChatWidget integrated in Docusaurus (via swizzled Root.tsx)
7. ✅ Health check (`GET /assistant/health`)

**Deliverables**:
- ✅ Working `/assistant/chatkit` endpoint with SSE streaming
- ✅ Chapter 1 indexed in Qdrant (correct Docusaurus URLs)
- ✅ Conversation history in Postgres via `PostgresThreadStore`
- ✅ ChatWidget in Docusaurus (every page, respects dark/light mode)

### Phase 2: Enhanced Features (P2-P3) ✅ COMPLETE

**Goal**: Text selection and context-aware search

**Implemented**:
1. ✅ Text selection event handler (excludes chat window via ref)
2. ✅ Context mode with `search_scope` parameter:
   - `current_page`: Filter to current lesson only
   - `current_chapter`: Filter to current chapter
   - `full_book`: Search all with context boosting
3. ✅ Page context from URL (`current_chapter`, `current_lesson`)
4. ✅ Source citations with correct Docusaurus URLs
5. ✅ Multi-query RAG (1-3 queries for better recall)
6. ✅ Context boosting (1.2x chapter, 1.1x lesson)

**Deliverables**:
- ✅ Text selection triggers context-aware queries
- ✅ `BookAssistantAgentContext` passes page context to tools
- ✅ `RunContextWrapper[Any]` pattern for context access in tools
- ✅ Source URLs link to correct book sections

### Phase 3: Quality & Polish ✅ PARTIAL

**Goal**: Production readiness

**Implemented**:
1. ✅ `BOOK_BASE_URL` configurable via environment variable
2. ✅ Proper SSE headers for streaming (including `X-Accel-Buffering: no`)
3. ✅ CORS configuration for frontend origins
4. ✅ AsyncQdrantClient for non-blocking I/O
5. ✅ Batch embeddings for multi-query search

**Remaining** (Future):
- [ ] Feedback collection (thumbs up/down)
- [ ] Rate limiting middleware (20 req/min)
- [ ] Input sanitization
- [ ] Comprehensive logging

### Phase 4: Reusable Intelligence ✅ COMPLETE

**Goal**: Capture implementation patterns for reuse

**Created**:
1. ✅ ChatKit Server Implementation Skill (`.claude/skills/chatkit-server-implementation/`)
   - SKILL.md with core patterns
   - references/rag-search-patterns.md
   - references/frontend-integration.md
   - references/troubleshooting.md
2. ✅ ChatKit Server Builder Agent (`.claude/agents/chatkit-server-builder.md`)

## Complexity Tracking

No constitution violations. Design follows standard web application patterns.

## Next Steps

Run `/sp.tasks` to generate actionable tasks from this plan.
