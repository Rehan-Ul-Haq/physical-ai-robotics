# Tasks: Book Assistant AI Agent

**Input**: Design documents from `/specs/002-book-assistant-agent/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md
**Status**: ✅ MVP COMPLETE (Phases 1-4)

**Tests**: Not explicitly requested - tests are optional
**Package Manager**: uv (per user specification)
**Architecture Pattern**: OpenAI Agents SDK + ChatKit Server (`ChatKitServer[TContext]` base class)

## Architecture Decision: Docusaurus Integration

**Decision**: Integrate ChatWidget into existing Docusaurus site (`book-source/`) instead of creating separate frontend.

**Rationale**:
- Single deployment (GitHub Pages already configured)
- Book content IS the frontend - no duplication
- Text selection works naturally on actual book pages
- Better UX - readers stay in context while chatting

**Impact on Tasks**:
- Frontend tasks (T024-T028) replaced with Docusaurus integration
- `frontend/` directory removed
- ChatWidget added to `book-source/src/components/`

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/app/` (flat structure per ChatKit pattern)
- **Frontend**: `book-source/src/` (Docusaurus integration)
- **Scripts**: `backend/scripts/`
- **Migrations**: `backend/migrations/`

---

## Phase 1: Setup (Shared Infrastructure) ✅ COMPLETE

**Purpose**: Project initialization with uv package manager and ChatKit dependencies

### Backend Setup

- [x] T001 Create backend directory structure: `backend/app/`, `backend/migrations/`, `backend/scripts/`, `backend/tests/`
- [x] T002 Initialize Python project with uv: `cd backend && uv init --name book-assistant-backend`
- [x] T003 [P] Add backend dependencies with uv in `backend/pyproject.toml`: `fastapi>=0.114.1,<0.116`, `openai>=1.40`, `openai-chatkit>=1.1.2,<2`, `httpx>=0.28,<0.29`, `uvicorn[standard]>=0.36,<0.37`, `qdrant-client`, `asyncpg`, `pydantic`, `python-dotenv`
- [x] T004 [P] Create `backend/.env.example` with OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, HOST, PORT

### Frontend Setup (Docusaurus Integration - NO separate frontend)

- [x] ~~T005 Create frontend directory structure~~ (REMOVED - using existing Docusaurus)
- [x] ~~T006 Initialize frontend with Vite React TypeScript~~ (REMOVED - using existing Docusaurus)
- [x] ~~T007 Add frontend dependencies~~ (REMOVED - Docusaurus already has React)
- [x] ~~T008 Configure TailwindCSS~~ (REMOVED - using CSS Modules with Docusaurus variables)
- [x] T009 [P] Add `BOOK_ASSISTANT_API_URL` to `book-source/.env.example`

---

## Phase 2: Foundational (Blocking Prerequisites) ✅ COMPLETE

**Purpose**: Core infrastructure that MUST be complete before ANY user story

### Database Setup

- [x] T010 Create Neon Postgres schema migration in `backend/migrations/001_initial_schema.sql` with `threads`, `thread_items`, `feedback_events` tables per data-model.md
- [x] T011 Create database connection utility in `backend/app/database.py` using asyncpg pool

### Qdrant Setup

- [x] T012 Create Qdrant setup script in `backend/scripts/setup_qdrant.py` to create `book_content` collection with 1536 dimensions
- [x] T013 Create book indexing script in `backend/scripts/index_book.py` with markdown chunking, embedding generation, and Qdrant upsert

### ChatKit Core Components

- [x] T014 Create configuration module in `backend/app/config.py` using pydantic Settings for environment variables
- [x] T015 Create Pydantic models in `backend/app/models.py`: `BookChunkPayload`, `Source`, `BookAssistantContext`, `ThreadItem`, `Thread`, `FeedbackCreate`, `FeedbackResponse`
- [x] T016 Implement `PostgresThreadStore` in `backend/app/postgres_thread_store.py` extending ChatKit `Store` protocol with methods: `generate_thread_id`, `generate_item_id`, `load_thread`, `save_thread`, `load_thread_items`, `add_thread_item`, `delete_thread`
- [x] T017 Create Qdrant service in `backend/app/qdrant_service.py` with `search()` method for vector similarity search with optional text filter

### Agent Definition

- [x] T018 Create RAG search tool in `backend/app/tools.py` using `@function_tool` decorator with query parameter, accessing Qdrant service
- [x] T019 Create Book Assistant Agent in `backend/app/book_assistant_agent.py` as `Agent[AgentContext]` with RAG tool and instructions for citing sources

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Ask Questions About Book Content (Priority: P1) ✅ MVP COMPLETE

**Goal**: Readers can ask questions about book content and receive accurate answers with source citations

**Independent Test**: Ask "What is Physical AI?" and verify response contains relevant book content with citations

### Backend Implementation

- [x] T020 [US1] Create `BookAssistantServer` in `backend/app/book_assistant_server.py` extending `ChatKitServer[BookAssistantContext]` with `respond()` method that runs agent and streams response
- [x] T021 [US1] Create thread item converter in `backend/app/thread_item_converter.py` for converting ChatKit items to agent input format
- [x] T022 [US1] Implement main FastAPI app in `backend/app/main.py` with:
  - `POST /assistant/chatkit` endpoint using `server.process()` returning `StreamingResponse`
  - `GET /assistant/health` endpoint
  - CORS middleware for frontend
- [x] T023 [US1] Add graceful error handling in `backend/app/book_assistant_server.py` for cases where no relevant content is found (FR-007)

### Frontend Implementation (Docusaurus Integration)

- [x] T024 [P] [US1] Create ChatWidget component in `book-source/src/components/ChatWidget/index.tsx` with:
  - Built-in API client with SSE handling for ChatKit streaming
  - Session ID management (localStorage)
  - Chat input and message display
  - Loading states and error handling
- [x] T025 [P] [US1] Create ChatWidget styles in `book-source/src/components/ChatWidget/styles.module.css`
- [x] T026 [US1] Integrate ChatWidget in `book-source/src/theme/Root.tsx` (appears on every page)
- [x] T027 [US1] Add BOOK_ASSISTANT_API_URL to `book-source/.env.example`
- [x] T028 [US1] ~~Style chat interface~~ (included in ChatWidget CSS module)

### Content Indexing

- [x] T029 [US1] Index Chapter 1 content into Qdrant using `backend/scripts/index_book.py` targeting `book-source/docs/01-robotic-nervous-system/01-introduction-to-physical-ai/`

**Checkpoint**: User Story 1 complete - users can ask questions and receive RAG-powered answers

---

## Phase 4: User Story 2 - Context-Specific Text Selection Queries (Priority: P2) ✅ COMPLETE

**Goal**: Readers can select text and ask questions specifically about that selection

**Independent Test**: Select paragraph on page, ask question, verify answer is constrained to selected context

### Backend Implementation

- [x] T030 [US2] Extend `BookAssistantContext` in `backend/app/models.py` with `PageContext` model for `current_chapter`, `current_lesson`
- [x] T031 [US2] Update `respond()` in `backend/app/book_assistant_server.py` to pass page context to `BookAssistantAgentContext`
- [x] T032 [US2] Update RAG tool in `backend/app/tools.py` with:
  - `search_scope` parameter: `current_page`, `current_chapter`, `full_book`
  - Context access via `RunContextWrapper[Any]` with `ctx.context.page_context`
  - Multi-query search (1-3 queries) with batch embeddings
  - Context boosting (1.2x chapter, 1.1x lesson)

### Frontend Implementation (Docusaurus)

- [x] T033 [P] [US2] Add text selection handling to ChatWidget in `book-source/src/components/ChatWidget/index.tsx`:
  - `mouseup` event listener for text selection on book pages
  - `chatWindowRef` to exclude chat window from selection capture
  - Selection text limit (2000 chars) with truncation
- [x] T034 [US2] Update ChatWidget to support context modes:
  - Send `page_context` with `current_chapter` and `current_lesson` in request
  - Extract context from URL path
  - Show selected text context when present
- [x] T035 [US2] Add clear selection button to ChatWidget

**Checkpoint**: User Story 2 complete ✅

---

## Phase 5: User Story 3 - Conversation History and Follow-up Questions (Priority: P3) ✅ COMPLETE

**Goal**: Chatbot maintains conversation context for natural multi-turn dialogue

**Independent Test**: Ask question, then follow up with "Can you explain that further?" and verify context is maintained

### Backend Implementation

- [x] T036 [US3] Update `respond()` in `backend/app/book_assistant_server.py` to load thread history via `to_agent_input()` function
- [x] T037 [US3] Implement `GET /assistant/thread/{thread_id}` endpoint in `backend/app/main.py` to retrieve thread with all items
- [x] T038 [US3] Implement `DELETE /assistant/thread/{thread_id}` endpoint in `backend/app/main.py` for "New conversation"

### Frontend Implementation (Docusaurus)

- [x] T039 [P] [US3] Thread management in ChatWidget via ChatKit SSE events:
  - `thread.created` event provides `thread_id`
  - Thread ID stored in component state
- [x] T040 [US3] Thread persistence to ChatWidget:
  - Store `thread_id` in component state with localStorage backup
  - Send `thread_id` with requests for conversation continuity
  - ChatKit protocol handles history via `threads.add_user_message`
- [x] T041 [US3] Add "New conversation" button to ChatWidget

**Checkpoint**: User Story 3 complete ✅

---

## Phase 6: User Story 4 - Source Citation and Navigation (Priority: P4) ✅ COMPLETE

**Goal**: Answers include clickable citations linking to book sections

**Independent Test**: Ask question, verify response includes citations with links that navigate to correct book sections

### Backend Implementation

- [x] T042 [US4] Update agent instructions in `backend/app/book_assistant_agent.py` to always cite sources in format: [Section Title](source_url_from_search)
- [x] T043 [US4] RAG tool in `backend/app/tools.py` returns `source_url` in formatted_sources:
  - `source_url` built from: `/docs/{part_slug}/{chapter_slug}/{lesson_slug}#{section_id}`
  - Indexed with correct Docusaurus URLs via `backend/scripts/index_book.py`
- [x] T044 [US4] Source metadata stored in Qdrant payload with `source_url` field

### Frontend Implementation (Docusaurus)

- [x] T045 [P] [US4] Markdown rendering in ChatWidget with `react-markdown`:
  - Citations rendered as clickable links
  - Links navigate to book sections
- [x] T046 [US4] Message rendering parses markdown citations automatically
- [ ] T047 [US4] (FUTURE) Hover preview for citations

**Checkpoint**: User Story 4 complete ✅

---

## Phase 7: Polish & Cross-Cutting Concerns ⏳ PARTIAL

**Purpose**: Production readiness, security, and quality improvements
**Status**: Infrastructure complete, advanced features pending

### Feedback Collection (FR-015) - FUTURE

- [ ] T048 [P] Implement `POST /assistant/feedback` endpoint in `backend/app/main.py` for thumbs up/down
- [ ] T049 [P] Create feedback service in `backend/app/feedback_service.py` with `create_feedback()` storing to `feedback_events` table
- [ ] T050 Add feedback buttons (thumbs up/down) to assistant messages in `book-source/src/components/ChatWidget/index.tsx`

### Rate Limiting (FR-009) - FUTURE

- [ ] T051 Create rate limiting middleware in `backend/app/middleware.py` (20 requests/minute per session)
- [ ] T052 Add rate limit middleware to FastAPI app in `backend/app/main.py`

### Security (FR-011, FR-012) - PARTIAL

- [ ] T053 Create input sanitization utility in `backend/app/sanitize.py` for prompt injection prevention
- [ ] T054 Apply input sanitization to user queries in `backend/app/book_assistant_server.py` before agent processing
- [x] T055 `.env.example` files created with all required variables (no secrets committed)

### Error Handling & Logging - PARTIAL

- [ ] T056 Add structured logging to all backend modules using Python logging
- [x] T057 ChatWidget has error handling (try/catch on fetch, error state display)

### Admin Tools - PARTIAL

- [ ] T058 [P] Implement `POST /assistant/reindex` endpoint in `backend/app/main.py` (admin-only) to trigger content reindexing
- [x] T059 [P] Implemented `GET /assistant/index/status` endpoint in `backend/app/main.py` returning Qdrant collection stats

### Documentation & Validation

- [ ] T060 Update `specs/002-book-assistant-agent/quickstart.md` with actual commands for running backend + Docusaurus dev
- [ ] T061 Run quickstart.md validation: follow guide end-to-end, verify all steps work

---

## Phase 8: Reusable Intelligence ✅ COMPLETE

**Purpose**: Capture implementation patterns for future ChatKit server projects
**Status**: Skill and subagent created based on implementation experience

### ChatKit Server Implementation Skill

- [x] T062 Create ChatKit Server Implementation Skill in `.claude/skills/chatkit-server-implementation/SKILL.md`
- [x] T063 Create RAG Search Patterns reference in `.claude/skills/chatkit-server-implementation/references/rag-search-patterns.md`
- [x] T064 Create Frontend Integration reference in `.claude/skills/chatkit-server-implementation/references/frontend-integration.md`
- [x] T065 Create Troubleshooting reference in `.claude/skills/chatkit-server-implementation/references/troubleshooting.md`

### ChatKit Server Builder Agent

- [x] T066 Create ChatKit Server Builder Agent in `.claude/agents/chatkit-server-builder.md`

**Deliverables**:
- ✅ `chatkit-server-implementation` skill with Persona + Questions + Principles framework
- ✅ Reference documentation for RAG patterns, frontend integration, troubleshooting
- ✅ `chatkit-server-builder` subagent for future ChatKit implementations

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - MVP delivery
- **User Story 2 (Phase 4)**: Depends on US1 completion (extends existing components)
- **User Story 3 (Phase 5)**: Depends on US1 completion (extends existing components)
- **User Story 4 (Phase 6)**: Depends on US1 completion (extends existing components)
- **Polish (Phase 7)**: Can start after US1, recommended after all stories complete

### User Story Dependencies

- **User Story 1 (P1)**: Foundation only - this IS the MVP
- **User Story 2 (P2)**: Can run in parallel with US3/US4 after US1, extends ChatWidget
- **User Story 3 (P3)**: Can run in parallel with US2/US4 after US1, extends ChatWidget
- **User Story 4 (P4)**: Can run in parallel with US2/US3 after US1, extends ChatWidget

### Within Each Phase

- Backend before frontend (API must exist for frontend to call)
- Models before services
- Services before endpoints
- Core implementation before enhancements

### Parallel Opportunities

**Phase 1 (Setup)**:
```bash
# These can run in parallel:
T003: Add backend dependencies
T004: Create backend .env.example
T007: Add frontend dependencies
T008: Configure TailwindCSS
T009: Create frontend .env.example
```

**Phase 2 (Foundational)**:
```bash
# These can run in parallel after T011:
T012: Qdrant setup script
T014: Config module
T015: Pydantic models
```

**Phase 3 (US1)**:
```bash
# These can run in parallel:
T024: API client
T025: Session hook
```

**Phase 7 (Polish)**:
```bash
# These can run in parallel:
T048: Feedback endpoint
T049: Feedback service
T058: Reindex endpoint
T059: Index status endpoint
```

---

## Parallel Example: User Story 1 MVP

```bash
# After Foundational phase completes, launch US1 in order:

# Backend first (sequential):
T020: BookAssistantServer
T021: Thread item converter
T022: FastAPI main.py
T023: Error handling

# Frontend (parallel after T022):
T024 + T025: API client and session hook (parallel)
T026: ChatWidget (after T024, T025)
T027: App integration
T028: Styling

# Content:
T029: Index Chapter 1
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T009)
2. Complete Phase 2: Foundational (T010-T019)
3. Complete Phase 3: User Story 1 (T020-T029)
4. **STOP and VALIDATE**: Test asking questions about Chapter 1 content
5. Deploy/demo if ready - this is a functional chatbot!

### Incremental Delivery

1. **MVP**: Setup + Foundational + US1 → Deploy (basic Q&A works)
2. **+US2**: Text selection → Deploy (enhanced context)
3. **+US3**: Conversation history → Deploy (multi-turn)
4. **+US4**: Source citations → Deploy (trust building)
5. **+Polish**: Feedback, rate limiting, security → Production ready

### Suggested MVP Scope

**Just User Story 1** provides:
- Working chatbot with RAG
- Chapter 1 indexed
- Basic chat UI
- Source citations in responses (basic)

This can be demoed and tested immediately after Phase 3 completion.

---

## Commands Reference

```bash
# Backend setup
cd backend
uv init --name book-assistant-backend
uv add "fastapi>=0.114.1,<0.116" "openai>=1.40" "openai-chatkit>=1.1.2,<2" "httpx>=0.28,<0.29" "uvicorn[standard]>=0.36,<0.37" qdrant-client asyncpg pydantic python-dotenv

# Run backend
uv run uvicorn app.main:app --host 127.0.0.1 --port 8001 --reload

# Run scripts
uv run python scripts/setup_qdrant.py
uv run python scripts/index_book.py

# Docusaurus dev (from book-source/)
cd book-source
npm install
npm start
```

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- Each user story is independently testable after completion
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- ChatKit pattern: `ChatKitServer` → `respond()` → `Runner.run_streamed()` → `stream_agent_response()`
