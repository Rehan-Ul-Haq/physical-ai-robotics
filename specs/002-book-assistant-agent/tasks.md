# Tasks: Book Assistant AI Agent

**Input**: Design documents from `/specs/002-book-assistant-agent/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/openapi.yaml

**Tests**: Not explicitly requested - tests are optional
**Package Manager**: uv (per user specification)
**Architecture Pattern**: OpenAI ChatKit Server (`ChatKitServer[TContext]` base class)

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

## Phase 4: User Story 2 - Context-Specific Text Selection Queries (Priority: P2)

**Goal**: Readers can select text and ask questions specifically about that selection

**Independent Test**: Select paragraph on page, ask question, verify answer is constrained to selected context

### Backend Implementation

- [ ] T030 [US2] Extend `BookAssistantContext` in `backend/app/models.py` to handle `context_mode` enum: "full_book" | "selected_text"
- [ ] T031 [US2] Update `respond()` in `backend/app/book_assistant_server.py` to pass `selected_text` to agent context when `context_mode` is "selected_text"
- [ ] T032 [US2] Update RAG tool in `backend/app/tools.py` to filter/prioritize results based on selected text context

### Frontend Implementation (Docusaurus)

- [ ] T033 [P] [US2] Add text selection handling to ChatWidget in `book-source/src/components/ChatWidget/index.tsx`:
  - `mouseup` event listener for text selection on book pages
  - "Ask about this" button that appears on selection
  - Selection text limit (2000 chars) with truncation notice
- [ ] T034 [US2] Update ChatWidget to support context modes:
  - Send `context_mode` and `selected_text` in request
  - Show context indicator when in selected_text mode
- [ ] T035 [US2] Add "Clear selection" button to ChatWidget for switching back to full_book mode

**Checkpoint**: User Story 2 complete - text selection enables context-specific queries

---

## Phase 5: User Story 3 - Conversation History and Follow-up Questions (Priority: P3)

**Goal**: Chatbot maintains conversation context for natural multi-turn dialogue

**Independent Test**: Ask question, then follow up with "Can you explain that further?" and verify context is maintained

### Backend Implementation

- [ ] T036 [US3] Update `respond()` in `backend/app/book_assistant_server.py` to load thread history via `PostgresThreadStore.load_thread_items()` and include in agent input
- [ ] T037 [US3] Implement `GET /assistant/thread/{thread_id}` endpoint in `backend/app/main.py` to retrieve thread with all items
- [ ] T038 [US3] Implement `DELETE /assistant/thread/{thread_id}` endpoint in `backend/app/main.py` for "New conversation" (calls `store.delete_thread()`)

### Frontend Implementation (Docusaurus)

- [ ] T039 [P] [US3] Add API methods to ChatWidget in `book-source/src/components/ChatWidget/index.tsx`: `getThread()` and `deleteThread()`
- [ ] T040 [US3] Add thread persistence to ChatWidget:
  - Store `thread_id` in component state (with localStorage backup)
  - Send `thread_id` with requests for conversation continuity
  - Load thread history on mount if thread_id exists
- [ ] T041 [US3] Add "New conversation" button to ChatWidget that calls delete endpoint and clears local state

**Checkpoint**: User Story 3 complete - multi-turn conversations work with context

---

## Phase 6: User Story 4 - Source Citation and Navigation (Priority: P4)

**Goal**: Answers include clickable citations linking to book sections

**Independent Test**: Ask question, verify response includes citations with links that navigate to correct book sections

### Backend Implementation

- [ ] T042 [US4] Update agent instructions in `backend/app/book_assistant_agent.py` to always cite sources in format: [Chapter X, Section Y](url)
- [ ] T043 [US4] Ensure RAG tool in `backend/app/tools.py` returns `source_url` metadata with search results
- [ ] T044 [US4] Update `ThreadItem` storage to include `sources` JSONB field in `backend/app/postgres_thread_store.py`

### Frontend Implementation (Docusaurus)

- [ ] T045 [P] [US4] Add SourceCitation rendering to ChatWidget in `book-source/src/components/ChatWidget/index.tsx` for clickable links
- [ ] T046 [US4] Update message rendering in ChatWidget to parse markdown citations and render as links
- [ ] T047 [US4] Add hover preview for citations showing chapter/section before click

**Checkpoint**: User Story 4 complete - citations link to book sections

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Production readiness, security, and quality improvements

### Feedback Collection (FR-015)

- [ ] T048 [P] Implement `POST /assistant/feedback` endpoint in `backend/app/main.py` for thumbs up/down
- [ ] T049 [P] Create feedback service in `backend/app/feedback_service.py` with `create_feedback()` storing to `feedback_events` table
- [ ] T050 Add feedback buttons (thumbs up/down) to assistant messages in `book-source/src/components/ChatWidget/index.tsx`

### Rate Limiting (FR-009)

- [ ] T051 Create rate limiting middleware in `backend/app/middleware.py` (20 requests/minute per session)
- [ ] T052 Add rate limit middleware to FastAPI app in `backend/app/main.py`

### Security (FR-011, FR-012)

- [ ] T053 Create input sanitization utility in `backend/app/sanitize.py` for prompt injection prevention
- [ ] T054 Apply input sanitization to user queries in `backend/app/book_assistant_server.py` before agent processing
- [ ] T055 Audit `.env.example` files to ensure no secrets are committed

### Error Handling & Logging

- [ ] T056 Add structured logging to all backend modules using Python logging
- [ ] T057 Add error boundary to ChatWidget in `book-source/src/components/ChatWidget/index.tsx` for graceful error handling

### Admin Tools

- [ ] T058 [P] Implement `POST /assistant/reindex` endpoint in `backend/app/main.py` (admin-only) to trigger content reindexing
- [ ] T059 [P] Implement `GET /assistant/index/status` endpoint to return Qdrant collection stats

### Documentation & Validation

- [ ] T060 Update `specs/002-book-assistant-agent/quickstart.md` with actual commands for running backend + Docusaurus dev
- [ ] T061 Run quickstart.md validation: follow guide end-to-end, verify all steps work

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
