# Feature Specification: Book Assistant AI Agent

**Feature Branch**: `002-book-assistant-agent`
**Created**: 2025-11-28
**Updated**: 2025-11-28
**Status**: MVP Complete (Phase 1-3 Implemented)
**Input**: User description: "Add feature of book assistant AI Agent using OpenAI's Agents SDK for RAG-based Q&A on Physical AI & Humanoid Robotics book content"

## Implementation Summary

**Completed Features**:
- RAG-powered Q&A using OpenAI Agents SDK with ChatKit Server pattern
- Context-aware search with page/chapter/book scope (search_scope parameter)
- Text selection from book content with context injection
- SSE streaming responses via ChatKit protocol
- Thread persistence in Neon Postgres
- Multi-query RAG with context boosting (chapter/lesson awareness)
- ChatWidget integrated in Docusaurus with dark/light mode support

**Key Implementation Decisions**:
- Used `openai-agents` SDK (not `openai-chatkit` as originally planned) for agent orchestration
- Implemented custom `BookAssistantAgentContext` extending `AgentContext` for page context
- Used `RunContextWrapper[Any]` pattern to avoid type annotation runtime errors
- Multi-query search (1-3 queries) with batch embeddings for better recall
- Context boosting: 1.2x for same chapter, 1.1x for same lesson

## Success Evals (Defined First) *(mandatory)*

**Eval-1**: 90% of user queries about topics covered in indexed chapters receive relevant answers (measured by thumbs up/down feedback, minimum 50 samples)

**Eval-2**: Average response time under 3 seconds for typical queries (measured by backend API latency logging)

**Eval-3**: System handles 100 concurrent users without response time degradation beyond 5 seconds (measured by load testing with realistic query patterns)

**Eval-4**: 80% of answers include accurate source citations linking to correct book sections (measured by manual review of 50 random responses)

**Eval-5**: Text selection feature completes end-to-end interaction in under 10 seconds (measured by frontend timer from text selection to response display)

**Eval-6**: Conversation context maintained correctly for 10+ exchanges within session (measured by follow-up question understanding tests using pronouns and implicit references)

## Business Context

**Course Goal Alignment**:
This feature supports the Physical AI & Humanoid Robotics course goal of providing accessible, interactive learning experiences. Students can query complex concepts (ROS 2 nodes, sensor fusion, humanoid kinematics) on-demand rather than searching through chapters.

**Learning Objective Support**:
- Enables just-in-time learning: students ask questions when stuck on exercises or concepts
- Reinforces book content through active questioning vs. passive reading
- Provides source citations to encourage deeper engagement with original book chapters
- Reduces cognitive load by surfacing relevant content without manual search

**Future Course Integration**:
- Chapter 14+ may teach students to build similar RAG agents for domain-specific knowledge bases
- Serves as working example of OpenAI Agents SDK capabilities students will learn later
- Demonstrates production-quality AI integration students can reference

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions About Book Content (Priority: P1)

A reader visiting the Physical AI & Humanoid Robotics book website wants to ask questions about concepts covered in the book. They open the embedded chatbot, type their question (e.g., "What is Physical AI?"), and receive an accurate, contextual answer derived from the book content.

**Why this priority**: Core functionality - without general Q&A capability, the chatbot provides no value. This is the fundamental RAG use case.

**Independent Test**: Can be fully tested by asking any question about Chapter 1 content and verifying the response accurately reflects book material.

**Acceptance Scenarios**:

1. **Given** a reader is on any page of the book site, **When** they click the chat icon, **Then** a chat interface opens ready to receive questions
2. **Given** the chat is open and user types "What is Physical AI?", **When** they submit the question, **Then** they receive an accurate answer synthesized from relevant book sections within 5 seconds
3. **Given** a user asks a question not covered in the book, **When** the query returns no relevant results, **Then** the chatbot gracefully indicates the topic isn't covered and suggests related topics that are

---

### User Story 2 - Context-Specific Text Selection Queries (Priority: P2)

A reader is viewing a specific section about sensor fusion. They highlight a paragraph they don't fully understand, and want to ask clarifying questions specifically about that highlighted text rather than the entire book.

**Why this priority**: Differentiated feature that enhances learning experience. Builds on P1 foundation but adds precision and context-awareness.

**Independent Test**: Can be tested by selecting text on a page, asking a question, and verifying the answer is constrained to the selected context.

**Acceptance Scenarios**:

1. **Given** a reader is viewing book content, **When** they highlight/select a text portion, **Then** a "Ask about this" option becomes available
2. **Given** text is selected and user clicks "Ask about this", **When** they type a clarifying question, **Then** the response is specifically about the selected text, not the entire book
3. **Given** selected text context is active, **When** user asks a question unrelated to the selection, **Then** system informs them the question is outside the selected context and offers to search the full book instead

---

### User Story 3 - Conversation History and Follow-up Questions (Priority: P3)

A reader has asked an initial question and wants to ask follow-up questions without repeating context. The chatbot maintains conversation context to enable natural multi-turn dialogue.

**Why this priority**: Improves user experience significantly but requires P1 to function. Natural conversation flow expected by users familiar with chat interfaces.

**Independent Test**: Can be tested by asking a question, then asking a follow-up using pronouns (e.g., "Can you explain that further?") and verifying context is maintained.

**Acceptance Scenarios**:

1. **Given** user has asked "What sensors does the G1 humanoid use?", **When** they follow up with "How does it process that data?", **Then** the chatbot understands "it" refers to the G1 humanoid
2. **Given** an ongoing conversation, **When** user clicks "New conversation", **Then** previous context is cleared and a fresh session begins
3. **Given** a conversation has 5+ exchanges, **When** user asks a new question, **Then** response time remains under 5 seconds

---

### User Story 4 - Source Citation and Navigation (Priority: P4)

When the chatbot provides an answer, the reader wants to see which sections of the book the information came from, and be able to navigate directly to those sections for deeper reading.

**Why this priority**: Builds trust and encourages deeper engagement with book content. Complements P1-P3 but not essential for basic functionality.

**Independent Test**: Can be tested by asking a question and verifying response includes clickable links to source sections.

**Acceptance Scenarios**:

1. **Given** chatbot provides an answer, **When** the answer is displayed, **Then** it includes citations showing which chapter/section(s) the information came from
2. **Given** a citation is shown (e.g., "Chapter 1, Section 2"), **When** user clicks on it, **Then** they are navigated to that section of the book
3. **Given** an answer synthesizes information from multiple sections, **When** displayed, **Then** all contributing sections are listed as sources

---

### Edge Cases

- What happens when the book content is updated? (Answer: Vector database must be re-indexed via CLI command when markdown files change)
- How does system handle ambiguous questions that could match multiple unrelated sections? (Answer: Return top matches with confidence indicators, let user clarify)
- What happens during high concurrent usage? (Answer: Rate limiting and queue management via FastAPI)
- How does system handle very long text selections? (Answer: Truncate selections over 2000 characters to first 2000 chars, notify user that partial context is being used)
- What happens if Qdrant Cloud is unavailable? (Answer: Display graceful error message, suggest trying again later)
- What happens if user submits malicious prompts attempting injection attacks? (Answer: Input sanitization layer prevents prompt injection before LLM processing)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide an embedded chat interface accessible from any page of the Docusaurus book site
- **FR-002**: System MUST accept natural language questions and return contextually relevant answers from book content
- **FR-003**: System MUST use vector embeddings stored in Qdrant Cloud to perform semantic search over book content
- **FR-004**: System MUST support two query modes: full-book search and selected-text-only search
- **FR-005**: System MUST maintain conversation context within a session to enable follow-up questions
- **FR-006**: System MUST provide source citations linking answers to specific book sections
- **FR-007**: System MUST handle cases where no relevant content is found with appropriate fallback responses
- **FR-008**: System MUST provide CLI command to re-index book content when markdown files are updated (manual trigger by content maintainer)
- **FR-009**: System MUST rate-limit requests to 20 queries per minute per browser session identifier to prevent abuse
- **FR-010**: System MUST respond to queries within 5 seconds under normal load conditions
- **FR-011**: System MUST sanitize all user inputs to prevent prompt injection attacks before passing queries to LLM
- **FR-012**: System MUST store OpenAI API keys in environment variables, never exposing them to frontend code or client requests
- **FR-013**: System MUST persist conversation history in Neon Postgres via ChatKit ThreadStore interface, with automatic expiration after 24 hours
- **FR-014**: System MAY log anonymized query patterns for monitoring and improvement (no personally identifiable information stored)
- **FR-015**: System MUST provide thumbs up/down feedback buttons after each response to measure user satisfaction for success criteria validation

### Key Entities

- **BookChunk**: A semantic unit of book content (paragraph, section) with its vector embedding, source location (chapter/section), and plain text
- **Conversation**: A session-scoped container holding message history, current context mode (full-book or selected-text), and user session identifier
- **Query**: User's question with optional selected-text context, timestamp, and conversation reference
- **Response**: Generated answer text with source citations (chapter/section references) and confidence metadata
- **FeedbackEvent**: User satisfaction signal (thumbs up/down) with response reference and timestamp

## Assumptions

- Docusaurus book site is already deployed and accessible at production URL
- OpenAI API access is available and configured for embedding generation and LLM responses
- Project has valid OpenAI API key with sufficient quota for embedding and completion requests
- Qdrant Cloud free tier provides sufficient storage and query capacity for initial book content (Chapter 1, with growth to full book)
- Book content is primarily English text with code examples
- Users have modern browsers supporting text selection events and ES6+ JavaScript
- Docusaurus site can integrate custom React components for chat interface

## Constraints

- Must extend `ChatKitServer[TContext]` base class from `openai-chatkit` package ✅ IMPLEMENTED
- Must implement `Store` protocol for Neon Postgres conversation persistence ✅ IMPLEMENTED
- Must use OpenAI Agents SDK (`agents` package) for agent orchestration ✅ IMPLEMENTED
- Must integrate ChatWidget as React component in existing Docusaurus site (`book-source/src/components/`) ✅ IMPLEMENTED
- Must use existing Docusaurus deployment (GitHub Pages) - no separate frontend deployment ✅ IMPLEMENTED
- Must use FastAPI for backend API ✅ IMPLEMENTED
- Must use Qdrant Cloud for vector storage ✅ IMPLEMENTED
- ChatWidget must respect Docusaurus dark/light mode via CSS variables ✅ IMPLEMENTED
- Chat interface must not obstruct book content reading experience ✅ IMPLEMENTED
- Must work on both desktop and mobile viewports ✅ IMPLEMENTED

## Actual Dependencies (Installed)

```toml
# Backend (pyproject.toml)
agents = ">=0.0.10"           # OpenAI Agents SDK
openai-chatkit = ">=0.0.3"    # ChatKit Server pattern
openai = ">=1.82.0"           # OpenAI API
fastapi = ">=0.115.12"        # Web framework
qdrant-client = ">=1.14.2"    # Vector database (AsyncQdrantClient)
asyncpg = ">=0.30.0"          # Postgres async driver
pydantic-settings = ">=2.9.1" # Environment config
uvicorn = {extras = ["standard"]}
```

## Out of Scope (Non-Goals)

- Voice input/output capabilities
- Multi-language support (English only for initial release)
- User authentication or personalization
- Analytics dashboard for usage patterns
- Integration with external knowledge bases beyond the book
- Offline functionality
