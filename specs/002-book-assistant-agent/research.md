# Research: Book Assistant AI Agent

**Feature Branch**: `002-book-assistant-agent`
**Date**: 2025-11-28
**Status**: Complete (Updated for ChatKit Server Pattern)

## Research Questions Addressed

### 1. OpenAI ChatKit Server Pattern (Primary Architecture)

**Decision**: Extend `ChatKitServer[TContext]` base class from `openai-chatkit` package

**Rationale**:
- ChatKit provides `ChatKitServer` base class for building custom chat servers
- Handles streaming, thread management, and request processing automatically
- Integrates with `ThreadStore` protocol for conversation persistence
- Reference implementation: [customer-support example](https://github.com/openai/openai-chatkit-advanced-samples/tree/main/examples/customer-support)

**Required Packages** (from customer-support pyproject.toml):
```toml
dependencies = [
    "fastapi>=0.114.1,<0.116",
    "openai>=1.40",
    "openai-chatkit>=1.1.2,<2",
    "httpx>=0.28,<0.29",
    "uvicorn[standard]>=0.36,<0.37",
]
```

**Implementation Pattern**:
```python
from typing import Any
from openai_chatkit import ChatKitServer, ThreadStore, Agent

# Define typed context for thread state
class BookAssistantContext(TypedDict):
    context_mode: str  # "full_book" or "selected_text"
    selected_text: str | None

# Extend ChatKitServer with typed context
class BookAssistantServer(ChatKitServer[BookAssistantContext]):
    def __init__(self, store: ThreadStore, agent: Agent[BookAssistantContext]):
        super().__init__(store=store)
        self.agent = agent

    async def respond(
        self,
        thread_id: str,
        user_message: str,
        context: BookAssistantContext,
    ) -> AsyncIterator[str]:
        """Process user message and stream response."""
        # Get thread items for conversation history
        thread = await self.store.get_thread(thread_id)
        items = convert_thread_to_items(thread)

        # Run agent with context
        async for chunk in stream_agent_response(
            self.agent, user_message, items, context
        ):
            yield chunk

    async def action(
        self,
        thread_id: str,
        action_type: str,
        payload: dict[str, Any],
        context: BookAssistantContext,
    ) -> dict[str, Any]:
        """Handle custom actions (feedback, context update)."""
        if action_type == "feedback":
            return await self.handle_feedback(thread_id, payload)
        return {"status": "unknown_action"}
```

**Key Classes**:
- `ChatKitServer[TContext]`: Base class with `respond()` and `action()` methods
- `ThreadStore`: Protocol for conversation persistence (we implement `PostgresThreadStore`)
- `Agent[TContext]`: Typed agent with tools and context access

**Alternatives Considered**:
- Raw FastAPI: Requires manual streaming, thread management - ChatKitServer provides this
- LangChain: Heavier abstraction, not compatible with ChatKit protocol
- Direct OpenAI API: No ChatKit integration, manual SSE handling

---

### 2. OpenAI Agents SDK (Agent Definition)

**Decision**: Use `Agent[TContext]` with function tools for RAG

**Rationale**:
- Agents SDK (via `openai` package >=1.40) provides agent orchestration
- `function_tool` decorator generates JSON schemas from Python signatures
- Agents receive typed context for accessing thread state
- Works with ChatKit's streaming infrastructure

**Implementation Pattern**:
```python
from openai.agents import Agent, function_tool

@function_tool
async def search_book_content(
    ctx: RunContext[BookAssistantContext],
    query: str,
) -> str:
    """Search book content using RAG pipeline.

    Args:
        ctx: Agent context with thread state
        query: User's question about book content
    """
    context_mode = ctx.context.get("context_mode", "full_book")
    selected_text = ctx.context.get("selected_text")

    # RAG search with Qdrant
    results = await qdrant_service.search(
        query=query,
        filter_text=selected_text if context_mode == "selected_text" else None
    )
    return format_results(results)

# Agent with typed context
agent: Agent[BookAssistantContext] = Agent(
    name="BookAssistant",
    instructions="""You are a helpful assistant for the Physical AI & Humanoid Robotics book.
    Always cite sources when answering. Use the search_book_content tool to find relevant information.""",
    tools=[search_book_content],
)
```

---

### 3. ThreadStore Implementation (Postgres)

**Decision**: Implement `ThreadStore` protocol with Neon Postgres backend

**Rationale**:
- ChatKit defines `ThreadStore` protocol for conversation persistence
- Customer-support example uses `MemoryStore` - we implement `PostgresThreadStore`
- Enables 24-hour conversation retention with automatic cleanup
- asyncpg provides non-blocking database operations

**ThreadStore Protocol**:
```python
from openai_chatkit import ThreadStore, Thread, ThreadItem

class PostgresThreadStore(ThreadStore):
    """ThreadStore implementation backed by Neon Postgres."""

    def __init__(self, pool: asyncpg.Pool):
        self.pool = pool

    async def get_thread(self, thread_id: str) -> Thread | None:
        """Get thread with all items."""
        async with self.pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT * FROM threads WHERE id = $1", thread_id
            )
            if not row:
                return None
            items = await conn.fetch(
                "SELECT * FROM thread_items WHERE thread_id = $1 ORDER BY created_at",
                thread_id
            )
            return Thread(
                id=row["id"],
                metadata=row["metadata"],
                items=[ThreadItem(**item) for item in items]
            )

    async def create_thread(self, metadata: dict[str, Any] = None) -> Thread:
        """Create new thread."""
        thread_id = str(uuid.uuid4())
        async with self.pool.acquire() as conn:
            await conn.execute(
                "INSERT INTO threads (id, metadata) VALUES ($1, $2)",
                thread_id, json.dumps(metadata or {})
            )
        return Thread(id=thread_id, metadata=metadata or {}, items=[])

    async def add_item(self, thread_id: str, item: ThreadItem) -> None:
        """Add item to thread."""
        async with self.pool.acquire() as conn:
            await conn.execute(
                """INSERT INTO thread_items (id, thread_id, role, content, sources, created_at)
                   VALUES ($1, $2, $3, $4, $5, $6)""",
                item.id, thread_id, item.role, item.content,
                json.dumps(item.sources) if item.sources else None,
                item.created_at
            )

    async def delete_thread(self, thread_id: str) -> None:
        """Delete thread and all items."""
        async with self.pool.acquire() as conn:
            await conn.execute("DELETE FROM threads WHERE id = $1", thread_id)
```

**Alternatives Considered**:
- MemoryStore (ChatKit default): No persistence, lost on restart
- Redis: Better for caching, Postgres better for structured thread data
- SQLite: Not cloud-native, doesn't scale

---

### 4. RAG Pipeline with Qdrant

**Decision**: Qdrant Cloud (free tier) with OpenAI embeddings

**Rationale**:
- Qdrant Python client supports both sync and async operations
- Simple upsert/search API with payload filtering
- Free tier sufficient for book content (~1-5 chapters initially)
- Native filtering for context-specific queries (selected text)

**Implementation Pattern**:
```python
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, Filter, FieldCondition, MatchValue

# Initialize client
client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Upsert book chunks
client.upsert(
    collection_name="book_content",
    points=[
        PointStruct(
            id=chunk_id,
            vector=embedding,
            payload={
                "text": chunk_text,
                "chapter": chapter_num,
                "section": section_id,
                "source_url": "/docs/part1/chapter1#section2"
            }
        )
    ]
)

# Search with optional filter
results = client.search(
    collection_name="book_content",
    query_vector=query_embedding,
    query_filter=Filter(
        must=[FieldCondition(key="chapter", match=MatchValue(value=1))]
    ) if filter_by_chapter else None,
    limit=5
)
```

**Chunking Strategy**:
- Semantic chunking by markdown headings (H2/H3)
- Chunk size: 500-1000 tokens with 100 token overlap
- Preserve source location metadata for citations

**Alternatives Considered**:
- Pinecone: More expensive, similar capabilities
- ChromaDB: Local only, no cloud tier
- pgvector: Requires more setup, Qdrant has better Python SDK

---

### 5. Text Selection Feature (P2 User Story)

**Decision**: Frontend text selection event → API with selected context

**Rationale**:
- Browser `window.getSelection()` API captures highlighted text
- Selected text passed as additional context to RAG query
- RAG filters or weighs results based on selected text similarity

**Implementation Pattern**:
```javascript
// Frontend: Capture selection
document.addEventListener('mouseup', () => {
    const selection = window.getSelection().toString().trim();
    if (selection.length > 0 && selection.length <= 2000) {
        setSelectedContext(selection);
        showAskAboutThisButton(true);
    }
});

// API call with selected context
const response = await fetch('/chatkit', {
    method: 'POST',
    body: JSON.stringify({
        query: userQuestion,
        selected_text: selectedContext,
        context_mode: 'selected_text'
    })
});
```

**Backend Handling**:
- If `selected_text` provided, use it as primary context
- Embed selected text and search for related chunks
- Response limited to selected context unless explicitly expanded

---

### 6. FastAPI Integration with ChatKit

**Decision**: Mount ChatKitServer on FastAPI app using `server.process()` endpoint

**Rationale**:
- ChatKitServer integrates with FastAPI via `server.process()` method
- Returns `StreamingResponse` with `"text/event-stream"` media type
- Health check and admin endpoints remain standard FastAPI routes

**Implementation Pattern** (from customer-support example):
```python
from fastapi import FastAPI, Request
from fastapi.responses import StreamingResponse

app = FastAPI()

# Create server with store and agent
store = PostgresThreadStore(pool)
agent = create_book_assistant_agent()
server = BookAssistantServer(store=store, agent=agent)

@app.post("/assistant/chatkit")
async def chatkit_endpoint(request: Request) -> StreamingResponse:
    """Main ChatKit endpoint using server.process()."""
    return StreamingResponse(
        server.process(request),
        media_type="text/event-stream"
    )

@app.get("/assistant/thread/{thread_id}")
async def get_thread(thread_id: str):
    """Get thread state for debugging/context."""
    thread = await store.get_thread(thread_id)
    if not thread:
        raise HTTPException(status_code=404)
    return thread

@app.get("/assistant/health")
async def health():
    """Health check endpoint."""
    return {"status": "healthy", "version": "1.0.0"}
```

**Key Integration Points**:
- `server.process(request)` handles ChatKit protocol
- StreamingResponse enables SSE for real-time responses
- Thread management via `ThreadStore` abstraction

---

### 7. Source Citations

**Decision**: Store source metadata in Qdrant payload, return with responses

**Rationale**:
- Each chunk stores: chapter, section, source_url
- RAG results include metadata alongside content
- LLM instructed to cite sources in response
- Frontend renders clickable links

**Citation Format**:
```json
{
    "answer": "Physical AI refers to...",
    "sources": [
        {
            "chapter": 1,
            "section": "Introduction to Physical AI",
            "url": "/docs/part1/chapter1#introduction",
            "relevance_score": 0.92
        }
    ]
}
```

---

### 8. Security Considerations

**Decision**: Multi-layer security approach

**Implemented Measures**:
1. **Input Sanitization**: Strip HTML/scripts, validate length (< 2000 chars)
2. **Prompt Injection Prevention**:
   - Separate system prompts from user content
   - Use structured tool outputs
   - Content validation before LLM input
3. **API Key Security**: Environment variables only, never in frontend
4. **Rate Limiting**: 20 requests/minute per session (FastAPI middleware)
5. **No PII Storage**: Anonymous session IDs, no user accounts

---

## Technology Stack Summary

| Component | Technology | Version | Purpose |
|-----------|------------|---------|---------|
| ChatKit Server | openai-chatkit | >=1.1.2,<2 | ChatKitServer base class, ThreadStore protocol |
| Agent Framework | openai (Agents SDK) | >=1.40 | Agent orchestration, function tools |
| Chat Frontend | ChatKit Web Component | Latest | Embedded chat UI |
| Backend API | FastAPI | >=0.114.1,<0.116 | REST API server |
| HTTP Client | httpx | >=0.28,<0.29 | Async HTTP requests |
| ASGI Server | uvicorn[standard] | >=0.36,<0.37 | Production server |
| Vector Database | Qdrant Cloud | Free tier | Semantic search |
| Chat Storage | Neon Postgres | Latest | ThreadStore persistence |
| Python Driver | asyncpg | Latest | Async Postgres operations |
| Embeddings | OpenAI text-embedding-3-small | Latest | Vector generation |
| LLM | GPT-4o-mini or GPT-4o | Latest | Response generation |

---

## Open Items Resolved

| Item | Resolution |
|------|------------|
| Chat history storage | Neon Postgres via ChatKit ThreadStore interface |
| Server architecture | Extend `ChatKitServer[TContext]` base class |
| Correct package | `openai-chatkit >=1.1.2,<2` (not `openai-agents`) |
| ThreadStore implementation | `PostgresThreadStore` implements ChatKit `ThreadStore` protocol |
| Embedding model | text-embedding-3-small (cost-effective, sufficient quality) |
| Chunk size | 500-1000 tokens with semantic boundaries |
| Rate limit threshold | 20 req/min per session |
| Citation format | JSON with chapter, section, URL, relevance score |

---

## Architecture Summary

```
                 ┌─────────────────────────────────────────┐
                 │           FastAPI Application           │
                 └─────────────────────────────────────────┘
                                    │
            ┌───────────────────────┼───────────────────────┐
            │                       │                       │
   ┌────────▼────────┐    ┌────────▼────────┐    ┌────────▼────────┐
   │  /assistant/    │    │  /assistant/    │    │  /assistant/    │
   │    chatkit      │    │    thread/      │    │    health       │
   │   (POST)        │    │   {thread_id}   │    │    (GET)        │
   └────────┬────────┘    └─────────────────┘    └─────────────────┘
            │
   ┌────────▼────────────────────────────────┐
   │     BookAssistantServer                 │
   │     (extends ChatKitServer[Context])    │
   │     - respond()                         │
   │     - action()                          │
   └────────┬────────────────────────────────┘
            │
   ┌────────┴────────┐              ┌─────────────────────┐
   │                 │              │                     │
┌──▼──────────────┐  │       ┌──────▼──────────────────┐  │
│ PostgresThread  │  │       │ Agent[BookAssistant     │  │
│ Store           │  │       │ Context]                │  │
│ (ThreadStore)   │  │       │ - search_book_content   │  │
└───────┬─────────┘  │       └──────────┬──────────────┘  │
        │            │                  │                 │
   ┌────▼────┐  ┌────▼────┐       ┌─────▼─────┐    ┌──────▼──────┐
   │  Neon   │  │  Neon   │       │  Qdrant   │    │   OpenAI    │
   │Postgres │  │Postgres │       │  Cloud    │    │     API     │
   │(threads)│  │(feedback│       │ (vectors) │    │ (LLM/embed) │
   └─────────┘  └─────────┘       └───────────┘    └─────────────┘
```

---

## Next Steps

1. **Complete**: data-model.md updated with ThreadStore schema
2. **Complete**: API contracts updated in OpenAPI format
3. **Complete**: quickstart.md updated with ChatKit dependencies
4. **Ready**: Run `/sp.tasks` to generate implementation tasks
