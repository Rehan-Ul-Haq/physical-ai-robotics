---
name: chatkit-server-implementation
description: This skill should be used when building ChatKit-based AI assistant servers with the OpenAI Agents SDK. It provides guidance for implementing SSE streaming, thread persistence, context passing, RAG integration, and frontend ChatKit widget integration. Use when creating conversational AI backends that need real-time streaming, thread management, or agentic tool use.
---

# ChatKit Server Implementation

This skill provides comprehensive guidance for implementing ChatKit-based AI assistant servers using OpenAI's ChatKit Python SDK with the Agents SDK for agentic workflows.

## Context & Problem

ChatKit provides a protocol for building conversational AI experiences with real-time streaming, thread management, and rich widget support. However, implementing a production-ready ChatKit server involves solving several interconnected problems:

1. **SSE Event Format**: ChatKit uses specific SSE event formats that must be precisely matched
2. **Thread Persistence**: Conversation history must be stored and retrieved correctly
3. **Context Passing**: User context (page location, selected text) must flow from frontend to agent tools
4. **Agent Integration**: OpenAI Agents SDK tools need access to request context
5. **RAG Integration**: Search tools need context-aware boosting and filtering

## Core Architecture

### Layer 1: FastAPI Endpoint

The ChatKit endpoint receives simple requests and transforms them to ChatKit protocol:

```python
@app.post("/assistant/chatkit")
async def chatkit_endpoint(request: Request) -> StreamingResponse:
    body = await request.body()
    data = json.loads(body)

    # Build context from request
    context: dict[str, Any] = {"request": request}

    # Extract frontend context (page_context, selected_text, etc.)
    if "context" in data and data["context"]:
        frontend_ctx = data["context"]
        if "page_context" in frontend_ctx:
            context["page_context"] = frontend_ctx["page_context"]

    # Transform to ChatKit protocol format
    thread_id = data.get("thread_id")
    query = data.get("query", "")

    user_input = {
        "content": [{"type": "input_text", "text": query}],
        "attachments": [],
        "inference_options": {},
    }

    if thread_id:
        chatkit_request = {
            "type": "threads.add_user_message",
            "params": {"thread_id": thread_id, "input": user_input},
        }
    else:
        chatkit_request = {
            "type": "threads.create",
            "params": {"input": user_input},
        }

    chatkit_body = json.dumps(chatkit_request).encode()
    result = await server.process(chatkit_body, context)

    return StreamingResponse(
        result,
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        },
    )
```

### Layer 2: ChatKitServer Extension

Extend `ChatKitServer` with custom `AgentContext` for passing request data to tools:

```python
from chatkit.agents import AgentContext, stream_agent_response
from chatkit.server import ChatKitServer
from chatkit.types import ThreadMetadata, ThreadStreamEvent, UserMessageItem

class CustomAgentContext(AgentContext):
    """Extended AgentContext with application-specific fields."""
    selected_text: str | None = None
    context_mode: str = "full_book"
    page_context: PageContext | None = None

class CustomServer(ChatKitServer[dict[str, Any]]):
    def __init__(self) -> None:
        store = PostgresThreadStore()
        super().__init__(store)
        self.assistant = my_agent

    async def respond(
        self,
        thread: ThreadMetadata,
        item: UserMessageItem | None,
        context: dict[str, Any],
    ) -> AsyncIterator[ThreadStreamEvent]:
        # Create agent context with custom fields
        agent_context = CustomAgentContext(
            thread=thread,
            store=self.store,
            request_context=context,
        )

        # Extract custom context from request
        if "page_context" in context and context["page_context"]:
            page_ctx = context["page_context"]
            agent_context.page_context = PageContext(
                current_chapter=page_ctx.get("current_chapter"),
                current_lesson=page_ctx.get("current_lesson"),
            )

        # Convert user message to agent input
        agent_input = await to_agent_input(thread, item, self.store, context)

        if not agent_input:
            return

        # Run agent with streaming
        result = Runner.run_streamed(
            self.assistant,
            input=agent_input,
            context=agent_context,
        )

        # Stream response events
        async for event in stream_agent_response(agent_context, result):
            yield event
```

### Layer 3: Agent Tools with Context Access

Access the custom context from within agent tools using `RunContextWrapper`:

```python
from agents import function_tool
from agents.run_context import RunContextWrapper
from typing import Any, Annotated

@function_tool(description_override="Search book content with context awareness.")
async def search_book_content(
    ctx: RunContextWrapper[Any],  # Use Any to avoid runtime NameError
    queries: Annotated[list[str], "Search queries"],
    search_scope: Annotated[str, "Search scope"] = "full_book",
    current_chapter: Annotated[int | None, "Chapter number"] = None,
    current_lesson: Annotated[str | None, "Lesson slug"] = None,
) -> dict:
    # Access custom context via ctx.context
    if ctx.context is not None:
        agent_ctx = ctx.context
        if hasattr(agent_ctx, "page_context") and agent_ctx.page_context is not None:
            if current_chapter is None:
                current_chapter = agent_ctx.page_context.current_chapter
            if current_lesson is None:
                current_lesson = agent_ctx.page_context.current_lesson

    # Perform search with context-aware boosting
    results = await search_service.search(
        queries=queries,
        current_chapter=current_chapter,
        current_lesson=current_lesson,
        search_scope=search_scope,
    )

    return {"found": True, "context": results}
```

### Layer 4: Thread Store Implementation

Implement the ChatKit `Store` protocol for thread persistence:

```python
from chatkit.store import Store
from chatkit.types import Page, ThreadItem, ThreadMetadata

class PostgresThreadStore(Store[dict[str, Any]]):
    def generate_thread_id(self, context: dict[str, Any]) -> str:
        return f"thread_{uuid.uuid4().hex[:16]}"

    def generate_item_id(
        self,
        item_type: Literal["message", "tool_call", "task", "workflow", "attachment"],
        thread: ThreadMetadata,
        context: dict[str, Any],
    ) -> str:
        return f"{item_type}_{uuid.uuid4().hex[:16]}"

    async def load_thread(self, thread_id: str, context: dict[str, Any]) -> ThreadMetadata:
        row = await db.fetchrow("SELECT ... FROM threads WHERE id = $1", thread_id)
        if row is None:
            return ThreadMetadata(id=thread_id, title=None, metadata={}, created_at=datetime.now(timezone.utc))
        return ThreadMetadata(id=row["id"], title=row["title"], metadata=row["metadata"], created_at=row["created_at"])

    async def save_thread(self, thread: ThreadMetadata, context: dict[str, Any]) -> None:
        await db.execute("""
            INSERT INTO threads (id, title, metadata, created_at, updated_at)
            VALUES ($1, $2, $3, NOW(), NOW())
            ON CONFLICT (id) DO UPDATE SET title = EXCLUDED.title, metadata = EXCLUDED.metadata, updated_at = NOW()
        """, thread.id, thread.title, json.dumps(thread.metadata))

    async def add_thread_item(self, thread_id: str, item: ThreadItem, context: dict[str, Any]) -> None:
        # Extract role and content from item type
        if isinstance(item, UserMessageItem):
            role = "user"
            content = extract_text_from_user_content(item.content)
        elif isinstance(item, AssistantMessageItem):
            role = "assistant"
            content = extract_text_from_assistant_content(item.content)

        await db.execute("""
            INSERT INTO thread_items (id, thread_id, type, role, content, created_at)
            VALUES ($1, $2, $3, $4, $5, $6)
        """, item.id, thread_id, "message", role, content, item.created_at)
```

## Dimensional Guidance

### When to Filter vs Boost in Search

**Filter** (strict boundary, may miss relevant content):
- User asks about specific content they're reading: `search_scope="current_page"`
- User asks about current chapter topics: `search_scope="current_chapter"`

**Boost** (wider net, prioritizes context):
- General/conceptual questions: `search_scope="full_book"` with context boosting
- Cross-chapter connections: Full search with chapter boost factor (1.2x)

### SSE Event Format Requirements

ChatKit SSE events must follow this exact format:

```
event: thread.created
data: {"id": "thread_xxx", "title": null, "metadata": {}}

event: item.created
data: {"id": "msg_xxx", "type": "message", "role": "assistant", "content": []}

event: item.delta
data: {"id": "msg_xxx", "delta": {"type": "output_text", "text": "Hello"}}

event: item.done
data: {"id": "msg_xxx"}

event: done
data: {}
```

### Type Annotations with Decorators

**Problem**: Using `RunContextWrapper[CustomContext]` causes `NameError` at runtime because decorators evaluate types before imports complete.

**Solution**: Use `RunContextWrapper[Any]` and access context attributes dynamically:

```python
# DON'T: Causes NameError at decorator evaluation time
async def my_tool(ctx: RunContextWrapper[CustomAgentContext]) -> str: ...

# DO: Works at runtime with dynamic attribute access
async def my_tool(ctx: RunContextWrapper[Any]) -> str:
    if ctx.context is not None and hasattr(ctx.context, "page_context"):
        page_context = ctx.context.page_context
```

### Frontend Integration

ChatKit frontend widget must:

1. **Send context with requests**:
```typescript
const sendMessage = async (message: string) => {
    const response = await fetch('/assistant/chatkit', {
        method: 'POST',
        body: JSON.stringify({
            query: message,
            thread_id: threadId,
            context: {
                page_context: {
                    current_chapter: getChapterFromPath(),
                    current_lesson: getLessonFromPath(),
                },
                selected_text: selectedText,
            },
        }),
    });
    // Handle SSE stream...
};
```

2. **Parse SSE events correctly**:
```typescript
const reader = response.body.getReader();
const decoder = new TextDecoder();
let buffer = '';

while (true) {
    const { done, value } = await reader.read();
    if (done) break;

    buffer += decoder.decode(value, { stream: true });
    const lines = buffer.split('\n');
    buffer = lines.pop() || '';

    for (const line of lines) {
        if (line.startsWith('data: ')) {
            const data = JSON.parse(line.slice(6));
            // Handle event based on type
        }
    }
}
```

3. **Exclude chat widget from text selection capture**:
```typescript
const chatWindowRef = useRef<HTMLDivElement>(null);

useEffect(() => {
    const handleMouseUp = (event: MouseEvent) => {
        // Ignore selections inside chat window
        if (chatWindowRef.current?.contains(event.target as Node)) {
            return;
        }
        // Capture book text selection...
    };
}, []);
```

## Anti-Patterns

### 1. Hardcoded Configuration

**Wrong**:
```python
book_base_url = "/physical-ai-robotics"
```

**Right**:
```python
# config.py using pydantic-settings
class Settings(BaseSettings):
    book_base_url: str = "/physical-ai-robotics"  # Override with BOOK_BASE_URL env var

    class Config:
        env_file = ".env"
```

### 2. Synchronous Qdrant Client

**Wrong**:
```python
from qdrant_client import QdrantClient
client = QdrantClient(url=url)  # Blocks async event loop
```

**Right**:
```python
from qdrant_client import AsyncQdrantClient

class QdrantService:
    async def _get_client(self) -> AsyncQdrantClient:
        if self._client is None:
            self._client = AsyncQdrantClient(url=url, api_key=api_key)
        return self._client
```

### 3. Missing Context Extraction

**Wrong**:
```python
async def respond(self, thread, item, context):
    # Ignores frontend context
    result = Runner.run_streamed(self.assistant, input=agent_input)
```

**Right**:
```python
async def respond(self, thread, item, context):
    agent_context = CustomAgentContext(thread=thread, store=self.store)

    # Extract ALL context from frontend
    if "page_context" in context:
        agent_context.page_context = PageContext(**context["page_context"])

    result = Runner.run_streamed(self.assistant, input=agent_input, context=agent_context)
```

### 4. Not Streaming SSE Headers

**Wrong**:
```python
return StreamingResponse(result, media_type="text/event-stream")
```

**Right**:
```python
return StreamingResponse(
    result,
    media_type="text/event-stream",
    headers={
        "Cache-Control": "no-cache",
        "Connection": "keep-alive",
        "X-Accel-Buffering": "no",  # Critical for nginx
    },
)
```

## Creative Variance

This skill intentionally leaves room for domain-specific decisions:

1. **Search Scope Strategy**: The `current_page` vs `current_chapter` vs `full_book` strategy depends on content density and user behavior patterns

2. **Boost Factors**: Chapter boost (1.2x) and lesson boost (1.1x) are tunable based on content relevance patterns

3. **Thread Metadata**: What metadata to store (session_id, selected_text history, etc.) depends on analytics needs

4. **Agent Instructions**: The system prompt and tool descriptions should be tailored to the specific content domain

5. **Widget Actions**: ChatKit supports `action` handlers for interactive widgets—implement based on UI requirements

## Database Schema

Required PostgreSQL tables for thread persistence:

```sql
CREATE TABLE threads (
    id VARCHAR(255) PRIMARY KEY,
    title TEXT,
    metadata JSONB DEFAULT '{}',
    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE TABLE thread_items (
    id VARCHAR(255) PRIMARY KEY,
    thread_id VARCHAR(255) REFERENCES threads(id) ON DELETE CASCADE,
    type VARCHAR(50) DEFAULT 'message',
    role VARCHAR(50),
    content TEXT,
    sources JSONB,
    metadata JSONB DEFAULT '{}',
    created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_thread_items_thread_id ON thread_items(thread_id);
CREATE INDEX idx_thread_items_created_at ON thread_items(created_at);
```

## Dependencies

```
openai-chatkit>=0.1.0
openai-agents>=0.1.0
fastapi>=0.100.0
qdrant-client>=1.7.0
asyncpg>=0.29.0
pydantic-settings>=2.0.0
```
