# ChatKit Server Troubleshooting

Common issues and solutions encountered during ChatKit server implementation.

## NameError with Type Annotations

### Problem
```
NameError: name 'BookAssistantAgentContext' is not defined
```

### Cause
Using specific type annotations with function tool decorators causes runtime errors because decorators evaluate at import time before types are fully resolved.

### Solution
Use `Any` type annotation and access attributes dynamically:

```python
# WRONG - causes NameError
from agents.run_context import RunContextWrapper

@function_tool()
async def my_tool(ctx: RunContextWrapper[BookAssistantAgentContext]) -> str:
    return ctx.context.page_context.current_lesson

# RIGHT - works at runtime
from typing import Any

@function_tool()
async def my_tool(ctx: RunContextWrapper[Any]) -> str:
    if ctx.context is not None and hasattr(ctx.context, "page_context"):
        page_context = ctx.context.page_context
        if page_context is not None:
            return page_context.current_lesson
    return None
```

## SSE Events Not Streaming

### Problem
Frontend receives all events at once instead of streaming.

### Cause
Missing SSE headers or buffering by reverse proxy (nginx).

### Solution
Add all required headers:

```python
return StreamingResponse(
    result,
    media_type="text/event-stream",
    headers={
        "Cache-Control": "no-cache",
        "Connection": "keep-alive",
        "X-Accel-Buffering": "no",  # CRITICAL for nginx
        "Content-Type": "text/event-stream",
    },
)
```

For nginx, also add:
```nginx
proxy_buffering off;
proxy_cache off;
```

## Context Not Reaching Agent Tools

### Problem
`ctx.context.page_context` is `None` even when frontend sends context.

### Cause
Context is passed to `server.process()` but not extracted into `AgentContext` in `respond()`.

### Solution
Extract all context fields in `respond()` method:

```python
async def respond(self, thread, item, context):
    agent_context = CustomAgentContext(thread=thread, store=self.store)

    # MUST extract context fields explicitly
    if "page_context" in context and context["page_context"]:
        page_ctx = context["page_context"]
        agent_context.page_context = PageContext(
            current_chapter=page_ctx.get("current_chapter"),
            current_lesson=page_ctx.get("current_lesson"),
        )

    # Also support flat structure for backwards compatibility
    elif "current_chapter" in context or "current_lesson" in context:
        agent_context.page_context = PageContext(
            current_chapter=context.get("current_chapter"),
            current_lesson=context.get("current_lesson"),
        )

    result = Runner.run_streamed(..., context=agent_context)
```

## Frontend Context Not Sent

### Problem
Backend receives empty `context` object.

### Cause
Frontend not including context in request body or using wrong field name.

### Solution
Verify frontend sends nested context structure:

```typescript
const response = await fetch('/assistant/chatkit', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
        query: message,
        thread_id: threadId,
        context: {  // MUST be named 'context'
            page_context: {  // MUST be named 'page_context'
                current_chapter: 1,
                current_lesson: 'reality-gap',
            },
        },
    }),
});
```

Backend extraction:
```python
if "context" in data and data["context"]:
    frontend_ctx = data["context"]
    if "page_context" in frontend_ctx:
        context["page_context"] = frontend_ctx["page_context"]
```

## Qdrant AsyncClient Errors

### Problem
```
RuntimeError: This event loop is already running
```

### Cause
Using synchronous `QdrantClient` in async context.

### Solution
Always use `AsyncQdrantClient`:

```python
from qdrant_client import AsyncQdrantClient

class QdrantService:
    def __init__(self):
        self._client: AsyncQdrantClient | None = None

    async def _get_client(self) -> AsyncQdrantClient:
        if self._client is None:
            self._client = AsyncQdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
            )
        return self._client
```

## Text Selection Capturing Chat Window

### Problem
Selecting text inside chat widget triggers the book text selection handler.

### Cause
Event listener on `document` captures all selections.

### Solution
Use ref to exclude chat window from selection handling:

```typescript
const chatWindowRef = useRef<HTMLDivElement>(null);

const handleMouseUp = (event: MouseEvent) => {
    // Check if selection is inside chat window
    if (chatWindowRef.current && chatWindowRef.current.contains(event.target as Node)) {
        return;  // Skip processing
    }
    // Handle book text selection...
};

// In JSX
<div ref={chatWindowRef} className={styles.chatWindow}>
```

## Thread Metadata Not Persisting

### Problem
Custom metadata (like `previous_response_id`) disappears between requests.

### Cause
Not saving thread metadata after updating it.

### Solution
Save thread after modifying metadata:

```python
async def respond(self, thread, item, context):
    # ... run agent ...

    if hasattr(result, "response_id") and result.response_id:
        if thread.metadata is None:
            thread.metadata = {}
        thread.metadata["previous_response_id"] = result.response_id
        await self.store.save_thread(thread, context)  # DON'T FORGET THIS
```

## JSON Parsing Errors in SSE

### Problem
```
SyntaxError: Unexpected end of JSON input
```

### Cause
SSE data split across multiple chunks, attempting to parse incomplete JSON.

### Solution
Buffer incomplete lines:

```typescript
let buffer = '';

while (true) {
    const { done, value } = await reader.read();
    if (done) break;

    buffer += decoder.decode(value, { stream: true });
    const lines = buffer.split('\n');
    buffer = lines.pop() || '';  // Keep incomplete line in buffer

    for (const line of lines) {
        if (line.startsWith('data: ')) {
            try {
                const data = JSON.parse(line.slice(6));
                handleEvent(data);
            } catch (e) {
                // Ignore parse errors, might be empty line
            }
        }
    }
}
```

## CORS Errors

### Problem
```
Access to fetch at 'http://localhost:8001/assistant/chatkit' from origin 'http://localhost:3000' has been blocked by CORS policy
```

### Solution
Add CORS middleware with correct origins:

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://127.0.0.1:3000",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## Debug Logging

Add debug logging to trace context flow:

```python
import logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

async def respond(self, thread, item, context):
    logger.debug(f"Received context: {context}")
    # ...
    logger.debug(f"Agent context page_context: {agent_context.page_context}")
```

Remove debug logging before production!
