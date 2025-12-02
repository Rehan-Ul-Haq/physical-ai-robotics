"""FastAPI main application for Book Assistant."""

from contextlib import asynccontextmanager
from typing import Any

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse

from app.book_assistant_server import book_assistant_server
from app.config import settings
from app.database import db
from app.models import (
    ChatKitRequest,
    HealthResponse,
    IndexStatus,
    ThreadResponse,
)
from app.qdrant_service import qdrant_service


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan handler."""
    # Startup
    await db.connect()
    yield
    # Shutdown
    await db.disconnect()


app = FastAPI(
    title="Book Assistant AI Agent API",
    description="RAG-powered chatbot API for the Physical AI & Humanoid Robotics book.",
    version="1.0.0",
    lifespan=lifespan,
)

# CORS middleware for frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",   # Docusaurus dev server
        "http://127.0.0.1:3000",
        "http://localhost:5173",   # Vite dev server
        "http://localhost:5174",
        "http://127.0.0.1:5173",
        "http://127.0.0.1:5174",
        "https://rehan-ul-haq.github.io",  # GitHub Pages production
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.post("/assistant/chatkit")
async def chatkit_endpoint(request: Request) -> StreamingResponse:
    """
    Main ChatKit endpoint using server.process().

    Handles user queries, performs RAG search, and streams AI-generated
    responses with citations. Implements ChatKit protocol for thread
    management and streaming.

    Accepts simple format: {query, thread_id?, context?}
    Transforms to ChatKit format: {type, thread_id?, content}
    """
    import json

    try:
        # Get the raw body
        body = await request.body()
        data = json.loads(body)

        # Build context from request
        context: dict[str, Any] = {
            "request": request,
        }

        # Extract context fields from frontend request
        if "context" in data and data["context"]:
            frontend_ctx = data["context"]
            if "context_mode" in frontend_ctx:
                context["context_mode"] = frontend_ctx["context_mode"]
            if "selected_text" in frontend_ctx:
                context["selected_text"] = frontend_ctx["selected_text"]
            if "session_id" in frontend_ctx:
                context["session_id"] = frontend_ctx["session_id"]
            if "page_context" in frontend_ctx:
                context["page_context"] = frontend_ctx["page_context"]

        # Transform simple format to ChatKit protocol format
        thread_id = data.get("thread_id")
        query = data.get("query", "")

        # Build the ChatKit protocol request structure
        # See: chatkit.types.ThreadsCreateReq, ThreadsAddUserMessageReq
        user_input = {
            "content": [{"type": "input_text", "text": query}],
            "attachments": [],
            "inference_options": {},
        }

        if thread_id:
            # Add message to existing thread
            # ChatKit expects thread_id inside params for ThreadsAddUserMessageReq
            chatkit_request = {
                "type": "threads.add_user_message",
                "params": {
                    "thread_id": thread_id,
                    "input": user_input,
                },
            }
        else:
            # Create new thread with first message
            chatkit_request = {
                "type": "threads.create",
                "params": {
                    "input": user_input,
                },
            }

        # Convert to bytes for ChatKit processing
        chatkit_body = json.dumps(chatkit_request).encode()

        # Process through ChatKit server
        result = await book_assistant_server.process(chatkit_body, context)

        return StreamingResponse(
            result,
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
            },
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/assistant/thread/{thread_id}", response_model=ThreadResponse)
async def get_thread(thread_id: str) -> ThreadResponse:
    """Retrieve a ChatKit thread and its items."""
    try:
        thread = await book_assistant_server.store.load_thread(thread_id, {})

        # Load thread items
        items_page = await book_assistant_server.store.load_thread_items(
            thread_id=thread_id,
            after=None,
            limit=100,
            order="asc",
            context={},
        )

        # Convert to response format
        from app.models import ThreadItem as ThreadItemModel, BookAssistantContext
        from datetime import datetime, timezone

        items = []
        for item in items_page.data:
            content = None
            role = None

            if hasattr(item, "content"):
                content = item.content
                if isinstance(content, list):
                    content = "\n".join(str(c) for c in content)

            # Determine role from item type
            item_type = type(item).__name__
            if "User" in item_type:
                role = "user"
            elif "Assistant" in item_type:
                role = "assistant"

            items.append(ThreadItemModel(
                id=item.id,
                thread_id=thread_id,
                type="message",
                role=role,
                content=content,
                sources=None,
                metadata={},
                created_at=getattr(item, "created_at", datetime.now(timezone.utc)),
            ))

        # Parse metadata
        metadata = None
        if thread.metadata:
            try:
                metadata = BookAssistantContext(**thread.metadata)
            except Exception:
                metadata = BookAssistantContext()

        return ThreadResponse(
            id=thread.id,
            title=thread.title,
            metadata=metadata,
            items=items,
            created_at=datetime.now(timezone.utc),  # Would come from DB
            updated_at=datetime.now(timezone.utc),
        )

    except Exception as e:
        raise HTTPException(status_code=404, detail=f"Thread not found: {str(e)}")


@app.delete("/assistant/thread/{thread_id}", status_code=204)
async def delete_thread(thread_id: str) -> None:
    """Delete a thread and all its items (new conversation)."""
    try:
        await book_assistant_server.store.delete_thread(thread_id, {})
    except Exception as e:
        raise HTTPException(status_code=404, detail=f"Thread not found: {str(e)}")


@app.get("/assistant/health", response_model=HealthResponse)
async def health_check() -> HealthResponse:
    """Health check endpoint."""
    checks = {}

    # Check database
    try:
        await db.fetchval("SELECT 1")
        checks["database"] = "ok"
    except Exception:
        checks["database"] = "error"

    # Check Qdrant
    try:
        stats = await qdrant_service.get_collection_stats()
        if stats.get("status") == "error":
            checks["qdrant"] = "error"
        else:
            checks["qdrant"] = "ok"
    except Exception:
        checks["qdrant"] = "error"

    # Check OpenAI (just validate key format)
    if settings.openai_api_key and settings.openai_api_key.startswith("sk-"):
        checks["openai"] = "ok"
    else:
        checks["openai"] = "error"

    overall_status = "healthy" if all(v == "ok" for v in checks.values()) else "unhealthy"

    return HealthResponse(
        status=overall_status,
        version="1.0.0",
        checks=checks,
    )


@app.get("/assistant/index/status", response_model=IndexStatus)
async def get_index_status() -> IndexStatus:
    """Get index status and statistics."""
    stats = await qdrant_service.get_collection_stats()

    return IndexStatus(
        total_chunks=stats.get("total_chunks", 0),
        chapters_indexed=[1],  # MVP: only Chapter 1
        last_indexed_at=None,
        vector_dimension=settings.embedding_dimensions,
    )


# For running with uvicorn directly
if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "app.main:app",
        host=settings.host,
        port=settings.port,
        reload=settings.debug,
    )
