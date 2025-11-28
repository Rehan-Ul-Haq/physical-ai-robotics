"""Pydantic models for Book Assistant."""

from datetime import datetime
from enum import Enum
from typing import Any
from pydantic import BaseModel, Field


class ContextMode(str, Enum):
    """Query context mode."""

    FULL_BOOK = "full_book"
    SELECTED_TEXT = "selected_text"


class Source(BaseModel):
    """Source citation from RAG search."""

    chapter: int = Field(..., ge=1, description="Chapter number")
    section: str = Field(..., description="Section title or ID")
    url: str = Field(..., description="URL to source location")
    relevance_score: float = Field(..., ge=0, le=1, description="Similarity score")


class BookChunkPayload(BaseModel):
    """Payload for book content chunks in Qdrant."""

    text: str
    chapter_number: int
    chapter_title: str
    section_id: str
    section_title: str
    source_url: str
    part_number: int
    token_count: int


class BookAssistantContext(BaseModel):
    """Context metadata for BookAssistant threads."""

    session_id: str | None = None
    context_mode: ContextMode = ContextMode.FULL_BOOK
    selected_text: str | None = None


class ThreadItemCreate(BaseModel):
    """Create request for thread item."""

    type: str = "message"
    role: str
    content: str
    sources: list[Source] | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)


class ThreadItem(BaseModel):
    """Thread item (message) model."""

    id: str
    thread_id: str
    type: str = "message"
    role: str | None = None
    content: str | None = None
    sources: list[Source] | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)
    created_at: datetime


class Thread(BaseModel):
    """Thread model."""

    id: str
    title: str | None = None
    metadata: BookAssistantContext = Field(default_factory=BookAssistantContext)
    created_at: datetime
    updated_at: datetime


class ThreadResponse(BaseModel):
    """Thread response with items."""

    id: str
    title: str | None = None
    metadata: BookAssistantContext | None = None
    items: list[ThreadItem] = Field(default_factory=list)
    created_at: datetime
    updated_at: datetime


class FeedbackType(str, Enum):
    """Feedback type enum."""

    THUMBS_UP = "thumbs_up"
    THUMBS_DOWN = "thumbs_down"


class FeedbackCreate(BaseModel):
    """Create request for feedback."""

    thread_item_id: str
    session_id: str
    feedback_type: FeedbackType


class FeedbackResponse(BaseModel):
    """Feedback response model."""

    id: str
    thread_item_id: str
    feedback_type: FeedbackType
    created_at: datetime


class ChatKitRequest(BaseModel):
    """ChatKit request model."""

    query: str = Field(..., max_length=2000, description="User's question")
    thread_id: str | None = Field(None, max_length=255, description="Existing thread ID")
    context: BookAssistantContext | None = Field(
        None, description="Thread context (context_mode, selected_text)"
    )


class HealthResponse(BaseModel):
    """Health check response."""

    status: str
    version: str = "1.0.0"
    checks: dict[str, str] = Field(default_factory=dict)


class IndexStatus(BaseModel):
    """Index status response."""

    total_chunks: int
    chapters_indexed: list[int]
    last_indexed_at: datetime | None = None
    vector_dimension: int = 1536
