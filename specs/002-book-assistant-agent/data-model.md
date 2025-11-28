# Data Model: Book Assistant AI Agent

**Feature Branch**: `002-book-assistant-agent`
**Date**: 2025-11-28
**Updated**: 2025-11-28 (ChatKit ThreadStore alignment)

## Entity Relationship Overview

Following ChatKit `ThreadStore` protocol for conversation persistence:

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   BookChunk     │     │     Thread      │────>│   ThreadItem    │
│  (Qdrant)       │     │  (Postgres)     │     │   (Postgres)    │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                               │                        │
                               │ metadata (JSONB)       │
                               │ - context_mode         v
                               │ - selected_text  ┌─────────────────┐
                               └─────────────────>│ FeedbackEvent   │
                                                  │   (Postgres)    │
                                                  └─────────────────┘
```

**Architecture Note**: This data model implements ChatKit's `ThreadStore` protocol. The `Thread` and `ThreadItem` entities map to ChatKit's abstract types, while `PostgresThreadStore` provides the concrete implementation.

---

## Entities

### 1. BookChunk (Qdrant Vector Database)

A semantic unit of book content stored as a vector embedding for similarity search.

**Collection**: `book_content`

**Schema**:
```json
{
  "id": "uuid",
  "vector": [0.1, 0.2, ...],  // 1536 dimensions (text-embedding-3-small)
  "payload": {
    "text": "string",           // Raw markdown text of the chunk
    "chapter_number": "integer", // Chapter number (1, 2, 3...)
    "chapter_title": "string",   // "Introduction to Physical AI"
    "section_id": "string",      // "physical-ai-definition"
    "section_title": "string",   // "What is Physical AI?"
    "source_url": "string",      // "/docs/part1/chapter1#physical-ai-definition"
    "part_number": "integer",    // Book part (1, 2, 3, 4)
    "token_count": "integer",    // Token count for context management
    "created_at": "datetime"     // ISO timestamp
  }
}
```

**Indexes**:
- Vector index: HNSW for approximate nearest neighbor search
- Payload filter index on `chapter_number`, `part_number`

**Validation Rules**:
- `text` max length: 4000 characters
- `token_count` max: 1000 tokens
- `source_url` must be valid relative URL

---

### 2. Thread (Neon Postgres - ChatKit ThreadStore)

A ChatKit Thread representing a conversation session. Implements ChatKit `Thread` type.

**Table**: `threads`

**Schema**:
```sql
CREATE TABLE threads (
    id VARCHAR(255) PRIMARY KEY,
    metadata JSONB NOT NULL DEFAULT '{}',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    expires_at TIMESTAMP WITH TIME ZONE DEFAULT (NOW() + INTERVAL '24 hours')
);

CREATE INDEX idx_threads_expires ON threads(expires_at);
CREATE INDEX idx_threads_metadata ON threads USING GIN (metadata);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | VARCHAR(255) | PK | Thread identifier (ChatKit generated) |
| metadata | JSONB | NOT NULL, DEFAULT '{}' | Thread context (context_mode, selected_text, session_id) |
| created_at | TIMESTAMP | DEFAULT NOW() | Creation timestamp |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last update timestamp |
| expires_at | TIMESTAMP | DEFAULT NOW() + 24h | Auto-cleanup timestamp |

**Metadata JSONB Structure** (BookAssistantContext):
```json
{
  "session_id": "browser-session-uuid",
  "context_mode": "full_book",
  "selected_text": null
}
```

**Validation Rules**:
- `id` must be valid string (ChatKit generates these)
- `metadata.context_mode` must be one of: 'full_book', 'selected_text'
- `metadata.selected_text` max length: 2000 characters

---

### 3. ThreadItem (Neon Postgres - ChatKit ThreadStore)

Individual items (messages) within a Thread. Implements ChatKit `ThreadItem` type.

**Table**: `thread_items`

**Schema**:
```sql
CREATE TABLE thread_items (
    id VARCHAR(255) PRIMARY KEY,
    thread_id VARCHAR(255) NOT NULL REFERENCES threads(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL,
    content TEXT NOT NULL,
    sources JSONB,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_thread_items_thread ON thread_items(thread_id);
CREATE INDEX idx_thread_items_created ON thread_items(thread_id, created_at);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | VARCHAR(255) | PK | Item identifier (ChatKit generated) |
| thread_id | VARCHAR(255) | FK to threads | Parent thread |
| role | VARCHAR(20) | NOT NULL | 'user' or 'assistant' |
| content | TEXT | NOT NULL | Message content |
| sources | JSONB | NULL | Citation sources (assistant items only) |
| created_at | TIMESTAMP | DEFAULT NOW() | Creation timestamp |

**Sources JSONB Structure**:
```json
[
  {
    "chapter": 1,
    "section": "Introduction to Physical AI",
    "url": "/docs/part1/chapter1#introduction",
    "relevance_score": 0.92
  }
]
```

**Validation Rules**:
- `role` must be one of: 'user', 'assistant'
- `content` max length: 10000 characters
- `sources` optional for assistant items

---

### 4. FeedbackEvent (Neon Postgres)

User satisfaction signals for success criteria measurement.

**Table**: `feedback_events`

**Schema**:
```sql
CREATE TABLE feedback_events (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    thread_item_id VARCHAR(255) NOT NULL REFERENCES thread_items(id) ON DELETE CASCADE,
    session_id VARCHAR(255) NOT NULL,
    feedback_type VARCHAR(20) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_feedback_thread_item ON feedback_events(thread_item_id);
CREATE INDEX idx_feedback_session ON feedback_events(session_id);
CREATE INDEX idx_feedback_type ON feedback_events(feedback_type);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK, auto-generated | Unique feedback identifier |
| thread_item_id | VARCHAR(255) | FK to thread_items | Associated assistant message |
| session_id | VARCHAR(255) | NOT NULL | Session for anonymized tracking |
| feedback_type | VARCHAR(20) | NOT NULL | 'thumbs_up' or 'thumbs_down' |
| created_at | TIMESTAMP | DEFAULT NOW() | Feedback timestamp |

**Validation Rules**:
- `feedback_type` must be one of: 'thumbs_up', 'thumbs_down'
- One feedback per thread_item per session (unique constraint)

**Unique Constraint**:
```sql
CREATE UNIQUE INDEX idx_feedback_unique ON feedback_events(thread_item_id, session_id);
```

---

## State Transitions

### Thread Lifecycle (ChatKit ThreadStore)

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   Created    │────>│   Active     │────>│   Expired    │
└──────────────┘     └──────────────┘     └──────────────┘
     │                     │                     │
     │ create_thread()     │ add_item()          │ 24h timeout
     │ Context mode set    │ Updated timestamp   │ Cleanup job
     v                     v                     v
```

### ThreadItem Flow

```
User Input ──> Sanitization ──> RAG Search ──> LLM Response ──> add_item()
                                    │                               │
                                    v                               v
                              Get Sources                    Store Sources
```

---

## Relationships

1. **Thread → ThreadItems**: One-to-Many (cascade delete)
2. **ThreadItem → FeedbackEvent**: One-to-One (per session)
3. **BookChunk**: Independent (no foreign keys, vector store)

---

## Database Migration

**Migration Order**:
1. Create `threads` table (ChatKit ThreadStore)
2. Create `thread_items` table (depends on threads)
3. Create `feedback_events` table (depends on thread_items)
4. Create Qdrant collection `book_content`

**Initial Migration Script**: `migrations/001_initial_schema.sql`
```sql
-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- Threads table (ChatKit ThreadStore)
CREATE TABLE IF NOT EXISTS threads (
    id VARCHAR(255) PRIMARY KEY,
    metadata JSONB NOT NULL DEFAULT '{}',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    expires_at TIMESTAMP WITH TIME ZONE DEFAULT (NOW() + INTERVAL '24 hours')
);

-- Thread items table (ChatKit ThreadStore)
CREATE TABLE IF NOT EXISTS thread_items (
    id VARCHAR(255) PRIMARY KEY,
    thread_id VARCHAR(255) NOT NULL REFERENCES threads(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    sources JSONB,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Feedback events table
CREATE TABLE IF NOT EXISTS feedback_events (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    thread_item_id VARCHAR(255) NOT NULL REFERENCES thread_items(id) ON DELETE CASCADE,
    session_id VARCHAR(255) NOT NULL,
    feedback_type VARCHAR(20) NOT NULL CHECK (feedback_type IN ('thumbs_up', 'thumbs_down')),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    UNIQUE(thread_item_id, session_id)
);

-- Indexes
CREATE INDEX IF NOT EXISTS idx_threads_expires ON threads(expires_at);
CREATE INDEX IF NOT EXISTS idx_threads_metadata ON threads USING GIN (metadata);
CREATE INDEX IF NOT EXISTS idx_thread_items_thread ON thread_items(thread_id);
CREATE INDEX IF NOT EXISTS idx_thread_items_created ON thread_items(thread_id, created_at);
CREATE INDEX IF NOT EXISTS idx_feedback_thread_item ON feedback_events(thread_item_id);
CREATE INDEX IF NOT EXISTS idx_feedback_session ON feedback_events(session_id);
```

---

## Pydantic Models (Python)

```python
from pydantic import BaseModel, Field
from datetime import datetime
from uuid import UUID
from typing import Optional, Literal, Any

# BookChunk payload for Qdrant
class BookChunkPayload(BaseModel):
    """Payload stored with vector in Qdrant."""
    text: str = Field(max_length=4000)
    chapter_number: int = Field(ge=1)
    chapter_title: str
    section_id: str
    section_title: str
    source_url: str
    part_number: int = Field(ge=1, le=4)
    token_count: int = Field(ge=1, le=1000)
    created_at: datetime = Field(default_factory=datetime.utcnow)

# Source citation
class Source(BaseModel):
    """Citation source for assistant messages."""
    chapter: int
    section: str
    url: str
    relevance_score: float = Field(ge=0, le=1)

# ChatKit ThreadStore types
class BookAssistantContext(BaseModel):
    """Thread metadata for BookAssistant context."""
    session_id: str
    context_mode: Literal["full_book", "selected_text"] = "full_book"
    selected_text: Optional[str] = Field(default=None, max_length=2000)

class ThreadItem(BaseModel):
    """ChatKit ThreadItem for message storage."""
    id: str
    role: Literal["user", "assistant"]
    content: str = Field(max_length=10000)
    sources: Optional[list[Source]] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)

class Thread(BaseModel):
    """ChatKit Thread for conversation storage."""
    id: str
    metadata: BookAssistantContext
    items: list[ThreadItem] = []
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

# Feedback
class FeedbackCreate(BaseModel):
    """Create feedback request."""
    thread_item_id: str
    session_id: str
    feedback_type: Literal["thumbs_up", "thumbs_down"]

class FeedbackResponse(BaseModel):
    """Feedback response."""
    id: UUID
    thread_item_id: str
    feedback_type: str
    created_at: datetime
```

---

## ThreadStore Protocol Implementation

```python
from typing import Protocol, Optional, Any
from abc import abstractmethod

class ThreadStore(Protocol):
    """ChatKit ThreadStore protocol for conversation persistence."""

    @abstractmethod
    async def get_thread(self, thread_id: str) -> Optional[Thread]:
        """Get thread by ID with all items."""
        ...

    @abstractmethod
    async def create_thread(self, metadata: dict[str, Any] = None) -> Thread:
        """Create new thread with optional metadata."""
        ...

    @abstractmethod
    async def add_item(self, thread_id: str, item: ThreadItem) -> None:
        """Add item to thread."""
        ...

    @abstractmethod
    async def delete_thread(self, thread_id: str) -> None:
        """Delete thread and all items."""
        ...
```

**PostgresThreadStore Implementation**: See `backend/app/postgres_thread_store.py`
