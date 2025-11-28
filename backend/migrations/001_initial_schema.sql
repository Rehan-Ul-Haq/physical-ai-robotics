-- Migration: 001_initial_schema
-- Description: ChatKit ThreadStore schema for Book Assistant
-- Date: 2025-11-28

-- Threads table (ChatKit Thread storage)
CREATE TABLE IF NOT EXISTS threads (
    id VARCHAR(255) PRIMARY KEY,
    title VARCHAR(500),
    metadata JSONB DEFAULT '{}',
    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW()
);

-- Thread items table (ChatKit ThreadItem storage)
CREATE TABLE IF NOT EXISTS thread_items (
    id VARCHAR(255) PRIMARY KEY,
    thread_id VARCHAR(255) NOT NULL REFERENCES threads(id) ON DELETE CASCADE,
    type VARCHAR(50) NOT NULL, -- 'message', 'tool_call', 'hidden_context', etc.
    role VARCHAR(20), -- 'user', 'assistant', 'system'
    content TEXT,
    sources JSONB, -- Array of source citations
    metadata JSONB DEFAULT '{}',
    created_at TIMESTAMPTZ DEFAULT NOW()
);

-- Feedback events table (user satisfaction signals)
CREATE TABLE IF NOT EXISTS feedback_events (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    thread_item_id VARCHAR(255) NOT NULL REFERENCES thread_items(id) ON DELETE CASCADE,
    session_id VARCHAR(255) NOT NULL,
    feedback_type VARCHAR(20) NOT NULL CHECK (feedback_type IN ('thumbs_up', 'thumbs_down')),
    created_at TIMESTAMPTZ DEFAULT NOW(),
    UNIQUE(thread_item_id, session_id)
);

-- Indexes for performance
CREATE INDEX IF NOT EXISTS idx_thread_items_thread_id ON thread_items(thread_id);
CREATE INDEX IF NOT EXISTS idx_thread_items_created_at ON thread_items(created_at);
CREATE INDEX IF NOT EXISTS idx_feedback_events_session_id ON feedback_events(session_id);
CREATE INDEX IF NOT EXISTS idx_threads_updated_at ON threads(updated_at);
