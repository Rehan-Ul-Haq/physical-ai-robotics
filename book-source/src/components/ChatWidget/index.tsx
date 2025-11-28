/**
 * ChatWidget Component for Book Assistant
 *
 * A floating chat interface that allows readers to ask questions about
 * the Physical AI & Humanoid Robotics book content. Integrates with
 * the RAG-powered backend API.
 */

import React, { useState, useRef, useEffect, FormEvent } from 'react';
import styles from './styles.module.css';

// Get API URL from environment or use default
const API_URL = typeof window !== 'undefined'
  ? (window as any).__BOOK_ASSISTANT_API_URL__ || 'http://localhost:8001'
  : 'http://localhost:8001';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  isStreaming?: boolean;
}

interface PageContext {
  current_chapter: number | null;
  current_lesson: string | null;
}

interface ChatKitContext {
  session_id?: string;
  context_mode?: 'full_book' | 'selected_text';
  selected_text?: string;
  page_context?: PageContext;
}

interface ChatKitRequest {
  query: string;
  thread_id?: string;
  context?: ChatKitContext;
}

// ChatKit SSE event types
interface ChatKitEvent {
  type: string;
  thread?: { id: string };
  update?: {
    type: string;
    delta?: string;
  };
}

// Stream result type
interface StreamResult {
  text?: string;
  threadId?: string;
}

// Generate session ID
function generateSessionId(): string {
  const array = new Uint8Array(16);
  crypto.getRandomValues(array);
  return Array.from(array, (byte) => byte.toString(16).padStart(2, '0')).join('');
}

// Get or create session ID
function getSessionId(): string {
  if (typeof window === 'undefined') return '';
  const SESSION_KEY = 'book-assistant-session-id';
  let sessionId = localStorage.getItem(SESSION_KEY);
  if (!sessionId) {
    sessionId = generateSessionId();
    localStorage.setItem(SESSION_KEY, sessionId);
  }
  return sessionId;
}

/**
 * Extract current chapter number from Docusaurus URL
 */
function getCurrentChapter(): number | null {
  if (typeof window === 'undefined') return null;
  const pathname = window.location.pathname;
  const match = pathname.match(/\/docs\/(\d{2})-/);
  if (match) {
    const chapterNum = parseInt(match[1], 10);
    return chapterNum >= 1 && chapterNum <= 14 ? chapterNum : null;
  }
  return null;
}

/**
 * Extract current lesson slug from Docusaurus URL
 */
function getCurrentLesson(): string | null {
  if (typeof window === 'undefined') return null;
  const pathname = window.location.pathname;
  const parts = pathname.split('/').filter(Boolean);
  if (parts.length >= 3 && parts[0] === 'docs') {
    const lessonSlug = parts[2];
    if (/^\d{2}-/.test(lessonSlug)) {
      return lessonSlug;
    }
  }
  return null;
}

/**
 * Get current page context from URL
 */
function getPageContext(): PageContext {
  return {
    current_chapter: getCurrentChapter(),
    current_lesson: getCurrentLesson(),
  };
}


// Stream chat API - handles ChatKit protocol SSE events
async function* streamChat(request: ChatKitRequest): AsyncGenerator<StreamResult, void, unknown> {
  const response = await fetch(`${API_URL}/assistant/chatkit`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    throw new Error(`HTTP error! status: ${response.status}`);
  }

  if (!response.body) {
    throw new Error('No response body');
  }

  const reader = response.body.getReader();
  const decoder = new TextDecoder();
  let buffer = '';

  try {
    while (true) {
      const { done, value } = await reader.read();
      if (done) break;

      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split('\n');
      buffer = lines.pop() || '';

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          const data = line.slice(6);
          if (data === '[DONE]') return;
          try {
            const parsed: ChatKitEvent = JSON.parse(data);

            // Handle thread.created - extract thread ID
            if (parsed.type === 'thread.created' && parsed.thread?.id) {
              yield { threadId: parsed.thread.id };
            }
            // Handle text deltas from ChatKit protocol
            else if (
              parsed.type === 'thread.item.updated' &&
              parsed.update?.type === 'assistant_message.content_part.text_delta' &&
              parsed.update.delta
            ) {
              yield { text: parsed.update.delta };
            }
          } catch {
            // Ignore parse errors for non-JSON lines
          }
        }
      }
    }
  } finally {
    reader.releaseLock();
  }
}

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [threadId, setThreadId] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [sessionId, setSessionId] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Initialize session ID
  useEffect(() => {
    setSessionId(getSessionId());
  }, []);

  // Auto-scroll to bottom
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Listen for text selection
  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim().length > 10) {
        const text = selection.toString().trim();
        // Limit to 2000 chars
        setSelectedText(text.slice(0, 2000));
      }
    };

    // Only listen when chat is open
    if (isOpen) {
      document.addEventListener('mouseup', handleMouseUp);
      return () => document.removeEventListener('mouseup', handleMouseUp);
    }
  }, [isOpen]);

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    if (!input.trim() || isLoading) return;

    const userMessage: Message = {
      id: `user-${Date.now()}`,
      role: 'user',
      content: input.trim(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);
    setError(null);

    // Get current page context from URL
    const pageContext = getPageContext();

    // Build context
    const context: ChatKitContext = {
      session_id: sessionId,
      context_mode: selectedText ? 'selected_text' : 'full_book',
      page_context: pageContext,
    };

    if (selectedText) {
      context.selected_text = selectedText;
    }

    // Add assistant message placeholder
    const assistantMessageId = `assistant-${Date.now()}`;
    const assistantMessage: Message = {
      id: assistantMessageId,
      role: 'assistant',
      content: '',
      isStreaming: true,
    };
    setMessages((prev) => [...prev, assistantMessage]);

    try {
      let fullResponse = '';

      for await (const result of streamChat({
        query: userMessage.content,
        thread_id: threadId || undefined,
        context,
      })) {
        // Handle thread ID from stream
        if (result.threadId && !threadId) {
          setThreadId(result.threadId);
        }
        // Handle text content
        if (result.text) {
          fullResponse += result.text;
          setMessages((prev) =>
            prev.map((msg) =>
              msg.id === assistantMessageId
                ? { ...msg, content: fullResponse }
                : msg
            )
          );
        }
      }

      // Mark streaming as complete
      setMessages((prev) =>
        prev.map((msg) =>
          msg.id === assistantMessageId
            ? { ...msg, isStreaming: false }
            : msg
        )
      );

      // Clear selected text after successful query
      setSelectedText(null);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
      setMessages((prev) => prev.filter((msg) => msg.id !== assistantMessageId));
    } finally {
      setIsLoading(false);
    }
  };

  const handleNewConversation = () => {
    setMessages([]);
    setThreadId(null);
    setError(null);
    setSelectedText(null);
  };

  const clearSelection = () => {
    setSelectedText(null);
    window.getSelection()?.removeAllRanges();
  };

  // Closed state - floating button
  if (!isOpen) {
    return (
      <button
        onClick={() => setIsOpen(true)}
        className={styles.floatingButton}
        aria-label="Open chat assistant"
        title="Ask about the book"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
      </button>
    );
  }

  // Open state - chat window
  return (
    <div className={styles.chatWindow}>
      {/* Header */}
      <div className={styles.header}>
        <div className={styles.headerInfo}>
          <h3 className={styles.headerTitle}>Book Assistant</h3>
          <p className={styles.headerSubtitle}>Ask questions about the book</p>
        </div>
        <div className={styles.headerActions}>
          <button
            onClick={handleNewConversation}
            className={styles.iconButton}
            title="New conversation"
          >
            <svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M3 12a9 9 0 0 1 9-9 9.75 9.75 0 0 1 6.74 2.74L21 8" />
              <path d="M21 3v5h-5" />
              <path d="M21 12a9 9 0 0 1-9 9 9.75 9.75 0 0 1-6.74-2.74L3 16" />
              <path d="M8 16H3v5" />
            </svg>
          </button>
          <button
            onClick={() => setIsOpen(false)}
            className={styles.iconButton}
            title="Close"
          >
            <svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="18" y1="6" x2="6" y2="18" />
              <line x1="6" y1="6" x2="18" y2="18" />
            </svg>
          </button>
        </div>
      </div>

      {/* Selected text indicator */}
      {selectedText && (
        <div className={styles.selectedTextBanner}>
          <span className={styles.selectedTextLabel}>
            Context: "{selectedText.slice(0, 50)}..."
          </span>
          <button onClick={clearSelection} className={styles.clearButton}>
            Clear
          </button>
        </div>
      )}

      {/* Messages */}
      <div className={styles.messages}>
        {messages.length === 0 && (
          <div className={styles.emptyState}>
            <p className={styles.emptyTitle}>Welcome to the Book Assistant!</p>
            <p className={styles.emptySubtitle}>
              Ask any question about Physical AI & Humanoid Robotics.
              <br />
              <small>Tip: Select text on the page to ask about specific content.</small>
            </p>
          </div>
        )}

        {messages.map((message) => (
          <div
            key={message.id}
            className={`${styles.message} ${
              message.role === 'user' ? styles.userMessage : styles.assistantMessage
            }`}
          >
            <div className={styles.messageContent}>
              {message.content}
              {message.isStreaming && <span className={styles.cursor} />}
            </div>
          </div>
        ))}

        {error && (
          <div className={styles.errorMessage}>
            {error}
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <form onSubmit={handleSubmit} className={styles.inputForm}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder="Ask a question..."
          disabled={isLoading}
          className={styles.input}
        />
        <button
          type="submit"
          disabled={isLoading || !input.trim()}
          className={styles.sendButton}
        >
          {isLoading ? (
            <svg className={styles.spinner} xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <circle cx="12" cy="12" r="10" opacity="0.25" />
              <path d="M12 2a10 10 0 0 1 10 10" />
            </svg>
          ) : (
            <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="22" y1="2" x2="11" y2="13" />
              <polygon points="22 2 15 22 11 13 2 9 22 2" />
            </svg>
          )}
        </button>
      </form>
    </div>
  );
}
