/**
 * ChatWidget Component for Book Assistant
 *
 * A floating chat interface that allows readers to ask questions about
 * the Physical AI & Humanoid Robotics book content. Integrates with
 * the RAG-powered backend API.
 * 
 * Authentication: Shows sign-in prompt for unauthenticated users.
 */

import React, { useState, useRef, useEffect, FormEvent } from 'react';
import ReactMarkdown from 'react-markdown';
import { useSession } from '@/lib/auth';
import { SignInModal, SignUpModal, PasswordResetRequest } from '@/components/Auth';
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
  contextText?: string;  // Store the selected text context with the message
}

interface PageContext {
  current_chapter: number | null;
  current_lesson: string | null;
  part_slug?: string;
  chapter_slug?: string;
  lesson_slug?: string;
  current_path?: string;
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
 * Extract page context from Docusaurus URL structure.
 *
 * URL pattern: /physical-ai-robotics/docs/{part-slug}/{chapter-slug}/{lesson-slug}
 * Example: /physical-ai-robotics/docs/robotic-nervous-system/introduction-to-physical-ai/reality-gap
 */

// Map part slugs to part numbers
const PART_SLUG_MAP: Record<string, number> = {
  'robotic-nervous-system': 1,
  'digital-twin': 2,
  'ai-robot-brain': 3,
  'vision-language-action': 4,
};

// Map known chapter slugs to chapter numbers
const CHAPTER_SLUG_MAP: Record<string, number> = {
  'introduction-to-physical-ai': 1,
  'ros2-fundamentals': 2,
  'ros2-communication-patterns': 3,
  'ros2-advanced-concepts': 4,
  'building-your-first-robot-application': 5,
};

function getPageContext(): PageContext {
  if (typeof window === 'undefined') {
    return { current_chapter: null, current_lesson: null };
  }

  const pathname = window.location.pathname;

  // Remove baseUrl prefix if present
  const baseUrl = '/physical-ai-robotics';
  const path = pathname.startsWith(baseUrl)
    ? pathname.slice(baseUrl.length)
    : pathname;

  // Split: ['docs', 'part-slug', 'chapter-slug', 'lesson-slug']
  const parts = path.split('/').filter(Boolean);

  // Must start with 'docs'
  if (parts.length < 2 || parts[0] !== 'docs') {
    return { current_chapter: null, current_lesson: null };
  }

  const partSlug = parts[1] || null;      // e.g., 'robotic-nervous-system'
  const chapterSlug = parts[2] || null;   // e.g., 'introduction-to-physical-ai'
  const lessonSlug = parts[3] || null;    // e.g., 'reality-gap'

  // Determine chapter number from slug
  let chapterNum: number | null = null;
  if (chapterSlug) {
    // Try direct mapping first
    chapterNum = CHAPTER_SLUG_MAP[chapterSlug] || null;

    // Fallback: try to extract from XX- prefix if present in slug
    if (!chapterNum) {
      const match = chapterSlug.match(/^(\d{2})-/);
      if (match) {
        chapterNum = parseInt(match[1], 10);
      }
    }
  }

  return {
    current_chapter: chapterNum,
    current_lesson: lessonSlug,
    part_slug: partSlug || undefined,
    chapter_slug: chapterSlug || undefined,
    lesson_slug: lessonSlug || undefined,
    current_path: path,
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
  // Check authentication - show sign-in prompt for unauthenticated users
  const { data: session, isPending: isSessionPending } = useSession();
  
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [threadId, setThreadId] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [sessionId, setSessionId] = useState('');
  const [showFullContext, setShowFullContext] = useState(false);
  const [isMaximized, setIsMaximized] = useState(false);
  const [authModal, setAuthModal] = useState<'none' | 'signIn' | 'signUp' | 'forgotPassword'>('none');
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const chatWindowRef = useRef<HTMLDivElement>(null);

  // Initialize session ID
  useEffect(() => {
    setSessionId(getSessionId());
  }, []);

  // Auto-scroll to bottom
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Listen for text selection (exclude selections inside the chat window)
  useEffect(() => {
    const handleMouseUp = (event: MouseEvent) => {
      // Ignore if selection is inside the chat window
      if (chatWindowRef.current && chatWindowRef.current.contains(event.target as Node)) {
        return;
      }

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

    // Store the selected text with this message
    const currentSelectedText = selectedText;

    const userMessage: Message = {
      id: `user-${Date.now()}`,
      role: 'user',
      content: input.trim(),
      contextText: currentSelectedText || undefined,
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
      context_mode: currentSelectedText ? 'selected_text' : 'full_book',
      page_context: pageContext,
    };

    if (currentSelectedText) {
      context.selected_text = currentSelectedText;
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

  // Auth modal handlers
  const openSignIn = () => setAuthModal('signIn');
  const openSignUp = () => setAuthModal('signUp');
  const openForgotPassword = () => setAuthModal('forgotPassword');
  const closeAuthModal = () => setAuthModal('none');

  // Closed state - floating button (always visible)
  if (!isOpen) {
    return (
      <button
        onClick={() => setIsOpen(true)}
        className={styles.floatingButton}
        aria-label="Open chat assistant"
        title="Ask about the book"
      >
        {/* Robot Eye */}
        <div className={styles.eyeContainer}>
          <div className={styles.eyeBall} />
          <div className={styles.eyelidTop} />
          <div className={styles.eyelidBottom} />
          <div className={styles.sleepIndicator} />
        </div>
        <span className={styles.floatingButtonLabel}>Robo AI</span>
      </button>
    );
  }

  // Open state - show sign-in prompt for unauthenticated users
  if (!session && !isSessionPending) {
    return (
      <div ref={chatWindowRef} className={styles.chatWindow}>
        {/* Header */}
        <div className={styles.header}>
          <div className={styles.headerBrand}>
            <div className={styles.headerIcon}>
              <div className={styles.headerEye} />
            </div>
            <div className={styles.headerInfo}>
              <h3 className={styles.headerTitle}>Robo AI</h3>
              <p className={styles.headerSubtitle}>Your Physical AI & Robotics Guide</p>
            </div>
          </div>
          <div className={styles.headerActions}>
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

        {/* Sign-in prompt with robot eye */}
        <div className={styles.messages}>
          <div className={styles.emptyState}>
            <div className={styles.emptyIcon}>
              <div className={styles.emptyRobotEye} />
            </div>
            <p className={styles.emptyTitle}>Welcome to Robo AI!</p>
            <p className={styles.emptySubtitle}>
              Sign in to chat and get personalized explanations on Physical AI & Robotics.
            </p>
            <button
              type="button"
              className={styles.authPromptBtn}
              onClick={openSignIn}
            >
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" style={{ marginRight: '8px' }}>
                <rect x="3" y="11" width="18" height="11" rx="2" ry="2" />
                <path d="M7 11V7a5 5 0 0 1 10 0v4" />
              </svg>
              Sign In to Chat
            </button>
          </div>
        </div>

        {/* Disabled input area for visual context */}
        <div className={styles.inputForm} style={{ opacity: 0.5, pointerEvents: 'none' }}>
          <input
            type="text"
            placeholder="Sign in to ask questions..."
            disabled
            className={styles.input}
          />
          <button
            type="button"
            disabled
            className={styles.sendButton}
          >
            <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="22" y1="2" x2="11" y2="13" />
              <polygon points="22 2 15 22 11 13 2 9 22 2" />
            </svg>
          </button>
        </div>

        {/* Auth Modals */}
        <SignInModal
          isOpen={authModal === 'signIn'}
          onClose={closeAuthModal}
          onSwitchToSignUp={openSignUp}
          onForgotPassword={openForgotPassword}
        />
        <SignUpModal
          isOpen={authModal === 'signUp'}
          onClose={closeAuthModal}
          onSwitchToSignIn={openSignIn}
        />
        <PasswordResetRequest
          isOpen={authModal === 'forgotPassword'}
          onClose={closeAuthModal}
          onBackToSignIn={openSignIn}
        />
      </div>
    );
  }

  // Loading state while checking session
  if (isSessionPending) {
    return (
      <div ref={chatWindowRef} className={styles.chatWindow}>
        <div className={styles.header}>
          <div className={styles.headerBrand}>
            <div className={styles.headerIcon}>
              <div className={styles.headerEye} />
            </div>
            <div className={styles.headerInfo}>
              <h3 className={styles.headerTitle}>Robo AI</h3>
              <p className={styles.headerSubtitle}>Loading...</p>
            </div>
          </div>
          <div className={styles.headerActions}>
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
        <div className={styles.messages}>
          <div className={styles.emptyState}>
            <div className={styles.emptyIcon}>
              <div className={styles.emptyRobotEye} />
            </div>
            <p className={styles.emptyTitle}>Loading...</p>
          </div>
        </div>
      </div>
    );
  }

  // Open state - authenticated user - full chat window
  return (
    <div ref={chatWindowRef} className={`${styles.chatWindow} ${isMaximized ? styles.maximized : ''}`}>
      {/* Header */}
      <div className={styles.header}>
        <div className={styles.headerBrand}>
          {/* Robot Eye Icon */}
          <div className={styles.headerIcon}>
            <div className={styles.headerEye} />
          </div>
          <div className={styles.headerInfo}>
            <h3 className={styles.headerTitle}>Robo AI</h3>
            <p className={styles.headerSubtitle}>Your Physical AI & Robotics Guide</p>
          </div>
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
            onClick={() => setIsMaximized(!isMaximized)}
            className={styles.iconButton}
            title={isMaximized ? "Restore" : "Maximize"}
          >
            {isMaximized ? (
              <svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M8 3v3a2 2 0 0 1-2 2H3m18 0h-3a2 2 0 0 1-2-2V3m0 18v-3a2 2 0 0 1 2-2h3M3 16h3a2 2 0 0 1 2 2v3" />
              </svg>
            ) : (
              <svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M8 3H5a2 2 0 0 0-2 2v3m18 0V5a2 2 0 0 0-2-2h-3m0 18h3a2 2 0 0 0 2-2v-3M3 16v3a2 2 0 0 0 2 2h3" />
              </svg>
            )}
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

      {/* Selected text chip - only show when there's actual content */}
      {selectedText && selectedText.trim() && (
        <div className={styles.contextArea}>
          <div className={styles.selectedTextChip}>
            <span className={styles.chipIcon}>
              <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z" />
                <polyline points="14 2 14 8 20 8" />
                <line x1="16" y1="13" x2="8" y2="13" />
                <line x1="16" y1="17" x2="8" y2="17" />
              </svg>
            </span>
            <span
              className={styles.chipText}
              onClick={() => setShowFullContext(true)}
              title="Click to expand"
            >
              "{selectedText.length > 60 ? selectedText.slice(0, 60) + '...' : selectedText}"
            </span>
            <button
              onClick={clearSelection}
              className={styles.chipClearBtn}
              title="Remove context"
            >
              <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <line x1="18" y1="6" x2="6" y2="18" />
                <line x1="6" y1="6" x2="18" y2="18" />
              </svg>
            </button>
          </div>
        </div>
      )}

      {/* Context modal */}
      {showFullContext && selectedText && (
        <div className={styles.contextModal} onClick={() => setShowFullContext(false)}>
          <div className={styles.contextModalContent} onClick={(e) => e.stopPropagation()}>
            <h4 className={styles.contextModalTitle}>Selected Text</h4>
            <p className={styles.contextModalText}>{selectedText}</p>
            <button
              onClick={() => setShowFullContext(false)}
              className={styles.contextModalClose}
            >
              Close
            </button>
          </div>
        </div>
      )}

      {/* Messages */}
      <div className={styles.messages}>
        {messages.length === 0 && (
          <div className={styles.emptyState}>
            <div className={styles.emptyIcon}>
              <div className={styles.emptyRobotEye} />
            </div>
            <p className={styles.emptyTitle}>Welcome to Robo AI!</p>
            <p className={styles.emptySubtitle}>
              I'm your intelligent guide to Physical AI & Humanoid Robotics.
              <br />
              <small>💡 Tip: Select text on the page to ask about specific content.</small>
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
              {/* Show combined view for user messages with selected text */}
              {message.role === 'user' && message.contextText ? (
                <div className={styles.messageWithContext}>
                  <div className={styles.selectedTextSection}>
                    <div className={styles.selectedTextLabel}>Selected from book:</div>
                    <div className={styles.selectedTextContent}>
                      {message.contextText}
                    </div>
                  </div>
                  <div className={styles.questionSection}>
                    <div className={styles.questionLabel}>Question:</div>
                    <div className={styles.questionContent}>
                      {message.content}
                    </div>
                  </div>
                </div>
              ) : message.role === 'assistant' ? (
                <div className={styles.markdownContent}>
                  <ReactMarkdown>{message.content}</ReactMarkdown>
                  {message.isStreaming && <span className={styles.cursor} />}
                </div>
              ) : (
                <>
                  {message.content}
                  {message.isStreaming && <span className={styles.cursor} />}
                </>
              )}
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
          placeholder={selectedText ? "Ask about selected text..." : "Ask a question..."}
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
