# Frontend ChatKit Widget Integration

This reference documents patterns for integrating a ChatKit widget with a Docusaurus or React-based documentation site.

## React Component Structure

```typescript
interface PageContext {
  current_chapter: number | null;
  current_lesson: string | null;
}

interface ChatWidgetState {
  messages: Message[];
  threadId: string | null;
  isLoading: boolean;
  selectedText: string | null;
  pageContext: PageContext;
}
```

## Extracting Page Context from URL

For Docusaurus sites with URL pattern `/docs/{part}/{chapter}/{lesson}`:

```typescript
function getPageContext(): PageContext {
  const path = window.location.pathname;
  const parts = path.split('/').filter(Boolean);

  // Pattern: /docs/robotic-nervous-system/introduction-to-physical-ai/reality-gap
  // parts = ['docs', 'robotic-nervous-system', 'introduction-to-physical-ai', 'reality-gap']

  let current_chapter: number | null = null;
  let current_lesson: string | null = null;

  if (parts.length >= 3) {
    // Extract chapter number from chapter slug (e.g., "01-introduction" -> 1)
    const chapterSlug = parts[2];
    const chapterMatch = chapterSlug.match(/^(\d+)-/);
    if (chapterMatch) {
      current_chapter = parseInt(chapterMatch[1], 10);
    }

    // Lesson is the last segment if it exists
    if (parts.length >= 4) {
      current_lesson = parts[3];
    }
  }

  return { current_chapter, current_lesson };
}
```

## Text Selection Capture (Excluding Chat Window)

```typescript
const chatWindowRef = useRef<HTMLDivElement>(null);

useEffect(() => {
  const handleMouseUp = (event: MouseEvent) => {
    // CRITICAL: Ignore selections inside chat window
    if (chatWindowRef.current && chatWindowRef.current.contains(event.target as Node)) {
      return;
    }

    const selection = window.getSelection();
    if (selection && selection.toString().trim().length > 10) {
      const text = selection.toString().trim();
      setSelectedText(text.slice(0, 2000)); // Limit to prevent oversized requests
    }
  };

  document.addEventListener('mouseup', handleMouseUp);
  return () => document.removeEventListener('mouseup', handleMouseUp);
}, []);

// In JSX:
<div ref={chatWindowRef} className={styles.chatWindow}>
  {/* Chat content */}
</div>
```

## SSE Stream Parsing

```typescript
async function sendMessage(message: string) {
  setIsLoading(true);

  const response = await fetch('/assistant/chatkit', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      query: message,
      thread_id: threadId,
      context: {
        page_context: getPageContext(),
        selected_text: selectedText,
        context_mode: selectedText ? 'selected_text' : 'full_book',
      },
    }),
  });

  if (!response.ok) {
    throw new Error(`HTTP ${response.status}`);
  }

  const reader = response.body!.getReader();
  const decoder = new TextDecoder();
  let buffer = '';
  let currentContent = '';
  let currentItemId: string | null = null;
  let newThreadId: string | null = null;

  while (true) {
    const { done, value } = await reader.read();
    if (done) break;

    buffer += decoder.decode(value, { stream: true });
    const lines = buffer.split('\n');
    buffer = lines.pop() || ''; // Keep incomplete line in buffer

    for (const line of lines) {
      if (line.startsWith('event: ')) {
        // Could use event type for routing
      } else if (line.startsWith('data: ')) {
        try {
          const data = JSON.parse(line.slice(6));
          handleSSEEvent(data);
        } catch (e) {
          // Ignore parse errors for incomplete data
        }
      }
    }
  }

  setIsLoading(false);
}

function handleSSEEvent(data: any) {
  // Thread created
  if (data.id?.startsWith('thread_') && !threadId) {
    setThreadId(data.id);
  }

  // Item delta (streaming text)
  if (data.delta?.type === 'output_text') {
    setMessages(prev => {
      const last = prev[prev.length - 1];
      if (last?.role === 'assistant') {
        return [
          ...prev.slice(0, -1),
          { ...last, content: last.content + data.delta.text },
        ];
      }
      return prev;
    });
  }

  // Item created (new assistant message)
  if (data.role === 'assistant' && data.content !== undefined) {
    setMessages(prev => [
      ...prev,
      { id: data.id, role: 'assistant', content: '' },
    ]);
  }
}
```

## CSS for Chat Widget

```css
.chatWindow {
  position: fixed;
  bottom: 80px;
  right: 20px;
  width: 400px;
  max-height: 600px;
  background: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 12px;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15);
  display: flex;
  flex-direction: column;
  z-index: 1000;
}

.messageContainer {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
}

.inputContainer {
  padding: 12px;
  border-top: 1px solid var(--ifm-color-emphasis-200);
}

/* Streaming indicator */
.streaming::after {
  content: '▋';
  animation: blink 1s infinite;
}

@keyframes blink {
  50% { opacity: 0; }
}
```

## Selected Text Context Display

When user has selected text, show it in the chat UI:

```typescript
{selectedText && (
  <div className={styles.selectedTextBanner}>
    <span>Selected: </span>
    <em>"{selectedText.slice(0, 100)}..."</em>
    <button onClick={() => setSelectedText(null)}>×</button>
  </div>
)}
```

## Error Handling

```typescript
async function sendMessage(message: string) {
  try {
    // ... send logic
  } catch (error) {
    if (error instanceof TypeError && error.message.includes('fetch')) {
      setError('Unable to connect to server');
    } else if (error instanceof Error) {
      setError(error.message);
    }
  } finally {
    setIsLoading(false);
  }
}
```

## Markdown Rendering for Responses

```typescript
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';

function AssistantMessage({ content }: { content: string }) {
  return (
    <div className={styles.assistantMessage}>
      <ReactMarkdown remarkPlugins={[remarkGfm]}>
        {content}
      </ReactMarkdown>
    </div>
  );
}
```

## CORS Configuration

Backend must allow frontend origins:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",   # Docusaurus dev
        "http://127.0.0.1:3000",
        "https://yourdomain.com",  # Production
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```
