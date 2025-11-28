# Book Assistant Fix Plan

## Analysis Summary

### Issue 1: Wrong Lesson Context Being Returned
**Problem**: User was on "The Reality Gap" lesson (`/physical-ai-robotics/docs/robotic-nervous-system/introduction-to-physical-ai/reality-gap`) but assistant returned chapter-level content instead of lesson-specific content.

**Root Cause Analysis**:
1. **Frontend URL Parsing is Wrong**: The `getCurrentChapter()` and `getCurrentLesson()` functions use incorrect regex patterns:
   - `getCurrentChapter()` expects `/docs/(\d{2})-` but actual URL is `/docs/robotic-nervous-system/...`
   - `getCurrentLesson()` expects `parts[2]` to match `/^\d{2}-/` but URL structure is different

2. **Actual URL Structure**: `http://localhost:3000/physical-ai-robotics/docs/robotic-nervous-system/introduction-to-physical-ai/reality-gap`
   - `parts[0]` = `physical-ai-robotics`
   - `parts[1]` = `docs`
   - `parts[2]` = `robotic-nervous-system` (part slug, not chapter)
   - `parts[3]` = `introduction-to-physical-ai` (chapter slug)
   - `parts[4]` = `reality-gap` (lesson slug)

3. **Qdrant Data Structure Mismatch**: Indexed data uses `section_id` like `the-perfect-simulation-problem` but frontend would send `reality-gap`. The search tool can't match lesson context.

---

### Issue 2: example.com URLs Instead of Proper Links
**Problem**: Response shows `[Chapter Structure](https://example.com/docs/part1/chapter1#chapter-structure)` instead of proper URLs.

**Root Cause Analysis**:
1. **Agent Generates Placeholder URLs**: The LLM is generating `example.com` as placeholder in the response markdown
2. **Indexed URLs Are Relative**: Qdrant stores `/docs/part1/chapter1#section-id` (wrong path format)
3. **No Base URL Configuration**: Backend doesn't know the frontend's base URL to construct absolute URLs
4. **Agent Instructions Don't Specify URL Format**: The agent should use the `source_url` from search results, not fabricate URLs

---

### Issue 3: Poor UX for Selected Text Context Display
**Problem**: Without selected text, chatbox shows "Context: ..." which is confusing. Selected text UX needs improvement.

**Root Cause Analysis**:
1. **Banner shows for any truthy selectedText**: Even empty states might trigger display
2. **No way to preview/edit context before sending**: User can't see full context or modify it
3. **Selected text disappears after query**: User loses context of what they asked about

---

## Proposed Fixes

### Fix 1: Correct URL Parsing in Frontend

**File**: `book-source/src/components/ChatWidget/index.tsx`

**Changes**:
```typescript
// NEW: Extract page info from Docusaurus URL structure
// URL: /physical-ai-robotics/docs/{part-slug}/{chapter-slug}/{lesson-slug}
// Example: /physical-ai-robotics/docs/robotic-nervous-system/introduction-to-physical-ai/reality-gap

function getPageContext(): PageContext {
  if (typeof window === 'undefined') return { current_chapter: null, current_lesson: null };

  const pathname = window.location.pathname;
  // Remove baseUrl prefix if present
  const baseUrl = '/physical-ai-robotics';
  const path = pathname.startsWith(baseUrl) ? pathname.slice(baseUrl.length) : pathname;

  // Split: ['', 'docs', 'part-slug', 'chapter-slug', 'lesson-slug']
  const parts = path.split('/').filter(Boolean);

  if (parts.length >= 2 && parts[0] === 'docs') {
    // Map part slugs to numbers
    const partSlugMap: Record<string, number> = {
      'robotic-nervous-system': 1,
      'digital-twin': 2,
      'ai-robot-brain': 3,
      'vision-language-action': 4,
    };

    const partSlug = parts[1];
    const chapterSlug = parts[2] || null;  // e.g., 'introduction-to-physical-ai'
    const lessonSlug = parts[3] || null;   // e.g., 'reality-gap'

    // Extract chapter number from first two chars of chapter slug if numeric
    // e.g., '01-introduction-to-physical-ai' -> 1
    let chapterNum: number | null = null;
    if (chapterSlug) {
      const match = chapterSlug.match(/^(\d{2})-/);
      if (match) {
        chapterNum = parseInt(match[1], 10);
      } else {
        // Fallback: map known chapter slugs
        const chapterMap: Record<string, number> = {
          'introduction-to-physical-ai': 1,
        };
        chapterNum = chapterMap[chapterSlug] || null;
      }
    }

    return {
      current_chapter: chapterNum,
      current_lesson: lessonSlug,  // Send lesson slug directly
      // NEW: Also send full path for better context
      current_path: path,
    };
  }

  return { current_chapter: null, current_lesson: null };
}
```

---

### Fix 2: Correct URL Generation in Indexing + Backend Config

**Part A: Update Index Script** (`backend/scripts/index_book.py`)

Store proper URLs that match Docusaurus routing:
```python
# Change from:
"source_url": f"/docs/part{part_num}/chapter{chapter_num}#{chunk['section_id']}",

# Change to (matching actual Docusaurus URL structure):
"source_url": f"/docs/{part_slug}/{chapter_slug}/{lesson_slug}#{chunk['section_id']}",
# Example: /docs/robotic-nervous-system/introduction-to-physical-ai/reality-gap#the-perfect-simulation-problem
```

**Part B: Add Base URL Config** (`backend/app/config.py`)

```python
# Add to settings
book_base_url: str = "http://localhost:3000/physical-ai-robotics"  # or from env
```

**Part C: Update Agent Instructions** (`backend/app/book_assistant_agent.py`)

Update instructions to use actual URLs from search results:
```
## How to Cite Sources

When citing sources, use the EXACT URLs provided in the search results.
Do NOT fabricate or guess URLs. Format: [Section Name](URL_FROM_SEARCH_RESULT)

Example:
- Search returns: {"section": "What is Physical AI?", "url": "/docs/robotic-nervous-system/introduction-to-physical-ai/paradigm-shift#what-is-physical-ai"}
- Cite as: [What is Physical AI?](/docs/robotic-nervous-system/introduction-to-physical-ai/paradigm-shift#what-is-physical-ai)
```

**Part D: Re-index with Correct URLs**

Re-run indexing after fixing URL format in index script.

---

### Fix 3: Improve Selected Text UX in Frontend

**File**: `book-source/src/components/ChatWidget/index.tsx`

**Changes**:

1. **Only show context banner when there's actual selected text** (not empty string)
2. **Add selected text as a "chip" that can be attached to messages**
3. **Show selected text in a collapsible/expandable preview**
4. **Keep selected text visible in the conversation for context**

```typescript
// NEW: SelectedTextChip component
interface SelectedTextChipProps {
  text: string;
  onClear: () => void;
  onExpand: () => void;
}

function SelectedTextChip({ text, onClear, onExpand }: SelectedTextChipProps) {
  const preview = text.length > 100 ? text.slice(0, 100) + '...' : text;
  return (
    <div className={styles.selectedTextChip}>
      <span className={styles.chipIcon}>📝</span>
      <span className={styles.chipText} onClick={onExpand} title="Click to expand">
        "{preview}"
      </span>
      <button onClick={onClear} className={styles.chipClearBtn} title="Remove context">
        ×
      </button>
    </div>
  );
}

// In the input area, show chip above input when text is selected:
{selectedText && selectedText.trim() && (
  <div className={styles.contextArea}>
    <span className={styles.contextLabel}>Asking about:</span>
    <SelectedTextChip
      text={selectedText}
      onClear={clearSelection}
      onExpand={() => setShowFullContext(true)}
    />
  </div>
)}

// Add modal/popup to show full selected text
{showFullContext && (
  <div className={styles.contextModal}>
    <div className={styles.contextModalContent}>
      <h4>Selected Text</h4>
      <p>{selectedText}</p>
      <button onClick={() => setShowFullContext(false)}>Close</button>
    </div>
  </div>
)}
```

5. **Include selected text context in user message display**:
```typescript
// When showing user message, if it had selected text context, show it
<div className={styles.userMessage}>
  {message.contextText && (
    <div className={styles.messageContext}>
      <small>📝 Asked about: "{message.contextText.slice(0, 50)}..."</small>
    </div>
  )}
  <div className={styles.messageContent}>{message.content}</div>
</div>
```

---

## Implementation Order

1. **Phase 1: Fix URL Parsing** (Frontend)
   - Update `getPageContext()` to correctly parse Docusaurus URLs
   - Test that context is sent correctly to backend

2. **Phase 2: Fix URL Storage** (Backend)
   - Update `index_book.py` to store correct URL paths
   - Add `book_base_url` config
   - Update agent instructions to use actual URLs from search
   - Re-index content

3. **Phase 3: Improve Selected Text UX** (Frontend)
   - Implement SelectedTextChip component
   - Add context preview in input area
   - Update message display to show context
   - Add CSS styles for new components

4. **Phase 4: Test End-to-End**
   - Test on Reality Gap lesson page
   - Verify correct lesson content is returned
   - Verify URLs are clickable and correct
   - Verify selected text UX is intuitive

---

## Files to Modify

| File | Changes |
|------|---------|
| `book-source/src/components/ChatWidget/index.tsx` | URL parsing, selected text UX |
| `book-source/src/components/ChatWidget/styles.module.css` | New styles for chips/modals |
| `backend/scripts/index_book.py` | Correct URL format in indexing |
| `backend/app/config.py` | Add book_base_url setting |
| `backend/app/book_assistant_agent.py` | Update citation instructions |
| `backend/app/models.py` | Add current_path to PageContext if needed |

---

## Questions for User Approval

1. **URL Format**: Should URLs be absolute (https://...) or relative (/docs/...)? Relative is better for dev/prod flexibility.

2. **Selected Text UX**: Do you prefer:
   - A) Chip above input field (proposed)
   - B) Inline in textarea as quoted text
   - C) Sidebar panel showing context

3. **Re-indexing**: This will require re-running the indexing script. Is that acceptable?

4. **Lesson-Level Filtering**: Should "summarize this lesson" queries filter ONLY to the current lesson, or include related chapter content for context?
