# RAG Search Patterns for ChatKit

This reference documents context-aware RAG search patterns used in ChatKit server implementations.

## Multi-Query Search with Batch Embeddings

For better recall, generate 1-3 semantically diverse queries and embed them in a single API call:

```python
async def _get_embeddings_batch(self, texts: list[str]) -> list[list[float]]:
    """Get embeddings for multiple texts in a single API call."""
    if not texts:
        return []
    openai = await self._get_openai()
    response = await openai.embeddings.create(
        model=settings.embedding_model,
        input=texts,
    )
    return [item.embedding for item in response.data]

async def search_multi_query(
    self,
    queries: list[str],
    limit: int = 5,
    current_chapter: int | None = None,
    current_lesson: str | None = None,
    search_scope: SearchScope = SearchScope.FULL_BOOK,
) -> list[tuple[BookChunkPayload, float]]:
    embeddings = await self._get_embeddings_batch(queries)
    search_filter = self._build_filter(search_scope, current_chapter, current_lesson)

    # Build batch query requests
    query_requests = [
        QueryRequest(query=emb, filter=search_filter, limit=limit, with_payload=True)
        for emb in embeddings
    ]

    # Execute batch search
    batch_results = await client.query_batch_points(
        collection_name=settings.qdrant_collection,
        requests=query_requests,
    )

    # Merge and deduplicate by point ID
    seen_ids: set[str] = set()
    merged: list[tuple[BookChunkPayload, float]] = []

    for result_set in batch_results:
        for point in result_set.points:
            if str(point.id) in seen_ids:
                continue
            seen_ids.add(str(point.id))
            # Process point...
            merged.append((chunk, score))

    merged.sort(key=lambda x: x[1], reverse=True)
    return merged[:limit]
```

## Context-Aware Boosting

Apply boost factors based on user's current location:

```python
# Boost factors
CHAPTER_BOOST = 1.2  # 20% boost for same chapter
LESSON_BOOST = 1.1   # Additional 10% boost for same lesson

# In search loop:
score = point.score
lesson_slug = payload.get("lesson_slug")

if search_scope == SearchScope.FULL_BOOK:
    if current_chapter and chunk.chapter_number == current_chapter:
        score *= CHAPTER_BOOST
        if current_lesson and lesson_slug == current_lesson:
            score *= LESSON_BOOST
```

## Search Scope Filter Building

```python
def _build_filter(
    self,
    search_scope: SearchScope,
    current_chapter: int | None,
    current_lesson: str | None,
) -> Filter | None:
    conditions = []

    if search_scope == SearchScope.CURRENT_PAGE and current_lesson:
        # Strict filter to lesson only
        conditions.append(
            FieldCondition(key="lesson_slug", match=MatchValue(value=current_lesson))
        )
    elif search_scope == SearchScope.CURRENT_CHAPTER and current_chapter:
        # Strict filter to chapter
        conditions.append(
            FieldCondition(key="chapter_number", match=MatchValue(value=current_chapter))
        )
    # FULL_BOOK: No filter, use boosting instead

    return Filter(must=conditions) if conditions else None
```

## Qdrant Payload Schema

Indexed chunks should contain:

```python
payload = {
    "text": str,              # The chunk content
    "chapter_number": int,    # For chapter filtering/boosting
    "chapter_title": str,     # For display
    "section_id": str,        # URL anchor slug
    "section_title": str,     # For citations
    "source_url": str,        # Full URL for linking
    "part_number": int,       # Book part for hierarchical filtering
    "part_slug": str,         # URL path segment
    "chapter_slug": str,      # URL path segment
    "lesson_slug": str,       # For lesson filtering/boosting
    "lesson_title": str,      # For display
    "token_count": int,       # For context window management
}
```

## Chunking Strategy

Split markdown by headings to preserve semantic coherence:

```python
def chunk_markdown(content: str, chapter_num: int, chapter_title: str, lesson_slug: str | None) -> list[dict]:
    chunks = []
    current_section = ""
    current_content: list[str] = []

    for line in content.split("\n"):
        if line.startswith("## ") or line.startswith("### "):
            if current_content:
                text = "\n".join(current_content).strip()
                if text:
                    chunks.append({
                        "section_title": current_section or chapter_title,
                        "section_id": current_section.lower().replace(" ", "-").replace(":", "").replace("?", "") or "intro",
                        "text": text,
                        "chapter_number": chapter_num,
                        "chapter_title": chapter_title,
                        "lesson_slug": lesson_slug,
                    })
            current_section = line.lstrip("#").strip()
            current_content = [line]
        else:
            current_content.append(line)

    # Don't forget final chunk
    if current_content:
        # ... same as above
```

## Source URL Generation

Build correct Docusaurus URLs from chunk metadata:

```python
def build_source_url(part_slug: str, chapter_slug: str, lesson_slug: str | None, section_id: str) -> str:
    """Build Docusaurus URL: /docs/{part}/{chapter}/{lesson}#{section}"""
    if lesson_slug:
        return f"/docs/{part_slug}/{chapter_slug}/{lesson_slug}#{section_id}"
    else:
        return f"/docs/{part_slug}/{chapter_slug}#{section_id}"
```
