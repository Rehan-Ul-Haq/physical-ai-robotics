"""Qdrant service for vector similarity search."""

from openai import AsyncOpenAI
from qdrant_client import AsyncQdrantClient
from qdrant_client.http.models import Filter, FieldCondition, MatchValue, QueryRequest

from app.config import settings
from app.models import BookChunkPayload, Source, SearchScope


class QdrantService:
    """Service for searching book content in Qdrant."""

    def __init__(self) -> None:
        self._client: AsyncQdrantClient | None = None
        self._openai: AsyncOpenAI | None = None

    async def _get_client(self) -> AsyncQdrantClient:
        """Get or create Qdrant client."""
        if self._client is None:
            self._client = AsyncQdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
            )
        return self._client

    async def _get_openai(self) -> AsyncOpenAI:
        """Get or create OpenAI client."""
        if self._openai is None:
            self._openai = AsyncOpenAI(api_key=settings.openai_api_key)
        return self._openai

    async def _get_embedding(self, text: str) -> list[float]:
        """Get embedding for text using OpenAI."""
        openai = await self._get_openai()
        response = await openai.embeddings.create(
            model=settings.embedding_model,
            input=text,
        )
        return response.data[0].embedding

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

    def _build_filter(
        self,
        search_scope: SearchScope,
        current_chapter: int | None,
        current_lesson: str | None,
    ) -> Filter | None:
        """Build Qdrant filter based on search scope and context."""
        conditions = []
        if search_scope == SearchScope.CURRENT_PAGE and current_lesson:
            # Filter by lesson_slug field (e.g., "reality-gap")
            conditions.append(
                FieldCondition(key="lesson_slug", match=MatchValue(value=current_lesson))
            )
        elif search_scope == SearchScope.CURRENT_CHAPTER and current_chapter:
            conditions.append(
                FieldCondition(key="chapter_number", match=MatchValue(value=current_chapter))
            )
        return Filter(must=conditions) if conditions else None

    async def search(
        self,
        query: str,
        limit: int = 5,
        chapter_filter: int | None = None,
        text_filter: str | None = None,
    ) -> list[tuple[BookChunkPayload, float]]:
        """
        Search for relevant book content.

        Args:
            query: The search query
            limit: Maximum number of results
            chapter_filter: Optional chapter number to filter by
            text_filter: Optional text to filter results (for selected text mode)

        Returns:
            List of (payload, score) tuples
        """
        client = await self._get_client()
        query_embedding = await self._get_embedding(query)

        # Build filter conditions
        filter_conditions = []
        if chapter_filter:
            filter_conditions.append(
                FieldCondition(
                    key="chapter_number",
                    match=MatchValue(value=chapter_filter),
                )
            )

        search_filter = None
        if filter_conditions:
            search_filter = Filter(must=filter_conditions)

        # Perform search using query_points (newer qdrant-client API)
        results = await client.query_points(
            collection_name=settings.qdrant_collection,
            query=query_embedding,
            query_filter=search_filter,
            limit=limit,
            with_payload=True,
        )

        # Convert to typed results
        typed_results = []
        for point in results.points:
            payload = point.payload
            if payload:
                chunk = BookChunkPayload(
                    text=payload.get("text", ""),
                    chapter_number=payload.get("chapter_number", 0),
                    chapter_title=payload.get("chapter_title", ""),
                    section_id=payload.get("section_id", ""),
                    section_title=payload.get("section_title", ""),
                    source_url=payload.get("source_url", ""),
                    part_number=payload.get("part_number", 0),
                    token_count=payload.get("token_count", 0),
                )

                # If text_filter is provided, boost results containing the text
                if text_filter:
                    if text_filter.lower() in chunk.text.lower():
                        typed_results.append((chunk, point.score * 1.2))  # Boost
                    else:
                        typed_results.append((chunk, point.score))
                else:
                    typed_results.append((chunk, point.score))

        # Sort by score if text filter was used (to account for boosting)
        if text_filter:
            typed_results.sort(key=lambda x: x[1], reverse=True)

        return typed_results

    async def search_with_sources(
        self,
        query: str,
        limit: int = 5,
        chapter_filter: int | None = None,
        text_filter: str | None = None,
    ) -> tuple[str, list[Source]]:
        """
        Search and return formatted context with sources.

        Returns:
            Tuple of (context_text, list of Source objects)
        """
        results = await self.search(query, limit, chapter_filter, text_filter)

        if not results:
            return "", []

        context_parts = []
        sources = []

        for chunk, score in results:
            context_parts.append(
                f"[Chapter {chunk.chapter_number}: {chunk.section_title}]\n{chunk.text}"
            )
            # Prepend book_base_url to source_url for full path
            full_url = f"{settings.book_base_url}{chunk.source_url}"
            sources.append(
                Source(
                    chapter=chunk.chapter_number,
                    section=chunk.section_title,
                    url=full_url,
                    relevance_score=score,
                )
            )

        context_text = "\n\n---\n\n".join(context_parts)
        return context_text, sources

    async def get_collection_stats(self) -> dict:
        """Get collection statistics."""
        client = await self._get_client()
        try:
            info = await client.get_collection(settings.qdrant_collection)
            return {
                "total_chunks": info.points_count or 0,
                "vectors_count": info.vectors_count or 0,
                "status": info.status.value if info.status else "unknown",
            }
        except Exception as e:
            return {
                "total_chunks": 0,
                "vectors_count": 0,
                "status": "error",
                "error": str(e),
            }

    async def search_multi_query(
        self,
        queries: list[str],
        limit: int = 5,
        current_chapter: int | None = None,
        current_lesson: str | None = None,
        search_scope: SearchScope = SearchScope.FULL_BOOK,
    ) -> list[tuple[BookChunkPayload, float]]:
        """
        Search using multiple queries with batch embedding and context boosting.

        Args:
            queries: List of 1-3 search queries
            limit: Maximum results to return
            current_chapter: Chapter user is viewing (for boosting)
            current_lesson: Lesson user is viewing (for boosting)
            search_scope: How to filter/boost results

        Returns:
            List of (chunk, score) tuples, deduplicated and sorted
        """
        if not queries:
            return []

        client = await self._get_client()
        embeddings = await self._get_embeddings_batch(queries)
        search_filter = self._build_filter(search_scope, current_chapter, current_lesson)

        # Build batch query requests
        query_requests = [
            QueryRequest(
                query=emb,
                filter=search_filter,
                limit=limit,
                with_payload=True,
            )
            for emb in embeddings
        ]

        # Execute batch search
        batch_results = await client.query_batch_points(
            collection_name=settings.qdrant_collection,
            requests=query_requests,
        )

        # Merge and deduplicate results by point ID
        seen_ids: set[str] = set()
        merged: list[tuple[BookChunkPayload, float]] = []

        # Boost factors for context-aware ranking
        chapter_boost = 1.2
        lesson_boost = 1.1

        for result_set in batch_results:
            for point in result_set.points:
                point_id = str(point.id)
                if point_id in seen_ids:
                    continue
                seen_ids.add(point_id)

                payload = point.payload
                if not payload:
                    continue

                chunk = BookChunkPayload(
                    text=payload.get("text", ""),
                    chapter_number=payload.get("chapter_number", 0),
                    chapter_title=payload.get("chapter_title", ""),
                    section_id=payload.get("section_id", ""),
                    section_title=payload.get("section_title", ""),
                    source_url=payload.get("source_url", ""),
                    part_number=payload.get("part_number", 0),
                    token_count=payload.get("token_count", 0),
                )

                score = point.score
                # Get lesson_slug from payload for boosting
                lesson_slug = payload.get("lesson_slug")
                if search_scope == SearchScope.FULL_BOOK:
                    if current_chapter and chunk.chapter_number == current_chapter:
                        score *= chapter_boost
                        if current_lesson and lesson_slug == current_lesson:
                            score *= lesson_boost

                merged.append((chunk, score))

        merged.sort(key=lambda x: x[1], reverse=True)
        return merged[:limit]

    def _format_results_with_sources(
        self, results: list[tuple[BookChunkPayload, float]]
    ) -> tuple[str, list[Source]]:
        """Format search results into context text and source list."""
        if not results:
            return "", []

        context_parts = []
        sources = []

        for chunk, score in results:
            context_parts.append(
                f"[Chapter {chunk.chapter_number}: {chunk.section_title}]\n{chunk.text}"
            )
            # Prepend book_base_url to source_url for full path
            full_url = f"{settings.book_base_url}{chunk.source_url}"
            sources.append(
                Source(
                    chapter=chunk.chapter_number,
                    section=chunk.section_title,
                    url=full_url,
                    relevance_score=min(score, 1.0),
                )
            )

        context_text = "\n\n---\n\n".join(context_parts)
        return context_text, sources

    async def search_multi_query_with_sources(
        self,
        queries: list[str],
        limit: int = 5,
        current_chapter: int | None = None,
        current_lesson: str | None = None,
        search_scope: SearchScope = SearchScope.FULL_BOOK,
    ) -> tuple[str, list[Source]]:
        """
        Multi-query search with formatted context and sources.

        Returns:
            Tuple of (context_text, list of Source objects)
        """
        results = await self.search_multi_query(
            queries=queries,
            limit=limit,
            current_chapter=current_chapter,
            current_lesson=current_lesson,
            search_scope=search_scope,
        )
        return self._format_results_with_sources(results)


# Global instance
qdrant_service = QdrantService()
