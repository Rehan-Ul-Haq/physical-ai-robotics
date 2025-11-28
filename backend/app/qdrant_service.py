"""Qdrant service for vector similarity search."""

from openai import AsyncOpenAI
from qdrant_client import AsyncQdrantClient
from qdrant_client.http.models import Filter, FieldCondition, MatchValue

from app.config import settings
from app.models import BookChunkPayload, Source


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

        # Perform search
        results = await client.search(
            collection_name=settings.qdrant_collection,
            query_vector=query_embedding,
            query_filter=search_filter,
            limit=limit,
            with_payload=True,
        )

        # Convert to typed results
        typed_results = []
        for result in results:
            payload = result.payload
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
                        typed_results.append((chunk, result.score * 1.2))  # Boost
                    else:
                        typed_results.append((chunk, result.score))
                else:
                    typed_results.append((chunk, result.score))

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
            sources.append(
                Source(
                    chapter=chunk.chapter_number,
                    section=chunk.section_title,
                    url=chunk.source_url,
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


# Global instance
qdrant_service = QdrantService()
