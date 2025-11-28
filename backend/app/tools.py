"""RAG search tool for Book Assistant Agent."""

from typing import Annotated, Any

from agents import function_tool
from agents.run_context import RunContextWrapper

from app.qdrant_service import qdrant_service
from app.models import SearchScope


@function_tool(
    description_override="""Search the Physical AI & Humanoid Robotics book for relevant content.

IMPORTANT: Always use this tool before answering questions about the book.

Parameters:
- queries: Provide 1-3 semantically diverse search queries for better recall
- search_scope: How to use page context:
  - "current_page": Strict filter to current lesson (use for "what does this mean?" questions)
  - "current_chapter": Filter to current chapter (use for chapter-specific questions)
  - "full_book": Search everything with context boosting (default, use for general questions)
- current_chapter: Chapter number user is viewing (1-14), provided by frontend context
- current_lesson: Lesson slug user is viewing, provided by frontend context

Examples:
- General question: queries=["What is Physical AI", "embodied AI definition", "robots physical world"]
- Chapter question: queries=["ROS nodes explained"], search_scope="current_chapter"
- Page question: queries=["explain this concept"], search_scope="current_page"
"""
)
async def search_book_content(
    ctx: RunContextWrapper[Any],
    queries: Annotated[list[str], "1-3 semantically diverse search queries for better recall"],
    search_scope: Annotated[str, "Search scope: current_page, current_chapter, or full_book"] = "full_book",
    current_chapter: Annotated[int | None, "Chapter number user is viewing (1-14)"] = None,
    current_lesson: Annotated[str | None, "Lesson slug user is viewing"] = None,
) -> dict:
    """
    Search the book content using multi-query RAG with context awareness.

    Args:
        ctx: Run context wrapper with BookAssistantAgentContext
        queries: List of 1-3 search queries (use different phrasings for better recall)
        search_scope: How to filter results based on context
        current_chapter: Chapter number from frontend context
        current_lesson: Lesson slug from frontend context

    Returns:
        Dictionary with search results including text and sources
    """
    # Validate and limit queries
    if not queries:
        return {
            "found": False,
            "message": "No search queries provided.",
            "context": "",
            "sources": [],
        }
    
    # Limit to max 3 queries
    queries = queries[:3]
    
    # Get context from agent context if available (frontend passes this)
    # The context is accessed via ctx.context which holds BookAssistantAgentContext
    if ctx.context is not None:
        agent_ctx = ctx.context
        # Use page context from frontend if not provided in tool call
        if hasattr(agent_ctx, "page_context") and agent_ctx.page_context is not None:
            if current_chapter is None:
                current_chapter = agent_ctx.page_context.current_chapter
            if current_lesson is None:
                current_lesson = agent_ctx.page_context.current_lesson

    # Parse search scope
    try:
        scope = SearchScope(search_scope)
    except ValueError:
        scope = SearchScope.FULL_BOOK

    # Perform multi-query search with context boosting
    context_text, sources = await qdrant_service.search_multi_query_with_sources(
        queries=queries,
        limit=5,
        current_chapter=current_chapter,
        current_lesson=current_lesson,
        search_scope=scope,
    )

    if not context_text:
        return {
            "found": False,
            "message": "No relevant content found in the book for these queries.",
            "queries_used": queries,
            "search_scope": search_scope,
            "context": "",
            "sources": [],
        }

    # Format sources for the agent
    formatted_sources = [
        {
            "chapter": s.chapter,
            "section": s.section,
            "url": s.url,
            "relevance": round(s.relevance_score, 3),
        }
        for s in sources
    ]

    return {
        "found": True,
        "queries_used": queries,
        "search_scope": search_scope,
        "current_chapter": current_chapter,
        "current_lesson": current_lesson,
        "context": context_text,
        "sources": formatted_sources,
        "source_count": len(sources),
    }
