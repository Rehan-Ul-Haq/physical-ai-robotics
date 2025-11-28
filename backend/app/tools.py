"""RAG search tool for Book Assistant Agent."""

from agents import function_tool
from agents.run_context import RunContextWrapper

from app.qdrant_service import qdrant_service


@function_tool(
    description_override="Search the Physical AI & Humanoid Robotics book for relevant content. Use this tool to find information before answering questions about the book."
)
async def search_book_content(
    ctx: RunContextWrapper,
    query: str,
) -> dict:
    """
    Search the book content using RAG.

    Args:
        ctx: Run context wrapper
        query: The search query to find relevant book content

    Returns:
        Dictionary with search results including text and sources
    """
    # Get context from the agent context if available
    text_filter = None
    if hasattr(ctx, "context") and ctx.context:
        agent_ctx = ctx.context
        if hasattr(agent_ctx, "selected_text") and agent_ctx.selected_text:
            text_filter = agent_ctx.selected_text

    # Perform search
    context_text, sources = await qdrant_service.search_with_sources(
        query=query,
        limit=5,
        text_filter=text_filter,
    )

    if not context_text:
        return {
            "found": False,
            "message": "No relevant content found in the book for this query.",
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
        "context": context_text,
        "sources": formatted_sources,
        "source_count": len(sources),
    }
