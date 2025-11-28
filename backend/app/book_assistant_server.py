"""BookAssistantServer extending ChatKitServer for Book Assistant."""

from typing import Any, AsyncIterator

from agents import Runner
from chatkit.agents import AgentContext, stream_agent_response
from chatkit.server import ChatKitServer
from chatkit.types import ThreadMetadata, ThreadStreamEvent, UserMessageItem

from app.book_assistant_agent import book_assistant_agent
from app.postgres_thread_store import PostgresThreadStore
from app.thread_item_converter import to_agent_input
from app.models import PageContext


class BookAssistantAgentContext(AgentContext):
    """Extended AgentContext with page context and selected text support."""

    selected_text: str | None = None
    context_mode: str = "full_book"
    page_context: PageContext | None = None


class BookAssistantServer(ChatKitServer[dict[str, Any]]):
    """ChatKit server for Book Assistant with RAG capabilities."""

    def __init__(self) -> None:
        store = PostgresThreadStore()
        super().__init__(store)
        self.assistant = book_assistant_agent

    async def respond(
        self,
        thread: ThreadMetadata,
        item: UserMessageItem | None,
        context: dict[str, Any],
    ) -> AsyncIterator[ThreadStreamEvent]:
        """
        Handle user messages and stream AI responses.

        This method:
        1. Converts the user message to agent input format
        2. Creates an AgentContext with thread, store, and page context
        3. Runs the agent with streaming
        4. Yields ChatKit events for the response
        """
        # Create agent context with page context support
        agent_context = BookAssistantAgentContext(
            thread=thread,
            store=self.store,
            request_context=context,
        )

        # Extract selected_text from context if present
        if "selected_text" in context:
            agent_context.selected_text = context["selected_text"]
        if "context_mode" in context:
            agent_context.context_mode = context["context_mode"]
        
        # Extract page context (current_chapter, current_lesson) from frontend
        if "page_context" in context and context["page_context"]:
            page_ctx = context["page_context"]
            agent_context.page_context = PageContext(
                current_chapter=page_ctx.get("current_chapter"),
                current_lesson=page_ctx.get("current_lesson"),
            )
        elif "current_chapter" in context or "current_lesson" in context:
            # Also support flat structure
            agent_context.page_context = PageContext(
                current_chapter=context.get("current_chapter"),
                current_lesson=context.get("current_lesson"),
            )

        # Convert user message to agent input
        agent_input = await to_agent_input(thread, item, self.store, context)


        if not agent_input:
            return

        # Get previous_response_id from thread metadata for efficient re-sending
        previous_response_id = None
        if thread.metadata:
            previous_response_id = thread.metadata.get("previous_response_id")

        # Run the agent with streaming
        result = Runner.run_streamed(
            self.assistant,
            input=agent_input,
            context=agent_context,
            previous_response_id=previous_response_id,
        )

        # Save the new response_id for next run
        if hasattr(result, "response_id") and result.response_id:
            if thread.metadata is None:
                thread.metadata = {}
            thread.metadata["previous_response_id"] = result.response_id

        # Stream the response events
        async for event in stream_agent_response(agent_context, result):
            yield event


# Global server instance
book_assistant_server = BookAssistantServer()
