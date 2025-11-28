"""Thread item converter for converting ChatKit items to agent input format."""

from typing import Any

from chatkit.agents import simple_to_agent_input
from chatkit.store import Store
from chatkit.types import (
    AssistantMessageItem,
    ThreadMetadata,
    UserMessageItem,
)


async def to_agent_input(
    thread: ThreadMetadata,
    item: UserMessageItem | None,
    store: Store,
    context: dict[str, Any],
) -> list | None:
    """
    Convert ChatKit items to agent input format.

    This handles:
    1. Converting the current user message
    2. Loading conversation history for context

    Args:
        thread: Thread metadata
        item: Current user message item (may be None)
        store: Thread store for loading history
        context: Request context

    Returns:
        List of agent input items or None if no input
    """
    if item is None:
        return None

    # Use the simple converter from chatkit.agents
    agent_input = await simple_to_agent_input(item)

    # Load conversation history if this is a continuing thread
    if thread.id:
        try:
            # Load recent thread items for context
            history_page = await store.load_thread_items(
                thread_id=thread.id,
                after=None,
                limit=20,  # Last 20 messages for context
                order="asc",  # Oldest first for proper ordering
                context=context,
            )

            if history_page.data:
                # Convert history items to agent input format
                history_input = []
                for history_item in history_page.data:
                    if isinstance(history_item, UserMessageItem):
                        history_input.append({
                            "role": "user",
                            "content": history_item.content,
                        })
                    elif isinstance(history_item, AssistantMessageItem):
                        content = history_item.content
                        if isinstance(content, list):
                            content = "\n".join(str(c) for c in content)
                        history_input.append({
                            "role": "assistant",
                            "content": content,
                        })

                # Prepend history to current input
                if history_input:
                    # The current input from simple_to_agent_input should be added after history
                    return history_input + agent_input

        except Exception:
            # If loading history fails, just use the current message
            pass

    return agent_input


async def load_thread_history(
    thread_id: str,
    store: Store,
    context: dict[str, Any],
    limit: int = 20,
) -> list[dict[str, str]]:
    """
    Load thread history as a list of message dicts.

    Args:
        thread_id: Thread ID to load
        store: Thread store
        context: Request context
        limit: Maximum messages to load

    Returns:
        List of message dicts with 'role' and 'content'
    """
    try:
        history_page = await store.load_thread_items(
            thread_id=thread_id,
            after=None,
            limit=limit,
            order="asc",
            context=context,
        )

        history = []
        for item in history_page.data:
            if isinstance(item, UserMessageItem):
                history.append({
                    "role": "user",
                    "content": item.content,
                })
            elif isinstance(item, AssistantMessageItem):
                content = item.content
                if isinstance(content, list):
                    content = "\n".join(str(c) for c in content)
                history.append({
                    "role": "assistant",
                    "content": content,
                })

        return history

    except Exception:
        return []
