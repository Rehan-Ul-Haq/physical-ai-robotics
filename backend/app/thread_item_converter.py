"""Thread item converter for converting ChatKit items to agent input format."""

from typing import Any

from chatkit.agents import simple_to_agent_input
from chatkit.store import Store
from chatkit.types import (
    AssistantMessageItem,
    UserMessageTextContent,
    ThreadItem,
    ThreadMetadata,
    UserMessageItem,
)


def build_context_prefix(context: dict[str, Any]) -> str:
    """
    Build a context prefix to prepend to the user message.

    This ensures the agent knows about:
    - Selected text (if any)
    - Current chapter and lesson
    """
    parts = []

    # Add page context info
    page_context = context.get("page_context", {})
    current_chapter = page_context.get("current_chapter") if page_context else context.get("current_chapter")
    current_lesson = page_context.get("current_lesson") if page_context else context.get("current_lesson")

    if current_chapter or current_lesson:
        location_parts = []
        if current_chapter:
            location_parts.append(f"Chapter {current_chapter}")
        if current_lesson:
            # Clean up lesson slug for display
            lesson_name = current_lesson.replace("-", " ").title()
            location_parts.append(f"Lesson: {lesson_name}")
        parts.append(f"[User is viewing: {', '.join(location_parts)}]")

    # Add selected text if present
    selected_text = context.get("selected_text")
    if selected_text:
        # Truncate if very long
        if len(selected_text) > 1500:
            selected_text = selected_text[:1500] + "..."
        parts.append(f"[User has selected this text from the book:]\n\"\"\"\n{selected_text}\n\"\"\"")

    if parts:
        return "\n".join(parts) + "\n\n[User's question:]\n"
    return ""


def inject_context_into_message(item: UserMessageItem, context: dict[str, Any]) -> UserMessageItem:
    """
    Inject context prefix into the user message content.

    This modifies the message to include context information that
    the agent needs to understand the user's question.
    """
    prefix = build_context_prefix(context)
    if not prefix:
        return item

    # Extract original text content
    original_text = ""
    if isinstance(item.content, list):
        for content_item in item.content:
            if hasattr(content_item, "text"):
                original_text += content_item.text
            elif isinstance(content_item, str):
                original_text += content_item
    elif isinstance(item.content, str):
        original_text = item.content

    # Create new message with context prefix (type must be "input_text" per ChatKit schema)
    new_content = [UserMessageTextContent(type="input_text", text=prefix + original_text)]

    # Use model_copy to preserve all required fields from original item
    return item.model_copy(update={"content": new_content})


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
    3. Injecting page context and selected text into the current message

    Args:
        thread: Thread metadata
        item: Current user message item (may be None)
        store: Thread store for loading history
        context: Request context (contains selected_text, page_context, etc.)

    Returns:
        List of agent input items or None if no input
    """
    if item is None:
        return None

    # Collect all items to convert (history + current)
    all_items: list[ThreadItem] = []

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
                # Only include user and assistant messages from history
                for history_item in history_page.data:
                    if isinstance(history_item, (UserMessageItem, AssistantMessageItem)):
                        all_items.append(history_item)

        except Exception:
            # If loading history fails, just use the current message
            pass

    # Inject context into current message (selected text, page context)
    # Only inject for the current message, not history
    modified_item = inject_context_into_message(item, context)
    all_items.append(modified_item)

    # Use ChatKit's converter for all items to ensure proper format
    return await simple_to_agent_input(all_items)


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
                # Extract text from content list
                content = item.content
                if isinstance(content, list):
                    text_parts = []
                    for c in content:
                        if hasattr(c, "text"):
                            text_parts.append(c.text)
                        elif isinstance(c, str):
                            text_parts.append(c)
                    content = " ".join(text_parts) if text_parts else ""
                history.append({
                    "role": "user",
                    "content": content,
                })
            elif isinstance(item, AssistantMessageItem):
                content = item.content
                if isinstance(content, list):
                    text_parts = []
                    for c in content:
                        if hasattr(c, "text"):
                            text_parts.append(c.text)
                        elif isinstance(c, str):
                            text_parts.append(c)
                    content = "\n".join(text_parts) if text_parts else ""
                history.append({
                    "role": "assistant",
                    "content": content,
                })

        return history

    except Exception:
        return []
