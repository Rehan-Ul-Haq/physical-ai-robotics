"""PostgresThreadStore implementation for ChatKit Store protocol."""

import json
import uuid
from datetime import datetime, timezone
from typing import Any, Literal

from chatkit.store import Store
from chatkit.types import Page, ThreadItem, ThreadMetadata

from app.database import db


class PostgresThreadStore(Store[dict[str, Any]]):
    """Postgres-backed thread store implementing ChatKit Store protocol."""

    def generate_thread_id(self, context: dict[str, Any]) -> str:
        """Generate a unique thread ID."""
        return f"thread_{uuid.uuid4().hex[:16]}"

    def generate_item_id(
        self,
        item_type: Literal["message", "tool_call", "task", "workflow", "attachment"],
        thread: ThreadMetadata,
        context: dict[str, Any],
    ) -> str:
        """Generate a unique item ID."""
        return f"{item_type}_{uuid.uuid4().hex[:16]}"

    async def load_thread(
        self, thread_id: str, context: dict[str, Any]
    ) -> ThreadMetadata:
        """Load thread metadata from Postgres."""
        row = await db.fetchrow(
            """
            SELECT id, title, metadata, created_at, updated_at
            FROM threads
            WHERE id = $1
            """,
            thread_id,
        )

        if row is None:
            # Return a new thread metadata if not found
            return ThreadMetadata(
                id=thread_id,
                title=None,
                metadata={},
            )

        metadata = row["metadata"] if row["metadata"] else {}
        if isinstance(metadata, str):
            metadata = json.loads(metadata)

        return ThreadMetadata(
            id=row["id"],
            title=row["title"],
            metadata=metadata,
        )

    async def save_thread(
        self, thread: ThreadMetadata, context: dict[str, Any]
    ) -> None:
        """Save or update thread metadata in Postgres."""
        metadata_json = json.dumps(thread.metadata) if thread.metadata else "{}"

        await db.execute(
            """
            INSERT INTO threads (id, title, metadata, created_at, updated_at)
            VALUES ($1, $2, $3, NOW(), NOW())
            ON CONFLICT (id) DO UPDATE
            SET title = EXCLUDED.title,
                metadata = EXCLUDED.metadata,
                updated_at = NOW()
            """,
            thread.id,
            thread.title,
            metadata_json,
        )

    async def load_thread_items(
        self,
        thread_id: str,
        after: str | None,
        limit: int,
        order: str,
        context: dict[str, Any],
    ) -> Page[ThreadItem]:
        """Load paginated thread items from Postgres."""
        order_direction = "DESC" if order == "desc" else "ASC"

        if after:
            # Get the created_at of the 'after' item for cursor pagination
            after_row = await db.fetchrow(
                "SELECT created_at FROM thread_items WHERE id = $1",
                after,
            )
            if after_row:
                if order == "desc":
                    query = f"""
                        SELECT id, thread_id, type, role, content, sources, metadata, created_at
                        FROM thread_items
                        WHERE thread_id = $1 AND created_at < $2
                        ORDER BY created_at {order_direction}
                        LIMIT $3
                    """
                else:
                    query = f"""
                        SELECT id, thread_id, type, role, content, sources, metadata, created_at
                        FROM thread_items
                        WHERE thread_id = $1 AND created_at > $2
                        ORDER BY created_at {order_direction}
                        LIMIT $3
                    """
                rows = await db.fetch(query, thread_id, after_row["created_at"], limit + 1)
            else:
                rows = []
        else:
            query = f"""
                SELECT id, thread_id, type, role, content, sources, metadata, created_at
                FROM thread_items
                WHERE thread_id = $1
                ORDER BY created_at {order_direction}
                LIMIT $2
            """
            rows = await db.fetch(query, thread_id, limit + 1)

        items = []
        for row in rows[:limit]:
            sources = row["sources"]
            if isinstance(sources, str):
                sources = json.loads(sources)

            metadata = row["metadata"]
            if isinstance(metadata, str):
                metadata = json.loads(metadata)

            # Create a ThreadItem based on the type
            item = self._row_to_thread_item(row, sources, metadata)
            items.append(item)

        has_more = len(rows) > limit
        return Page(
            data=items,
            has_more=has_more,
            after=items[-1].id if has_more and items else None,
        )

    def _row_to_thread_item(
        self, row: Any, sources: Any, metadata: Any
    ) -> ThreadItem:
        """Convert a database row to a ThreadItem."""
        from chatkit.types import AssistantMessageItem, UserMessageItem, HiddenContextItem

        item_type = row["type"]
        role = row["role"]

        if role == "user":
            return UserMessageItem(
                id=row["id"],
                thread_id=row["thread_id"],
                content=row["content"] or "",
                created_at=row["created_at"],
            )
        elif role == "assistant":
            return AssistantMessageItem(
                id=row["id"],
                thread_id=row["thread_id"],
                content=[row["content"]] if row["content"] else [],
                created_at=row["created_at"],
            )
        elif item_type == "hidden_context":
            return HiddenContextItem(
                id=row["id"],
                thread_id=row["thread_id"],
                content=[row["content"]] if row["content"] else [],
                created_at=row["created_at"],
            )
        else:
            # Default to AssistantMessageItem
            return AssistantMessageItem(
                id=row["id"],
                thread_id=row["thread_id"],
                content=[row["content"]] if row["content"] else [],
                created_at=row["created_at"],
            )

    async def add_thread_item(
        self, thread_id: str, item: ThreadItem, context: dict[str, Any]
    ) -> None:
        """Add a thread item to Postgres."""
        from chatkit.types import AssistantMessageItem, UserMessageItem

        # Determine role and content based on item type
        role = None
        content = None
        item_type = "message"

        if isinstance(item, UserMessageItem):
            role = "user"
            content = item.content
        elif isinstance(item, AssistantMessageItem):
            role = "assistant"
            content = item.content[0] if item.content else None
        else:
            # Handle other types
            item_type = getattr(item, "type", "message")
            content = str(item.content) if hasattr(item, "content") else None

        sources_json = None
        metadata_json = "{}"

        created_at = getattr(item, "created_at", datetime.now(timezone.utc))

        await db.execute(
            """
            INSERT INTO thread_items (id, thread_id, type, role, content, sources, metadata, created_at)
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
            ON CONFLICT (id) DO NOTHING
            """,
            item.id,
            thread_id,
            item_type,
            role,
            content,
            sources_json,
            metadata_json,
            created_at,
        )

    async def save_item(
        self, thread_id: str, item: ThreadItem, context: dict[str, Any]
    ) -> None:
        """Save or update a thread item."""
        # For now, delegate to add_thread_item
        await self.add_thread_item(thread_id, item, context)

    async def load_item(
        self, thread_id: str, item_id: str, context: dict[str, Any]
    ) -> ThreadItem:
        """Load a single thread item."""
        row = await db.fetchrow(
            """
            SELECT id, thread_id, type, role, content, sources, metadata, created_at
            FROM thread_items
            WHERE id = $1 AND thread_id = $2
            """,
            item_id,
            thread_id,
        )

        if row is None:
            raise ValueError(f"Item {item_id} not found in thread {thread_id}")

        sources = row["sources"]
        if isinstance(sources, str):
            sources = json.loads(sources)

        metadata = row["metadata"]
        if isinstance(metadata, str):
            metadata = json.loads(metadata)

        return self._row_to_thread_item(row, sources, metadata)

    async def delete_thread(self, thread_id: str, context: dict[str, Any]) -> None:
        """Delete a thread and all its items."""
        # Items are deleted via CASCADE
        await db.execute(
            "DELETE FROM threads WHERE id = $1",
            thread_id,
        )

    async def load_threads(
        self,
        limit: int,
        after: str | None,
        order: str,
        context: dict[str, Any],
    ) -> Page[ThreadMetadata]:
        """Load paginated threads."""
        order_direction = "DESC" if order == "desc" else "ASC"

        if after:
            after_row = await db.fetchrow(
                "SELECT updated_at FROM threads WHERE id = $1",
                after,
            )
            if after_row:
                if order == "desc":
                    query = f"""
                        SELECT id, title, metadata, created_at, updated_at
                        FROM threads
                        WHERE updated_at < $1
                        ORDER BY updated_at {order_direction}
                        LIMIT $2
                    """
                else:
                    query = f"""
                        SELECT id, title, metadata, created_at, updated_at
                        FROM threads
                        WHERE updated_at > $1
                        ORDER BY updated_at {order_direction}
                        LIMIT $2
                    """
                rows = await db.fetch(query, after_row["updated_at"], limit + 1)
            else:
                rows = []
        else:
            query = f"""
                SELECT id, title, metadata, created_at, updated_at
                FROM threads
                ORDER BY updated_at {order_direction}
                LIMIT $1
            """
            rows = await db.fetch(query, limit + 1)

        threads = []
        for row in rows[:limit]:
            metadata = row["metadata"]
            if isinstance(metadata, str):
                metadata = json.loads(metadata)

            threads.append(
                ThreadMetadata(
                    id=row["id"],
                    title=row["title"],
                    metadata=metadata,
                )
            )

        has_more = len(rows) > limit
        return Page(
            data=threads,
            has_more=has_more,
            after=threads[-1].id if has_more and threads else None,
        )

    async def save_attachment(self, attachment: Any, context: dict[str, Any]) -> None:
        """Save attachment (not implemented for MVP)."""
        pass

    async def load_attachment(
        self, attachment_id: str, context: dict[str, Any]
    ) -> Any:
        """Load attachment (not implemented for MVP)."""
        raise NotImplementedError("Attachments not supported in MVP")

    async def delete_attachment(
        self, attachment_id: str, context: dict[str, Any]
    ) -> None:
        """Delete attachment (not implemented for MVP)."""
        pass
