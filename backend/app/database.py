"""Database connection utility using asyncpg pool."""

import asyncpg
from contextlib import asynccontextmanager
from typing import AsyncIterator

from app.config import settings


class Database:
    """Async database connection pool manager."""

    def __init__(self) -> None:
        self._pool: asyncpg.Pool | None = None

    async def connect(self) -> None:
        """Initialize the connection pool."""
        if self._pool is None:
            self._pool = await asyncpg.create_pool(
                dsn=settings.database_url,
                min_size=2,
                max_size=10,
                command_timeout=60,
            )

    async def disconnect(self) -> None:
        """Close the connection pool."""
        if self._pool is not None:
            await self._pool.close()
            self._pool = None

    @asynccontextmanager
    async def connection(self) -> AsyncIterator[asyncpg.Connection]:
        """Get a connection from the pool."""
        if self._pool is None:
            await self.connect()
        async with self._pool.acquire() as conn:  # type: ignore
            yield conn

    async def execute(self, query: str, *args) -> str:
        """Execute a query and return status."""
        async with self.connection() as conn:
            return await conn.execute(query, *args)

    async def fetch(self, query: str, *args) -> list[asyncpg.Record]:
        """Fetch multiple rows."""
        async with self.connection() as conn:
            return await conn.fetch(query, *args)

    async def fetchrow(self, query: str, *args) -> asyncpg.Record | None:
        """Fetch a single row."""
        async with self.connection() as conn:
            return await conn.fetchrow(query, *args)

    async def fetchval(self, query: str, *args):
        """Fetch a single value."""
        async with self.connection() as conn:
            return await conn.fetchval(query, *args)


# Global database instance
db = Database()
