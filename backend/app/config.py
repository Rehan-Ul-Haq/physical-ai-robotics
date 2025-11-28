"""Configuration module using pydantic Settings."""

from pydantic_settings import BaseSettings
from functools import lru_cache


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # OpenAI
    openai_api_key: str

    # Qdrant Cloud
    qdrant_url: str
    qdrant_api_key: str

    # Neon Postgres
    database_url: str

    # Server
    host: str = "127.0.0.1"
    port: int = 8001
    debug: bool = False

    # Rate Limiting
    rate_limit_requests: int = 20
    rate_limit_window: int = 60

    # Model settings
    embedding_model: str = "text-embedding-3-small"
    chat_model: str = "gpt-4o-mini"

    # Qdrant collection
    qdrant_collection: str = "book_content"
    embedding_dimensions: int = 1536

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        extra = "ignore"


@lru_cache
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()  # type: ignore


settings = get_settings()
