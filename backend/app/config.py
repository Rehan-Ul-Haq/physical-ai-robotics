"""Configuration module using pydantic Settings."""

import os
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

    # Book URL configuration
    # SITE_BASE_URL: Full URL to the deployed site (e.g., https://rehan-ul-haq.github.io)
    # BOOK_BASE_URL: Path prefix for the book (e.g., /physical-ai-robotics)
    site_base_url: str = "https://rehan-ul-haq.github.io"
    book_base_url: str = "/physical-ai-robotics"

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        extra = "ignore"


@lru_cache
def get_settings() -> Settings:
    """Get cached settings instance."""
    instance = Settings()  # type: ignore
    # Export OpenAI API key to environment for agents SDK
    # The agents SDK requires OPENAI_API_KEY as an environment variable
    if instance.openai_api_key and "OPENAI_API_KEY" not in os.environ:
        os.environ["OPENAI_API_KEY"] = instance.openai_api_key
    return instance


settings = get_settings()
