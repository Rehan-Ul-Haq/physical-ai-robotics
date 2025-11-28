"""Qdrant setup script to create book_content collection."""

import os
import sys

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dotenv import load_dotenv

load_dotenv()

from qdrant_client import QdrantClient
from qdrant_client.http.models import VectorParams, Distance

from app.config import settings


def setup_qdrant() -> None:
    """Create Qdrant collection for book content."""
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
    )

    # Check if collection exists
    collections = client.get_collections().collections
    collection_names = [c.name for c in collections]

    if settings.qdrant_collection in collection_names:
        print(f"Collection '{settings.qdrant_collection}' already exists.")
        # Get collection info
        info = client.get_collection(settings.qdrant_collection)
        print(f"  Vectors count: {info.vectors_count}")
        print(f"  Points count: {info.points_count}")
        return

    # Create collection
    client.create_collection(
        collection_name=settings.qdrant_collection,
        vectors_config=VectorParams(
            size=settings.embedding_dimensions,  # text-embedding-3-small dimensions
            distance=Distance.COSINE,
        ),
    )

    print(f"Qdrant collection '{settings.qdrant_collection}' created successfully")
    print(f"  Dimensions: {settings.embedding_dimensions}")
    print(f"  Distance: COSINE")


if __name__ == "__main__":
    setup_qdrant()
