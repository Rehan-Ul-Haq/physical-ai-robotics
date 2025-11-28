"""Book indexing script for chunking markdown and storing in Qdrant."""

import os
import sys
import uuid
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dotenv import load_dotenv

load_dotenv()

from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct

from app.config import settings


def chunk_markdown(
    content: str, chapter_num: int, chapter_title: str
) -> list[dict]:
    """Split markdown by headings into chunks."""
    chunks = []
    current_section = ""
    current_content: list[str] = []

    for line in content.split("\n"):
        if line.startswith("## ") or line.startswith("### "):
            if current_content:
                text = "\n".join(current_content).strip()
                if text:  # Only add non-empty chunks
                    chunks.append(
                        {
                            "section_title": current_section or chapter_title,
                            "section_id": (
                                current_section.lower().replace(" ", "-").replace(":", "")
                                or "intro"
                            ),
                            "text": text,
                            "chapter_number": chapter_num,
                            "chapter_title": chapter_title,
                        }
                    )
            current_section = line.lstrip("#").strip()
            current_content = [line]
        else:
            current_content.append(line)

    # Add final chunk
    if current_content:
        text = "\n".join(current_content).strip()
        if text:
            chunks.append(
                {
                    "section_title": current_section or chapter_title,
                    "section_id": (
                        current_section.lower().replace(" ", "-").replace(":", "")
                        or "intro"
                    ),
                    "text": text,
                    "chapter_number": chapter_num,
                    "chapter_title": chapter_title,
                }
            )

    return chunks


def get_embedding(client: OpenAI, text: str) -> list[float]:
    """Get embedding from OpenAI."""
    response = client.embeddings.create(
        model=settings.embedding_model,
        input=text,
    )
    return response.data[0].embedding


def index_chapter(
    openai_client: OpenAI,
    qdrant_client: QdrantClient,
    chapter_path: Path,
    chapter_num: int,
    chapter_title: str,
    part_num: int,
) -> int:
    """Index a chapter into Qdrant. Returns number of chunks indexed."""
    content = chapter_path.read_text(encoding="utf-8")
    chunks = chunk_markdown(content, chapter_num, chapter_title)

    if not chunks:
        print(f"  No chunks found in {chapter_path}")
        return 0

    points = []
    for chunk in chunks:
        embedding = get_embedding(openai_client, chunk["text"])
        point = PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={
                "text": chunk["text"],
                "chapter_number": chunk["chapter_number"],
                "chapter_title": chunk["chapter_title"],
                "section_id": chunk["section_id"],
                "section_title": chunk["section_title"],
                "source_url": f"/docs/part{part_num}/chapter{chapter_num}#{chunk['section_id']}",
                "part_number": part_num,
                "token_count": len(chunk["text"].split()),  # Approximate
            },
        )
        points.append(point)

    qdrant_client.upsert(
        collection_name=settings.qdrant_collection,
        points=points,
    )

    print(f"  Indexed {len(points)} chunks from Chapter {chapter_num}")
    return len(points)


def index_all_chapters(book_source_path: str) -> None:
    """Index all chapters from the book source."""
    openai_client = OpenAI(api_key=settings.openai_api_key)
    qdrant_client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
    )

    book_source = Path(book_source_path)
    if not book_source.exists():
        print(f"Book source path not found: {book_source}")
        return

    total_chunks = 0

    # Chapter mapping (part, chapter, title)
    chapters = [
        (1, 1, "Introduction to Physical AI"),
        # Add more chapters as they become available
    ]

    for part_num, chapter_num, chapter_title in chapters:
        chapter_dir = book_source / f"part{part_num}" / f"chapter{chapter_num}"
        readme_path = chapter_dir / "README.md"

        if readme_path.exists():
            print(f"Indexing Part {part_num}, Chapter {chapter_num}: {chapter_title}")
            chunks = index_chapter(
                openai_client,
                qdrant_client,
                readme_path,
                chapter_num,
                chapter_title,
                part_num,
            )
            total_chunks += chunks

            # Also index any lesson files
            for lesson_file in chapter_dir.glob("*.md"):
                if lesson_file.name != "README.md":
                    lesson_title = lesson_file.stem.replace("-", " ").title()
                    print(f"  Indexing lesson: {lesson_file.name}")
                    chunks = index_chapter(
                        openai_client,
                        qdrant_client,
                        lesson_file,
                        chapter_num,
                        f"{chapter_title} - {lesson_title}",
                        part_num,
                    )
                    total_chunks += chunks
        else:
            print(f"Chapter not found: {readme_path}")

    print(f"\nTotal chunks indexed: {total_chunks}")


def index_chapter_1() -> None:
    """Index only Chapter 1 (for MVP)."""
    openai_client = OpenAI(api_key=settings.openai_api_key)
    qdrant_client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
    )

    # Path to Chapter 1 from backend directory (actual book structure)
    book_source = Path(__file__).parent.parent.parent / "book-source" / "docs"
    chapter_dir = book_source / "01-robotic-nervous-system" / "01-introduction-to-physical-ai"
    readme_path = chapter_dir / "README.md"

    if not chapter_dir.exists():
        print(f"Chapter 1 directory not found at: {chapter_dir}")
        return

    total_chunks = 0

    # Index README if exists
    if readme_path.exists():
        print("Indexing Chapter 1 README: Introduction to Physical AI")
        chunks = index_chapter(
            openai_client,
            qdrant_client,
            readme_path,
            chapter_num=1,
            chapter_title="Introduction to Physical AI",
            part_num=1,
        )
        total_chunks += chunks

    # Index all lesson files
    for lesson_file in sorted(chapter_dir.glob("*.md")):
        if lesson_file.name not in ["README.md", "_category_.json"]:
            # Extract lesson number and title from filename
            # e.g., "01-paradigm-shift.md" -> "01 Paradigm Shift"
            lesson_name = lesson_file.stem
            lesson_title = " ".join(
                word.title() for word in lesson_name.split("-")[1:]
            )
            print(f"Indexing lesson: {lesson_file.name}")
            chunks = index_chapter(
                openai_client,
                qdrant_client,
                lesson_file,
                chapter_num=1,
                chapter_title=f"Introduction to Physical AI - {lesson_title}",
                part_num=1,
            )
            total_chunks += chunks

    print(f"\nTotal chunks indexed: {total_chunks}")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Index book content into Qdrant")
    parser.add_argument(
        "--all",
        action="store_true",
        help="Index all chapters (default: Chapter 1 only)",
    )
    parser.add_argument(
        "--path",
        type=str,
        default="../book-source/docs",
        help="Path to book source directory",
    )

    args = parser.parse_args()

    if args.all:
        index_all_chapters(args.path)
    else:
        index_chapter_1()
