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


# Part slug mapping (folder name -> URL slug)
PART_SLUG_MAP = {
    "01-robotic-nervous-system": "robotic-nervous-system",
    "02-digital-twin": "digital-twin",
    "03-ai-robot-brain": "ai-robot-brain",
    "04-vision-language-action": "vision-language-action",
}

# Chapter slug mapping (folder name -> URL slug)
def get_url_slug(folder_name: str) -> str:
    """Convert folder name to URL slug by removing numeric prefix."""
    # e.g., "01-introduction-to-physical-ai" -> "introduction-to-physical-ai"
    parts = folder_name.split("-", 1)
    if len(parts) > 1 and parts[0].isdigit():
        return parts[1]
    return folder_name


def chunk_markdown(
    content: str,
    chapter_num: int,
    chapter_title: str,
    lesson_slug: str | None = None,
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
                                current_section.lower().replace(" ", "-").replace(":", "").replace("?", "")
                                or "intro"
                            ),
                            "text": text,
                            "chapter_number": chapter_num,
                            "chapter_title": chapter_title,
                            "lesson_slug": lesson_slug,
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
                        current_section.lower().replace(" ", "-").replace(":", "").replace("?", "")
                        or "intro"
                    ),
                    "text": text,
                    "chapter_number": chapter_num,
                    "chapter_title": chapter_title,
                    "lesson_slug": lesson_slug,
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


def build_source_url(
    part_slug: str,
    chapter_slug: str,
    lesson_slug: str | None,
    section_id: str,
) -> str:
    """Build the correct Docusaurus URL for a content chunk.

    URL pattern: /docs/{part-slug}/{chapter-slug}/{lesson-slug}#{section-id}
    Example: /docs/robotic-nervous-system/introduction-to-physical-ai/reality-gap#the-perfect-simulation-problem
    """
    if lesson_slug:
        return f"/docs/{part_slug}/{chapter_slug}/{lesson_slug}#{section_id}"
    else:
        return f"/docs/{part_slug}/{chapter_slug}#{section_id}"


def index_lesson(
    openai_client: OpenAI,
    qdrant_client: QdrantClient,
    lesson_path: Path,
    chapter_num: int,
    chapter_title: str,
    lesson_title: str,
    part_num: int,
    part_slug: str,
    chapter_slug: str,
    lesson_slug: str,
) -> int:
    """Index a lesson into Qdrant. Returns number of chunks indexed."""
    content = lesson_path.read_text(encoding="utf-8")
    chunks = chunk_markdown(
        content,
        chapter_num,
        f"{chapter_title} - {lesson_title}",
        lesson_slug,
    )

    if not chunks:
        print(f"  No chunks found in {lesson_path}")
        return 0

    points = []
    for chunk in chunks:
        embedding = get_embedding(openai_client, chunk["text"])
        source_url = build_source_url(
            part_slug,
            chapter_slug,
            lesson_slug,
            chunk["section_id"],
        )
        point = PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={
                "text": chunk["text"],
                "chapter_number": chunk["chapter_number"],
                "chapter_title": chunk["chapter_title"],
                "section_id": chunk["section_id"],
                "section_title": chunk["section_title"],
                "source_url": source_url,
                "part_number": part_num,
                "part_slug": part_slug,
                "chapter_slug": chapter_slug,
                "lesson_slug": lesson_slug,
                "lesson_title": lesson_title,
                "token_count": len(chunk["text"].split()),  # Approximate
            },
        )
        points.append(point)

    qdrant_client.upsert(
        collection_name=settings.qdrant_collection,
        points=points,
    )

    return len(points)


def index_chapter_readme(
    openai_client: OpenAI,
    qdrant_client: QdrantClient,
    readme_path: Path,
    chapter_num: int,
    chapter_title: str,
    part_num: int,
    part_slug: str,
    chapter_slug: str,
) -> int:
    """Index a chapter README into Qdrant. Returns number of chunks indexed."""
    content = readme_path.read_text(encoding="utf-8")
    chunks = chunk_markdown(content, chapter_num, chapter_title, None)

    if not chunks:
        print(f"  No chunks found in {readme_path}")
        return 0

    points = []
    for chunk in chunks:
        embedding = get_embedding(openai_client, chunk["text"])
        source_url = build_source_url(
            part_slug,
            chapter_slug,
            None,  # No lesson for chapter README
            chunk["section_id"],
        )
        point = PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={
                "text": chunk["text"],
                "chapter_number": chunk["chapter_number"],
                "chapter_title": chunk["chapter_title"],
                "section_id": chunk["section_id"],
                "section_title": chunk["section_title"],
                "source_url": source_url,
                "part_number": part_num,
                "part_slug": part_slug,
                "chapter_slug": chapter_slug,
                "lesson_slug": None,
                "lesson_title": None,
                "token_count": len(chunk["text"].split()),
            },
        )
        points.append(point)

    qdrant_client.upsert(
        collection_name=settings.qdrant_collection,
        points=points,
    )

    return len(points)


def clear_collection(qdrant_client: QdrantClient) -> None:
    """Clear all points from the collection before re-indexing."""
    try:
        # Delete and recreate collection
        from qdrant_client.http.models import Distance, VectorParams

        qdrant_client.delete_collection(settings.qdrant_collection)
        print(f"Deleted existing collection: {settings.qdrant_collection}")

        qdrant_client.create_collection(
            collection_name=settings.qdrant_collection,
            vectors_config=VectorParams(
                size=settings.embedding_dimensions,
                distance=Distance.COSINE,
            ),
        )
        print(f"Created fresh collection: {settings.qdrant_collection}")
    except Exception as e:
        print(f"Note: Could not clear collection: {e}")


def index_chapter_1() -> None:
    """Index only Chapter 1 (for MVP) with correct Docusaurus URLs."""
    openai_client = OpenAI(api_key=settings.openai_api_key)
    qdrant_client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
    )

    # Clear existing data first
    clear_collection(qdrant_client)

    # Path to Chapter 1 from backend directory (actual book structure)
    book_source = Path(__file__).parent.parent.parent / "book-source" / "docs"
    part_folder = "01-robotic-nervous-system"
    chapter_folder = "01-introduction-to-physical-ai"
    chapter_dir = book_source / part_folder / chapter_folder
    readme_path = chapter_dir / "README.md"

    if not chapter_dir.exists():
        print(f"Chapter 1 directory not found at: {chapter_dir}")
        return

    # URL slugs (matching Docusaurus routing)
    part_slug = get_url_slug(part_folder)      # "robotic-nervous-system"
    chapter_slug = get_url_slug(chapter_folder) # "introduction-to-physical-ai"

    total_chunks = 0

    # Index README if exists
    if readme_path.exists():
        print("Indexing Chapter 1 README: Introduction to Physical AI")
        chunks = index_chapter_readme(
            openai_client,
            qdrant_client,
            readme_path,
            chapter_num=1,
            chapter_title="Introduction to Physical AI",
            part_num=1,
            part_slug=part_slug,
            chapter_slug=chapter_slug,
        )
        total_chunks += chunks
        print(f"  Indexed {chunks} chunks")

    # Index all lesson files
    for lesson_file in sorted(chapter_dir.glob("*.md")):
        if lesson_file.name not in ["README.md", "_category_.json"]:
            # Extract lesson slug and title from filename
            # e.g., "01-paradigm-shift.md" -> slug="paradigm-shift", title="Paradigm Shift"
            lesson_folder_name = lesson_file.stem  # "01-paradigm-shift"
            lesson_slug = get_url_slug(lesson_folder_name)  # "paradigm-shift"
            lesson_title = " ".join(
                word.title() for word in lesson_slug.split("-")
            )

            print(f"Indexing lesson: {lesson_file.name}")
            print(f"  URL: /docs/{part_slug}/{chapter_slug}/{lesson_slug}")

            chunks = index_lesson(
                openai_client,
                qdrant_client,
                lesson_file,
                chapter_num=1,
                chapter_title="Introduction to Physical AI",
                lesson_title=lesson_title,
                part_num=1,
                part_slug=part_slug,
                chapter_slug=chapter_slug,
                lesson_slug=lesson_slug,
            )
            total_chunks += chunks
            print(f"  Indexed {chunks} chunks")

    print(f"\nTotal chunks indexed: {total_chunks}")


def index_all_chapters(book_source_path: str) -> None:
    """Index all chapters from the book source."""
    openai_client = OpenAI(api_key=settings.openai_api_key)
    qdrant_client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
    )

    # Clear existing data first
    clear_collection(qdrant_client)

    book_source = Path(book_source_path)
    if not book_source.exists():
        print(f"Book source path not found: {book_source}")
        return

    total_chunks = 0

    # Scan for part directories (XX-part-name)
    for part_dir in sorted(book_source.glob("*-*")):
        if not part_dir.is_dir():
            continue

        part_folder = part_dir.name
        part_slug = get_url_slug(part_folder)
        part_num = int(part_folder.split("-")[0])

        print(f"\n=== Part {part_num}: {part_slug} ===")

        # Scan for chapter directories within the part
        for chapter_dir in sorted(part_dir.glob("*-*")):
            if not chapter_dir.is_dir():
                continue

            chapter_folder = chapter_dir.name
            chapter_slug = get_url_slug(chapter_folder)
            chapter_num = int(chapter_folder.split("-")[0])
            chapter_title = " ".join(word.title() for word in chapter_slug.split("-"))

            print(f"\nChapter {chapter_num}: {chapter_title}")

            # Index README
            readme_path = chapter_dir / "README.md"
            if readme_path.exists():
                chunks = index_chapter_readme(
                    openai_client,
                    qdrant_client,
                    readme_path,
                    chapter_num,
                    chapter_title,
                    part_num,
                    part_slug,
                    chapter_slug,
                )
                total_chunks += chunks
                print(f"  README: {chunks} chunks")

            # Index lessons
            for lesson_file in sorted(chapter_dir.glob("*.md")):
                if lesson_file.name in ["README.md", "_category_.json", "index.md"]:
                    continue

                lesson_folder_name = lesson_file.stem
                lesson_slug = get_url_slug(lesson_folder_name)
                lesson_title = " ".join(word.title() for word in lesson_slug.split("-"))

                chunks = index_lesson(
                    openai_client,
                    qdrant_client,
                    lesson_file,
                    chapter_num,
                    chapter_title,
                    lesson_title,
                    part_num,
                    part_slug,
                    chapter_slug,
                    lesson_slug,
                )
                total_chunks += chunks
                print(f"  {lesson_slug}: {chunks} chunks")

    print(f"\n=== Total chunks indexed: {total_chunks} ===")


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
