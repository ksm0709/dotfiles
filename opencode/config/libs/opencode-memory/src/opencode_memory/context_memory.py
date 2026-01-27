"""
Context Memory Layer

OpenMemory-based long-term memory storage/retrieval layer.
Retrieves related memories via semantic search and stores checkpoint summaries.
"""

import asyncio
import fcntl
import logging
import os
import random
import re
import time
from contextlib import contextmanager
from datetime import datetime
from functools import wraps
from pathlib import Path
from typing import Any, Callable, Dict, Optional, TypeVar, cast

from .utils.llm import LLMClient

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════════════════
# Content Filter - Filter meaningless content
# ═══════════════════════════════════════════════════════════════════════════

# Prohibited patterns (regex)
NOISE_PATTERNS = [
    r"^(in\s*)?(progress|processing|loading|waiting)\.{0,3}$",
    r"^\.{2,}$",  # "...", "...."
    r"^(checking|reviewing|analyzing)\.{0,3}$",
    r"^(work|task)\s*(in\s*)?(progress|started|completed)?\.{0,3}$",
    r"^test.*$",  # Test data
    r"^\s*$",  # Empty string
]

# Minimum content length (shorter is considered meaningless)
MIN_CONTENT_LENGTH = 10

# Minimum meaningful word count
MIN_MEANINGFUL_WORDS = 3

# Compiled pattern cache
_compiled_patterns: list[re.Pattern] | None = None


def _get_compiled_patterns() -> list[re.Pattern]:
    """Return compiled noise patterns (cached)."""
    global _compiled_patterns
    if _compiled_patterns is None:
        _compiled_patterns = [
            re.compile(p, re.IGNORECASE | re.UNICODE) for p in NOISE_PATTERNS
        ]
    return _compiled_patterns


def is_meaningful_content(content: str) -> bool:
    """
    Determine if content is worth saving.

    Args:
        content: Content to check

    Returns:
        bool: True if meaningful content
    """
    if not content:
        return False

    # Check length after stripping whitespace
    stripped = content.strip()
    if len(stripped) < MIN_CONTENT_LENGTH:
        return False

    # Match noise patterns
    for pattern in _get_compiled_patterns():
        if pattern.match(stripped):
            return False

    # Check meaningful word count (Korean/English words)
    words = re.findall(r"[가-힣a-zA-Z]+", stripped)
    if len(words) < MIN_MEANINGFUL_WORDS:
        return False

    return True


# ═══════════════════════════════════════════════════════════════════════════
# SQLite Race Condition Handling
# ═══════════════════════════════════════════════════════════════════════════

# Retry settings
MAX_RETRIES = 5
BASE_DELAY = 0.1  # 100ms
MAX_DELAY = 2.0  # 2s

T = TypeVar("T")


def with_retry(
    max_retries: int = MAX_RETRIES,
    base_delay: float = BASE_DELAY,
    max_delay: float = MAX_DELAY,
    retryable_exceptions: tuple = (Exception,),
) -> Callable:
    """
    Retry decorator for OpenMemory operations.

    Applies exponential backoff with jitter to minimize
    collisions during concurrent access.

    Args:
        max_retries: Maximum retry count
        base_delay: Base wait time (seconds)
        max_delay: Maximum wait time (seconds)
        retryable_exceptions: Exception types to retry
    """

    def decorator(func: Callable[..., T]) -> Callable[..., T]:
        @wraps(func)
        def wrapper(*args, **kwargs) -> T:
            last_exception = None
            for attempt in range(max_retries):
                try:
                    return func(*args, **kwargs)
                except retryable_exceptions as e:
                    last_exception = e
                    error_msg = str(e).lower()

                    # Only retry on temporary errors specific to OpenMemory
                    retryable_errors = {
                        # Network/Connection issues
                        "timeout",
                        "connection",
                        "network",
                        "temporary",
                        "unavailable",
                        # Database/Storage issues
                        "database",
                        "storage",
                        "disk",
                        "space",
                        "corrupt",
                        # Concurrency issues
                        "locked",
                        "busy",
                        "conflict",
                        "concurrent",
                        "race",
                        # OpenMemory specific issues
                        "embedding",
                        "vector",
                        "index",
                        "search",
                        "ollama",
                        # System issues
                        "resource",
                        "memory",
                        "cpu",
                        "overload",
                    }

                    # Also check exception type for specific retryable cases
                    exception_type = type(e).__name__.lower()
                    retryable_exception_types = {
                        "timeouterror",
                        "connectionerror",
                        "operationalerror",
                        "ioerror",
                        "oserror",
                        "interruptederror",
                    }

                    # Check for permanent errors that should not be retried
                    permanent_errors = {
                        "not found",
                        "syntax",
                        "permission",
                        "no such",
                        "invalid",
                        "authentication",
                        "unauthorized",
                    }

                    is_permanent = any(err in error_msg for err in permanent_errors)

                    should_retry = not is_permanent and (
                        any(error in error_msg for error in retryable_errors)
                        or any(
                            exc_type in exception_type
                            for exc_type in retryable_exception_types
                        )
                    )

                    if should_retry:
                        # Exponential backoff + jitter
                        delay = min(base_delay * (2**attempt), max_delay)
                        jitter = random.uniform(0, delay * 0.1)
                        actual_delay = delay + jitter

                        # Categorize error for better logging
                        error_category = "unknown"
                        if any(error in error_msg for error in ["timeout", "time"]):
                            error_category = "timeout"
                        elif any(
                            error in error_msg for error in ["connection", "network"]
                        ):
                            error_category = "connection"
                        elif any(
                            error in error_msg
                            for error in ["locked", "busy", "conflict"]
                        ):
                            error_category = "concurrency"
                        elif any(
                            error in error_msg
                            for error in ["embedding", "vector", "search"]
                        ):
                            error_category = "vector_operation"
                        elif any(
                            error in error_msg
                            for error in ["database", "storage", "disk"]
                        ):
                            error_category = "storage"

                        logger.warning(
                            f"OpenMemory {error_category} error, retry {attempt + 1}/{max_retries} "
                            f"after {actual_delay:.2f}s: {type(e).__name__}: {e}"
                        )
                        time.sleep(actual_delay)
                    else:
                        # Raise other errors immediately
                        raise

            # All retries failed
            logger.error(f"All {max_retries} retries failed: {last_exception}")
            if last_exception:
                raise last_exception
            raise Exception(f"All {max_retries} retries failed")

        return wrapper

    return decorator


@contextmanager
def file_lock(lock_path: Path, timeout: float = 10.0):
    """
    File-based lock (flock).

    Prevents concurrent access between multiple processes.
    Graceful fallback on Windows where flock doesn't work.

    Args:
        lock_path: Lock file path
        timeout: Lock acquisition timeout (seconds)
    """
    lock_file = None
    try:
        lock_file = open(lock_path, "w")
        start_time = time.time()

        while True:
            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except (IOError, OSError):
                if time.time() - start_time > timeout:
                    raise TimeoutError(f"Failed to acquire lock: {lock_path}")
                time.sleep(0.1)

        yield

    finally:
        if lock_file:
            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
            except Exception:
                pass
            lock_file.close()


def _run_async(coro):
    """Run an async coroutine synchronously."""
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        loop = None

    if loop and loop.is_running():
        import concurrent.futures

        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(asyncio.run, coro)
            return future.result()
    else:
        return asyncio.run(coro)


class ContextMemory:
    """
    OpenMemory-based long-term memory layer.

    Stores agent's learnings, analysis results, user preferences, etc.
    in a semantically searchable format.

    Main features:
    - query(): Search related memories
    - add(): Store memory
    - query_and_update(): Atomic search + save (for checkpoints)
    """

    def __init__(self, config: dict):
        """
        Args:
            config: Configuration dictionary (see database, context sections)
        """
        self.config = config
        self._mem: Optional[Any] = None
        self._user_id = "context-manager"
        self._initialized = False
        self._current_task: str | None = None
        self._llm = LLMClient(config)

        # Ensure database directory
        db_path = config.get("database", {}).get("path")
        if db_path:
            # Use the directory containing the database file as working directory
            self._working_dir = Path(db_path).parent
            self._working_dir.mkdir(parents=True, exist_ok=True)
        else:
            self._working_dir = Path.cwd() / "data" / "memory"
            self._working_dir.mkdir(parents=True, exist_ok=True)

        # File lock path (for write operation synchronization)
        # Use absolute path to avoid path resolution issues
        self._lock_path = self._working_dir.resolve() / ".context_memory.lock"
        self._state_path = self._working_dir.resolve() / ".context_state.json"

        # Load persisted state
        self._load_state()

        # Ensure initialized on startup
        self._ensure_initialized()

    def _load_state(self):
        """Load persisted state (current task, etc.)."""
        if self._state_path.exists():
            try:
                import json

                with open(self._state_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    self._current_task = data.get("current_task")
            except Exception:
                pass

    def _save_state(self):
        """Save state to file."""
        try:
            import json

            data = {"current_task": self._current_task}
            with open(self._state_path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception:
            pass

    def _ensure_initialized(self):
        """Ensure OpenMemory is initialized."""
        if self._initialized and self._mem is not None:
            return

        try:
            from openmemory import Memory

            # Change working directory
            original_cwd = os.getcwd()
            os.chdir(str(self._working_dir))

            try:
                self._mem = Memory(user=self._user_id)
                self._initialized = True
            except Exception as e:
                logger.error(f"OpenMemory initialization failed: {e}")
                self._mem = None
                self._initialized = True  # Mark as tried
            finally:
                os.chdir(original_cwd)

        except ImportError as e:
            error_msg = str(e)
            if "dotenv" in error_msg:
                logger.warning(
                    "python-dotenv not installed. Run: pip install python-dotenv"
                )
            elif "langchain" in error_msg.lower() or "BaseMessage" in error_msg:
                logger.warning(
                    "langchain-core not installed. Run: pip install langchain-core"
                )
            else:
                logger.warning(f"OpenMemory import failed: {error_msg}")
            # Fallback when OpenMemory not installed
            self._mem = None
            self._initialized = True

    def set_current_task(self, task: str | None):
        """Set current task (for tag generation)."""
        self._current_task = task
        self._save_state()

    # ═══════════════════════════════════════════════════════════════
    # Core Methods
    # ═══════════════════════════════════════════════════════════════

    def add(
        self,
        content: str,
        tags: list[str] | None = None,
        metadata: dict | None = None,
    ) -> dict:
        """
        Add memory.

        Args:
            content: Content to store
            tags: Tag list
            metadata: Additional metadata

        Returns:
            dict: Save result
        """
        # Filter meaningless content
        if not is_meaningful_content(content):
            logger.debug(f"Filtered noise content: {content[:50]}...")
            return {"status": "filtered", "reason": "noise content"}

        # REQ-CR-01 & REQ-CR-02: Contextual Retrieval (Prepend context)
        if metadata and "task_goal" in metadata:
            task_goal = metadata["task_goal"]
            content = f"[Context: {task_goal}] {content}"

        self._ensure_initialized()

        if not self._mem:
            return {"status": "skipped", "reason": "OpenMemory not available"}

        async def _add():
            if self._mem:
                kwargs: Dict[str, Any] = {}
                if tags:
                    kwargs["tags"] = tags
                if metadata:
                    kwargs["metadata"] = metadata

                result = await self._mem.add(content, user_id=self._user_id, **kwargs)
                return result
            return None

        @with_retry()
        def _do_add():
            original_cwd = os.getcwd()
            # Use relative lock path after changing directory
            relative_lock_path = self._lock_path.name
            os.chdir(str(self._working_dir))
            try:
                # Synchronize write operations with file lock
                with file_lock(Path(relative_lock_path)):
                    return _run_async(_add())
            finally:
                os.chdir(original_cwd)

        try:
            result = _do_add()
            if result:
                return {
                    "status": "success",
                    "id": result.get("id") or result.get("root_memory_id"),
                    "content": content[:100] + "..." if len(content) > 100 else content,
                    "tags": tags,
                }
            else:
                return {"status": "skipped", "reason": "OpenMemory not available"}
        except Exception as e:
            logger.error(f"Failed to add memory: {e}")
            return {"status": "error", "message": str(e)}

    def query(
        self,
        question: str,
        limit: int | None = None,
        tags: list[str] | None = None,
        auto_cleanup: bool = True,
    ) -> list[dict]:
        """
        Search memories.

        Args:
            question: Search query
            limit: Maximum result count
            tags: Tag filter (currently not supported)
            auto_cleanup: Auto-delete meaningless memories

        Returns:
            list: List of related memories
        """
        self._ensure_initialized()

        if not self._mem:
            return []

        if limit is None:
            limit = (
                self.config.get("context", {})
                .get("search", {})
                .get("default_limit", 10)
            )

        # Ensure limit is an integer
        if limit is None:
            limit = 10

        min_score = (
            self.config.get("context", {}).get("search", {}).get("min_score", 0.3)
        )

        # REQ-QE-01: Task-Aware Query Expansion (HyDE)
        # If the question looks like a task or is short, expand it
        queries = [question]
        if len(question.split()) < 10:  # Simple heuristic for "short/task-like"
            expanded = self._expand_query(question)
            if expanded:
                queries.extend(expanded)
                # Remove duplicates while preserving order
                queries = list(dict.fromkeys(queries))

        async def _query_all():
            if not self._mem:
                return []

            all_results = []
            # REQ-QE-02: Multi-query search and integration
            for q in queries:
                results = await self._mem.search(
                    q,
                    user_id=self._user_id,
                    limit=limit * 2,  # Fetch more for reranking
                )
                all_results.extend(results)

            # Deduplicate by ID
            seen_ids = set()
            unique_results = []
            for r in all_results:
                rid = r.get("id")
                if rid not in seen_ids:
                    unique_results.append(r)
                    seen_ids.add(rid)

            return unique_results

        original_cwd = os.getcwd()
        os.chdir(str(self._working_dir))

        try:
            results = _run_async(_query_all())

            # Normalize results and filter noise
            normalized: list[dict] = []
            noise_ids = []

            for r in results:
                score = r.get("score", 0)
                content = r.get("content", "")
                memory_id = r.get("id")

                if not is_meaningful_content(content):
                    if memory_id and auto_cleanup:
                        noise_ids.append(memory_id)
                    continue

                if score >= min_score:
                    normalized.append(
                        {
                            "id": memory_id,
                            "content": content,
                            "score": score,
                            "tags": r.get("tags", []),
                            "metadata": r.get("metadata", {}),
                            "created_at": r.get("created_at"),
                        }
                    )

            # REQ-SR-02 & REQ-SR-03: 2-Stage Retrieval (Reranking)
            if normalized and self._current_task:
                normalized = self._llm_rerank(normalized, self._current_task)

            # Schedule async deletion of meaningless memories (background)
            if noise_ids:
                self._schedule_cleanup(noise_ids)

            return normalized[:limit]

        except Exception as e:
            logger.error(f"Query failed: {e}")
            return []
        finally:
            os.chdir(original_cwd)

    def _expand_query(self, question: str) -> list[str]:
        """
        Expand query using LLM (HyDE).
        """
        return self._llm.generate_queries(question)

    def _llm_rerank(self, results: list[dict], task: str) -> list[dict]:
        """
        Rerank results using LLM based on task relevance.
        """
        return self._llm.rerank(task, results)

    def _schedule_cleanup(self, memory_ids: list[str]):
        """
        Schedule meaningless memory deletion (background).

        Currently synchronous, but limited to prevent hanging.
        """
        # Limit cleanup to 3 items per query to prevent hanging
        for memory_id in memory_ids[:3]:
            try:
                self.delete(memory_id)
                logger.info(f"Cleaned up noise memory: {memory_id}")
            except Exception as e:
                logger.warning(f"Failed to cleanup memory {memory_id}: {e}")

    def delete(self, memory_id: str) -> dict:
        """
        Delete memory.

        Args:
            memory_id: Memory ID to delete

        Returns:
            dict: Delete result
        """
        self._ensure_initialized()

        if not self._mem:
            return {"status": "skipped", "reason": "OpenMemory not available"}

        async def _delete():
            if self._mem:
                await self._mem.delete(memory_id)
                return {"status": "success", "id": memory_id}
            return {"status": "skipped", "reason": "OpenMemory not available"}

        @with_retry()
        def _do_delete():
            original_cwd = os.getcwd()
            # Use relative lock path after changing directory
            relative_lock_path = self._lock_path.name
            os.chdir(str(self._working_dir))
            try:
                with file_lock(Path(relative_lock_path)):
                    return _run_async(_delete())
            finally:
                os.chdir(original_cwd)

        try:
            return cast(dict, _do_delete())
        except Exception as e:
            logger.error(f"Failed to delete memory {memory_id}: {e}")
            return {"status": "error", "message": str(e)}

    # ═══════════════════════════════════════════════════════════════
    # Checkpoint Methods (2-layer core)
    # ═══════════════════════════════════════════════════════════════

    def query_and_update(self, summary: str) -> dict:
        """
        For checkpoints: Atomic search + save.

        1. Search related past memories with summary
        2. Save summary as new memory
        3. Return related memories

        Args:
            summary: Compressed working memory summary

        Returns:
            dict: {
                "related_memories": list[dict],  # Related past memories
                "saved": dict,                   # Save result
            }
        """
        # 1. Search
        related = self.query(summary, limit=5)

        # 2. Save
        tags = ["checkpoint"]
        if self._current_task:
            # Tag length limit (30 chars)
            task_tag = self._current_task[:30].replace(" ", "_")
            tags.append(f"task:{task_tag}")

        metadata = {
            "type": "checkpoint",
            "timestamp": datetime.now().isoformat(),
            "task": self._current_task,
        }

        saved = self.add(summary, tags=tags, metadata=metadata)

        return {
            "related_memories": related,
            "saved": saved,
        }

    def get_for_task(self, task: str, limit: int = 5) -> list[dict]:
        """
        Get task-related memories.

        Args:
            task: Task description
            limit: Maximum result count

        Returns:
            list: List of related memories
        """
        return self.query(task, limit=limit)

    def save_checkpoint(self, summary: str, task: str | None = None) -> dict:
        """
        Save checkpoint (legacy compatibility).

        Args:
            summary: Progress summary
            task: Current task (for tags)

        Returns:
            dict: Save result
        """
        tags = ["checkpoint"]
        if task:
            tags.append(f"task:{task[:50]}")

        metadata = {
            "type": "checkpoint",
            "timestamp": datetime.now().isoformat(),
        }

        return self.add(summary, tags=tags, metadata=metadata)

    def save_result(self, result: str, task: str | None = None) -> dict:
        """
        Save task result.

        Args:
            result: Result summary
            task: Task description

        Returns:
            dict: Save result
        """
        tags = ["result"]
        if task:
            tags.append(f"task:{task[:50]}")

        metadata = {
            "type": "task_result",
            "timestamp": datetime.now().isoformat(),
        }

        return self.add(result, tags=tags, metadata=metadata)

    # ═══════════════════════════════════════════════════════════════
    # Status
    # ═══════════════════════════════════════════════════════════════

    def get_summary(self) -> dict:
        """Status summary."""
        return {
            "initialized": self._initialized,
            "available": self._mem is not None,
            "working_dir": str(self._working_dir),
            "current_task": self._current_task,
        }
