"""
Working Memory Layer

Session-based short-term state management layer.
Accumulates work memories between checkpoints and generates compressed summaries.

Supports file-based persistence to maintain state across subprocess calls.
"""

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Optional


class WorkingMemory:
    """
    Session-based short-term memory layer.

    - Accumulates work memories (items) between checkpoints
    - Generates compressed summary at checkpoint
    - Summary is saved to Context Memory
    """

    # Priority weights for summary (defaults)
    DEFAULT_PRIORITY_WEIGHTS = {
        "error_fix": 10,  # Error→fix pattern
        "decision": 8,  # Decisions
        "change": 5,  # File changes
        "read": 1,  # Simple reads
    }

    def __init__(self, config: dict):
        """
        Args:
            config: Configuration dictionary (see working section)
        """
        self.config = config
        working_config = config.get("working", {})

        # Config values
        self._max_items: int = working_config.get("max_items", 50)
        self._checkpoint_interval: int = working_config.get(
            "auto_checkpoint_interval", 3
        )
        self._priority_weights: dict = working_config.get("summary", {}).get(
            "priority_weights", self.DEFAULT_PRIORITY_WEIGHTS
        )

        # Persistence settings
        self._persist = working_config.get("persist", False)
        self._persist_path: Path | None = None
        if self._persist:
            persist_path_str = working_config.get(
                "persist_path", "data/memory/working_memory.json"
            )
            self._persist_path = Path(persist_path_str)
            self._persist_path.parent.mkdir(parents=True, exist_ok=True)

        # Initialize state
        self._session_id: str | None = None
        self._task: str | None = None
        self._idle_count: int = 0
        self._last_checkpoint: datetime | None = None

        # Work memories (accumulate between checkpoints)
        self._items: list[dict] = []

        # Key-value store for legacy compatibility
        self._state: dict[str, Any] = {}

        # Load existing state if persistence enabled
        if self._persist and self._persist_path and self._persist_path.exists():
            self._load_from_file()

    # ═══════════════════════════════════════════════════════════════
    # Session/Task Management
    # ═══════════════════════════════════════════════════════════════

    def init(self, session_id: str):
        """
        Initialize session.

        Args:
            session_id: Session ID
        """
        self._session_id = session_id
        self._task = None
        self._idle_count = 0
        self._last_checkpoint = datetime.now()
        self._items = []
        self._state = {
            "session_id": session_id,
            "started_at": datetime.now().isoformat(),
        }
        self._save_to_file()

    def start_task(self, task: str):
        """
        Start task.

        Args:
            task: Task description
        """
        self._task = task
        self._idle_count = 0
        self._state["current_task"] = task
        self._state["task_started_at"] = datetime.now().isoformat()
        self._save_to_file()

    def end_task(self):
        """End task."""
        self._task = None
        self._state.pop("current_task", None)
        self._state.pop("task_started_at", None)
        self._save_to_file()

    # ═══════════════════════════════════════════════════════════════
    # Work Memory Management (checkpoint-based)
    # ═══════════════════════════════════════════════════════════════

    def add_item(
        self,
        item_type: str,
        content: str,
        importance: str = "low",
        metadata: dict | None = None,
    ):
        """
        Add work memory item.

        Args:
            item_type: Item type
                - error: Error occurred
                - fix: Error fixed
                - decision: Decision made
                - change: File changed
                - read: File read
                - note: Other notes
            content: Content
            importance: Importance level (high, medium, low)
            metadata: Additional metadata (e.g., {"file": "path/to/file.py"})
        """
        # Enforce max items limit
        if len(self._items) >= self._max_items:
            # Remove low importance items first
            low_importance = [
                i for i, item in enumerate(self._items) if item["importance"] == "low"
            ]
            if low_importance:
                self._items.pop(low_importance[0])
            else:
                self._items.pop(0)  # Remove oldest item

        self._items.append(
            {
                "type": item_type,
                "content": content,
                "importance": importance,
                "metadata": metadata or {},
                "timestamp": datetime.now().isoformat(),
            }
        )
        self._save_to_file()

    def summarize_since_checkpoint(self) -> str:
        """
        Generate compressed summary of work memories since checkpoint.

        Priority:
        1. error → fix: Problem and solution
        2. decision: Architecture/design choices
        3. change: File creation/modification
        4. read: Simple reads (minimized)

        Returns:
            str: Compressed summary string
        """
        if not self._items:
            if self._task:
                return f"Task in progress: {self._task}"
            return ""

        summaries = []

        # REQ-AM-01: Prioritize decision, fix, change
        # 1. Extract error → fix patterns
        errors = [i for i in self._items if i["type"] == "error"]
        fixes = [i for i in self._items if i["type"] == "fix"]
        if errors and fixes:
            # Most recent error-fix pair
            error_content = errors[-1]["content"][:100]
            fix_content = fixes[-1]["content"][:100]
            summaries.append(f"Problem solved: {error_content} → {fix_content}")
        elif errors:
            # Unresolved error
            summaries.append(f"Error occurred: {errors[-1]['content'][:100]}")

        # 2. Decisions (max 3)
        decisions = [i for i in self._items if i["type"] == "decision"]
        for d in decisions[-3:]:
            summaries.append(f"Decision: {d['content'][:100]}")

        # 3. Changes (summarize as file list)
        changes = [i for i in self._items if i["type"] == "change"]
        if changes:
            # Extract file names
            files = []
            for c in changes:
                file_path = c.get("metadata", {}).get("file")
                if file_path:
                    files.append(file_path.split("/")[-1])  # Filename only
                else:
                    files.append(c["content"][:30])
            unique_files = list(dict.fromkeys(files))  # Remove duplicates, keep order
            summaries.append(f"Changed: {', '.join(unique_files[:10])}")

        # 4. Other high importance items
        high_items = [
            i
            for i in self._items
            if i["importance"] == "high"
            and i["type"] not in ("error", "fix", "decision", "change")
        ]
        for h in high_items[-3:]:
            summaries.append(f"{h['type'].capitalize()}: {h['content'][:80]}")

        # REQ-AM-02: Minimize 'read' logs
        # Only include 'read' if there are no other significant items or if explicitly high importance
        if not summaries:
            reads = [i for i in self._items if i["type"] == "read"]
            if reads:
                files = [
                    r.get("metadata", {}).get("file", r["content"][:30]).split("/")[-1]
                    for r in reads[-5:]
                ]
                summaries.append(f"Analyzed: {', '.join(list(dict.fromkeys(files)))}")

        if summaries:
            return " | ".join(summaries)

        # Fallback: Task context
        if self._task:
            return f"Task progress: {self._task} ({len(self._items)} items)"
        return f"Session activity: {len(self._items)} items"

    def clear_since_checkpoint(self):
        """Clear work memories after checkpoint."""
        self._items = []
        self._last_checkpoint = datetime.now()
        self._idle_count = 0
        self._save_to_file()

    def get_items(self) -> list[dict]:
        """Get current work memory items list."""
        return self._items.copy()

    def get_items_count(self) -> int:
        """Get work memory items count."""
        return len(self._items)

    # ═══════════════════════════════════════════════════════════════
    # Legacy Compatibility: Key-value Store
    # ═══════════════════════════════════════════════════════════════

    def set(self, key: str, value: Any):
        """
        Set state (legacy compatibility).

        Args:
            key: Key
            value: Value
        """
        self._state[key] = value
        self._save_to_file()

    def get(self, key: str, default: Any = None) -> Any:
        """
        Get state (legacy compatibility).

        Args:
            key: Key
            default: Default value

        Returns:
            Stored value or default
        """
        return self._state.get(key, default)

    def keys(self) -> list[str]:
        """Get all keys list (legacy compatibility)."""
        return list(self._state.keys())

    def count(self) -> int:
        """Get stored items count (legacy compatibility)."""
        return len(self._state)

    def keep_only(self, keys: list[str]):
        """
        Keep only specified keys (legacy compatibility).

        Args:
            keys: Keys to keep
        """
        system_keys = {"session_id", "started_at", "current_task", "task_started_at"}
        keys_to_keep = set(keys) | system_keys
        self._state = {k: v for k, v in self._state.items() if k in keys_to_keep}
        self._save_to_file()

    def clear(self):
        """Clear all."""
        session_id = self._session_id
        self._items = []
        self._state = {
            "session_id": session_id,
            "started_at": datetime.now().isoformat(),
        }
        self._task = None
        self._idle_count = 0
        self._last_checkpoint = datetime.now()
        self._save_to_file()

    # ═══════════════════════════════════════════════════════════════
    # Idle Count Management
    # ═══════════════════════════════════════════════════════════════

    def increment_idle(self) -> bool:
        """
        Increment idle count.

        Returns:
            bool: Whether checkpoint is needed
        """
        self._idle_count += 1
        self._save_to_file()
        return self._idle_count >= self._checkpoint_interval

    def reset_idle(self):
        """Reset idle count."""
        self._idle_count = 0
        self._save_to_file()

    # ═══════════════════════════════════════════════════════════════
    # Properties and Status
    # ═══════════════════════════════════════════════════════════════

    @property
    def current_task(self) -> Optional[str]:
        """Current task."""
        return self._task

    @property
    def session_id(self) -> Optional[str]:
        """Session ID."""
        return self._session_id

    @property
    def last_checkpoint(self) -> Optional[datetime]:
        """Last checkpoint time."""
        return self._last_checkpoint

    def get_state_snapshot(self) -> dict:
        """Get current state snapshot."""
        return {
            "session_id": self._session_id,
            "current_task": self._task,
            "idle_count": self._idle_count,
            "items_count": len(self._items),
            "last_checkpoint": (
                self._last_checkpoint.isoformat() if self._last_checkpoint else None
            ),
            "state": self._state.copy(),
        }

    def get_summary(self) -> dict:
        """Get status summary."""
        return {
            "session_id": self._session_id,
            "current_task": self._task,
            "items_count": len(self._items),
            "state_count": len(self._state),
            "idle_count": self._idle_count,
            "checkpoint_interval": self._checkpoint_interval,
            "last_checkpoint": (
                self._last_checkpoint.isoformat() if self._last_checkpoint else None
            ),
        }

    # ═══════════════════════════════════════════════════════════════
    # File-based Persistence
    # ═══════════════════════════════════════════════════════════════

    def _load_from_file(self):
        """Load state from file."""
        if not self._persist_path or not self._persist_path.exists():
            return

        try:
            with open(self._persist_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            self._session_id = data.get("session_id")
            self._task = data.get("task")
            self._idle_count = data.get("idle_count", 0)
            self._items = data.get("items", [])
            self._state = data.get("state", {})

            last_checkpoint_str = data.get("last_checkpoint")
            if last_checkpoint_str:
                self._last_checkpoint = datetime.fromisoformat(last_checkpoint_str)
        except (json.JSONDecodeError, KeyError, ValueError):
            # Ignore corrupted files and start fresh
            pass

    def _save_to_file(self):
        """Save state to file."""
        if not self._persist or not self._persist_path:
            return

        data = {
            "session_id": self._session_id,
            "task": self._task,
            "idle_count": self._idle_count,
            "items": self._items,
            "state": self._state,
            "last_checkpoint": (
                self._last_checkpoint.isoformat() if self._last_checkpoint else None
            ),
            "saved_at": datetime.now().isoformat(),
        }

        try:
            with open(self._persist_path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2, default=str)
        except OSError:
            # Ignore save failures (could add logging)
            pass
