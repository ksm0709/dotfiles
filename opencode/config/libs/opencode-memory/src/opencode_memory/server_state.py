"""
Server State Management

Manages the global state of the API server, including:
- Shared ContextMemory (Long-term memory)
- Session-specific WorkingMemory
- Configuration
"""

import logging
import os
import time
import uuid
from typing import Dict, Optional

from .config import load_config
from .context_memory import ContextMemory
from .working_memory import WorkingMemory

logger = logging.getLogger(__name__)


class ServerState:
    """
    Singleton state manager for the API server.
    """

    _instance: Optional["ServerState"] = None

    def __init__(self):
        self.config = load_config()

        # Shared resources
        self.context_memory = ContextMemory(self.config)

        # Session isolation
        # Dict[session_id, WorkingMemory]
        self.sessions: Dict[str, WorkingMemory] = {}

        # Lifecycle tracking
        self.last_activity = time.time()
        self.started_at = time.time()

        # Security
        self.api_key = os.environ.get("CONTEXT_SERVER_KEY") or str(uuid.uuid4())
        self.port: int = 0  # Assigned at startup

    @classmethod
    def get_instance(cls) -> "ServerState":
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def get_working_memory(self, session_id: str) -> WorkingMemory:
        """Get or create working memory for a session."""
        self.touch()
        if session_id not in self.sessions:
            logger.info(f"Creating new working memory for session: {session_id}")
            wm = WorkingMemory(self.config)
            wm.init(session_id)
            self.sessions[session_id] = wm
        return self.sessions[session_id]

    def touch(self):
        """Update last activity timestamp."""
        self.last_activity = time.time()

    def is_idle(self, timeout_seconds: int) -> bool:
        """Check if server has been idle for longer than timeout."""
        return (time.time() - self.last_activity) > timeout_seconds
