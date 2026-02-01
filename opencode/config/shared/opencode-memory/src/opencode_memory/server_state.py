"""
Server State Management

Manages the global state of the API server, including:
- Shared ContextMemory (Long-term memory)
- Session-specific WorkingMemory
- SemanticRecordStore (Semantic Memory)
- Configuration
"""

import logging
import os
import time
import uuid
from typing import Dict, Optional

from .config import get_project_root, load_config
from .context_memory import ContextMemory
from .services.boundary_detector import create_detector
from .services.episode_manager import EpisodeManager
from .services.problem_tracker import ProblemTracker
from .services.reflection_engine import ReflectionEngine
from .services.semantic_extractor import SemanticExtractor
from .storage.episode_store import EpisodeStore
from .storage.semantic_store import SemanticRecordStore
from .utils.llm import AsyncLLMClient
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

        # Storage
        project_root = get_project_root()

        # Semantic Memory Store
        semantic_db_path = self.config.get("semantic_database", {}).get(
            "path", str(project_root / "data" / "memory" / "semantic.sqlite")
        )
        self.semantic_store = SemanticRecordStore(db_path=semantic_db_path)
        self.semantic_store.initialize()

        # Episode Store (Phase 3)
        episode_db_path = self.config.get("episode_database", {}).get(
            "path", str(project_root / "data" / "memory" / "episodes.sqlite")
        )
        self.episode_store = EpisodeStore(db_path=episode_db_path)
        self.episode_store.initialize()

        # Services
        # Problem Tracker (Phase 2)
        self.problem_tracker = ProblemTracker()

        # LLM + Semantic Extractor (Phase 5)
        self.llm_client = AsyncLLMClient(self.config)
        self.semantic_extractor = SemanticExtractor(self.llm_client)

        llm_config = self.config.get("llm", {})
        enable_reflection = llm_config.get("enable_reflection", True)
        enable_boundary_detection = llm_config.get("enable_boundary_detection", True)

        # Reflection Engine (Phase 4)
        self.reflection_engine = ReflectionEngine(
            self.context_memory,
            semantic_extractor=self.semantic_extractor,
            enable_llm_reflection=enable_reflection,
        )

        # Episode Manager (Phase 3)
        self.episode_manager = EpisodeManager(
            store=self.episode_store, reflection_engine=self.reflection_engine
        )

        # Boundary Detector (Phase 3)
        self.boundary_detector = create_detector(
            time_threshold_minutes=30,
            use_llm=self.llm_client.enabled and enable_boundary_detection,
            semantic_extractor=self.semantic_extractor,
        )

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
