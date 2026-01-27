"""
OpenCode Memory - 2-layer memory system for AI agents.

Provides:
- ContextMemory: Long-term semantic memory with OpenMemory
- WorkingMemory: Session-based short-term memory with persistence
- ContextManager: Unified interface for both layers
"""

from opencode_memory.config import load_config, validate_config
from opencode_memory.context_manager import ContextManager
from opencode_memory.context_memory import ContextMemory, is_meaningful_content
from opencode_memory.working_memory import WorkingMemory

__version__ = "0.1.0"
__all__ = [
    "ContextManager",
    "ContextMemory",
    "WorkingMemory",
    "load_config",
    "validate_config",
    "is_meaningful_content",
]
