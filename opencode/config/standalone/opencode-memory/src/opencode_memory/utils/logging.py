"""
Logging Configuration Module

Provides centralized logging setup for client and server.
"""

import logging
import sys
from logging.handlers import RotatingFileHandler
from pathlib import Path

from ..config import get_project_root

# Constants
LOG_FORMAT = "[%(asctime)s] [%(levelname)s] [%(process)d] [%(name)s] %(message)s"
MAX_BYTES = 5 * 1024 * 1024  # 5MB
BACKUP_COUNT = 3


def _get_log_dir() -> Path:
    """Get log directory path."""
    root = get_project_root()
    log_dir = root / "data" / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    return log_dir


def setup_logger(
    name: str,
    log_file: str,
    level: int = logging.INFO,
    capture_stdout: bool = False,
) -> logging.Logger:
    """
    Setup logger with file rotation.

    Args:
        name: Logger name
        log_file: Log filename (e.g., 'client.log')
        level: Logging level
        capture_stdout: Whether to capture stdout/stderr (for server)

    Returns:
        logging.Logger: Configured logger
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Avoid adding duplicate handlers
    if logger.handlers:
        return logger

    log_path = _get_log_dir() / log_file

    # File Handler (Rotating)
    file_handler = RotatingFileHandler(
        log_path, maxBytes=MAX_BYTES, backupCount=BACKUP_COUNT, encoding="utf-8"
    )
    file_handler.setFormatter(logging.Formatter(LOG_FORMAT))
    logger.addHandler(file_handler)

    # Stream Handler (Console) - Optional, mainly for dev
    # For TUI client, we might want to suppress console output to avoid breaking UI
    if not capture_stdout:
        stream_handler = logging.StreamHandler(sys.stderr)
        stream_handler.setFormatter(logging.Formatter(LOG_FORMAT))
        logger.addHandler(stream_handler)

    return logger


def get_server_log_path() -> Path:
    """Return absolute path to server log file."""
    return _get_log_dir() / "server.log"


def get_client_logger() -> logging.Logger:
    """Get configured client logger."""
    return setup_logger("opencode.client", "client.log")
