"""
Server Lifecycle Manager

Handles server startup, port selection, and discovery file management.
Executed by the plugin to spawn the server process.
"""

import fcntl
import json
import os
import socket
import subprocess
import sys
import time
import uuid
from pathlib import Path

from .utils.logging import get_server_log_path


def get_free_port():
    """Find a free port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("", 0))
        return s.getsockname()[1]


def get_server_info_path(project_root: Path) -> Path:
    """Get path to context_server.json."""
    data_dir = project_root / "data"
    data_dir.mkdir(parents=True, exist_ok=True)
    return data_dir / "context_server.json"


def acquire_lock(lock_path: Path):
    """Acquire file lock to prevent race conditions."""
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock_file = open(lock_path, "w")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        return lock_file
    except IOError:
        return None


def start_server(project_root: str):
    """Start the API server."""
    root_path = Path(project_root).resolve()
    server_info_path = get_server_info_path(root_path)
    lock_path = server_info_path.with_suffix(".lock")

    # 1. Acquire lock
    lock_file = acquire_lock(lock_path)
    if not lock_file:
        print("Server is starting by another process...")
        return

    try:
        # 2. Check if already running
        if server_info_path.exists():
            try:
                with open(server_info_path) as f:
                    info = json.load(f)
                # Check if process exists
                os.kill(info["pid"], 0)
                print(f"Server already running at port {info['port']}")
                return
            except (OSError, json.JSONDecodeError, KeyError):
                # Process dead or file corrupted
                pass

        # 3. Prepare configuration
        port = get_free_port()
        api_key = str(uuid.uuid4())

        # 4. Spawn server process
        env = os.environ.copy()
        env["CONTEXT_SERVER_KEY"] = api_key
        env["PYTHONPATH"] = str(root_path)

        # Use the same python interpreter
        python_exe = sys.executable

        cmd = [
            python_exe,
            "-m",
            "uvicorn",
            "opencode_memory.server:app",
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
            "--log-level",
            "info",
        ]

        # Start as detached process
        log_path = get_server_log_path()
        log_path.parent.mkdir(parents=True, exist_ok=True)

        process = subprocess.Popen(
            cmd,
            cwd=str(root_path),
            env=env,
            stdout=open(log_path, "a"),
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        # 5. Write server info
        info = {
            "pid": process.pid,
            "port": port,
            "key": api_key,
            "started_at": time.time(),
        }

        with open(server_info_path, "w") as f:
            json.dump(info, f)

        print(f"Server started at port {port} (PID: {process.pid})")

        # Wait a bit to ensure startup
        time.sleep(1)

    finally:
        # Release lock
        if lock_file:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
            lock_file.close()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python -m opencode_memory.server_lifecycle <project_root>")
        sys.exit(1)

    start_server(sys.argv[1])
