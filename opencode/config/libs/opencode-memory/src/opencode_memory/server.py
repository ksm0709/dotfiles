"""
Context API Server

FastAPI-based server for Context Manager.
Provides HTTP endpoints for plugin communication.
"""

import asyncio
import json
import logging
import os
import signal
import sys
import time
from contextlib import asynccontextmanager
from typing import Optional

import uvicorn
from fastapi import BackgroundTasks, FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from .config import get_project_root
from .server_state import ServerState

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stderr)],
)
logger = logging.getLogger("context-server")

# Constants
IDLE_TIMEOUT = 30 * 60  # 30 minutes
CHECK_INTERVAL = 60  # Check every minute


# Global state helper
def get_state():
    return ServerState.get_instance()


shutdown_event = asyncio.Event()


# Request Models
class InitRequest(BaseModel):
    session_id: str


class StartRequest(BaseModel):
    session_id: str
    task: str


class CheckpointRequest(BaseModel):
    session_id: str
    summary: str = ""


class EndRequest(BaseModel):
    session_id: str
    result: str = ""


class RecordRequest(BaseModel):
    session_id: str
    type: str
    content: str
    importance: str = "low"
    metadata: Optional[dict] = None


class AddRequest(BaseModel):
    content: str
    tags: Optional[list[str]] = None
    metadata: Optional[dict] = None


class QueryRequest(BaseModel):
    query: str
    limit: int = 5
    tags: Optional[list[str]] = None


# Lifecycle Manager
async def idle_checker():
    """Monitor idle time and shutdown if inactive."""
    while not shutdown_event.is_set():
        if get_state().is_idle(IDLE_TIMEOUT):
            logger.info(
                f"Server idle for {IDLE_TIMEOUT}s (Activity Timeout). Shutting down..."
            )
            os.kill(os.getpid(), signal.SIGTERM)
            break
        await asyncio.sleep(CHECK_INTERVAL)


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Starting Context API Server...")

    project_root = get_project_root()
    registry_path = project_root / "data" / "context_server.json"

    # Check for existing server
    if registry_path.exists():
        try:
            with open(registry_path, "r") as f:
                data = json.load(f)
                pid = data.get("pid")
                if pid and pid != os.getpid():
                    try:
                        os.kill(pid, 0)
                        logger.warning(
                            f"Another server is running at PID {pid}. Exiting."
                        )
                        sys.exit(0)
                    except OSError:
                        # Process not running
                        pass
        except Exception:
            # Ignore read errors
            pass

    # Write registry file
    try:
        registry_path.parent.mkdir(parents=True, exist_ok=True)

        registry_data = {
            "port": get_state().port,
            "pid": os.getpid(),
            "api_key": get_state().api_key,
            "started_at": time.time(),
        }

        with open(registry_path, "w") as f:
            json.dump(registry_data, f)

        logger.info(f"Server registry written to {registry_path}")
    except Exception as e:
        logger.error(f"Failed to write server registry: {e}")

    asyncio.create_task(idle_checker())
    yield
    # Shutdown
    logger.info("Shutting down Context API Server...")
    shutdown_event.set()

    # Remove registry file
    if registry_path:
        try:
            if registry_path.exists():
                # Check PID before removing
                should_remove = True
                try:
                    with open(registry_path, "r") as f:
                        data = json.load(f)
                        saved_pid = data.get("pid")
                        # If file belongs to another process, do not remove
                        if saved_pid and saved_pid != os.getpid():
                            logger.warning(
                                f"Registry PID ({saved_pid}) does not match "
                                f"current PID ({os.getpid()}). Skipping removal."
                            )
                            should_remove = False
                except Exception:
                    pass

                if should_remove:
                    registry_path.unlink()
                    logger.info("Server registry removed")
        except Exception as e:
            logger.error(f"Failed to remove server registry: {e}")


app = FastAPI(lifespan=lifespan)


# Middleware for API Key validation
@app.middleware("http")
async def validate_api_key(request: Request, call_next):
    if request.url.path == "/health":
        return await call_next(request)

    expected_key = get_state().api_key
    if not expected_key:
        return await call_next(request)

    client_key = request.headers.get("X-API-Key")
    if client_key != expected_key:
        return JSONResponse(status_code=403, content={"error": "Invalid API Key"})

    get_state().touch()  # Update activity on valid request
    return await call_next(request)


# Endpoints


@app.get("/health")
async def health_check():
    return {
        "status": "ok",
        "pid": os.getpid(),
        "uptime": time.time() - get_state().started_at,
    }


@app.post("/init")
async def init_session(req: InitRequest):
    try:
        wm = get_state().get_working_memory(req.session_id)
        wm.init(req.session_id)  # Always reset on explicit init
        return {
            "status": "success",
            "session_id": req.session_id,
            "timestamp": time.time(),
        }
    except Exception as e:
        logger.error(f"Init failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/start")
async def start_task(req: StartRequest):
    try:
        wm = get_state().get_working_memory(req.session_id)
        wm.start_task(req.task)
        get_state().context_memory.set_current_task(req.task)

        # Retrieve related memories
        relevant = get_state().context_memory.get_for_task(req.task, limit=5)

        # Format result
        result = {
            "status": "success",
            "task": req.task,
            "timestamp": time.time(),
            "relevant_memories": [],
            "context_summary": f"Starting new task: {req.task}",
        }

        if relevant:
            result["relevant_memories"] = [
                {
                    "content": (
                        m["content"][:200] + "..."
                        if len(m["content"]) > 200
                        else m["content"]
                    ),
                    "score": round(m.get("score", 0), 2),
                    "tags": m.get("tags", []),
                }
                for m in relevant
            ]
            # Simple summary generation (can be improved)
            summary_lines = [f"## Task Started: {req.task}\n", "### Related Memories"]
            for i, m in enumerate(relevant, 1):
                summary_lines.append(
                    f"{i}. [{round(m.get('score', 0), 2)}] {m['content'][:150]}..."
                )
                if m.get("tags"):
                    summary_lines.append(f"   - Tags: {', '.join(m['tags'])}")
            result["context_summary"] = "\n".join(summary_lines)

        return result

    except Exception as e:
        logger.error(f"Start failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/checkpoint")
async def checkpoint(req: CheckpointRequest):
    try:
        wm = get_state().get_working_memory(req.session_id)

        # 1. Generate summary from working memory
        summary = req.summary
        items_count = wm.get_items_count()
        if not summary:
            summary = wm.summarize_since_checkpoint()

        if not summary:
            return {"status": "skipped", "reason": "no items to checkpoint"}

        # 2. Save to context memory
        get_state().context_memory.query_and_update(summary)

        # 3. Clear working memory
        wm.clear_since_checkpoint()

        return {
            "status": "success",
            "summary": summary,
            "items_processed": items_count,
            "timestamp": time.time(),
        }
    except Exception as e:
        logger.error(f"Checkpoint failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/auto-checkpoint")
async def auto_checkpoint(req: InitRequest):
    try:
        wm = get_state().get_working_memory(req.session_id)

        # Trigger if items count >= 7 (matching tests)
        if wm.get_items_count() >= 7:
            return await checkpoint(CheckpointRequest(session_id=req.session_id))

        # Trigger if idle count reached (matching tests)
        if wm.increment_idle():
            return await checkpoint(CheckpointRequest(session_id=req.session_id))

        return {"status": "skipped", "reason": "threshold not reached"}
    except Exception as e:
        logger.error(f"Auto-checkpoint failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/end")
async def end_task(req: EndRequest):
    try:
        wm = get_state().get_working_memory(req.session_id)

        # 1. Generate final summary
        summary = req.result
        checkpoint_result = None
        if not summary:
            summary = wm.summarize_since_checkpoint()

        # 2. Save if content exists
        if summary:
            checkpoint_result = get_state().context_memory.query_and_update(summary)

        # 3. Cleanup
        wm.clear()
        get_state().context_memory.set_current_task(None)

        result = {"status": "success", "timestamp": time.time()}
        if checkpoint_result:
            result["checkpoint"] = "performed"

        return result
    except Exception as e:
        logger.error(f"End failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/record")
async def record_item(req: RecordRequest, background_tasks: BackgroundTasks):
    try:
        wm = get_state().get_working_memory(req.session_id)
        wm.add_item(req.type, req.content, req.importance, req.metadata)

        # REQ-AM-01 & REQ-AM-03: Adaptive Storage
        # Also add to context memory for specific important types (persistence)
        # We now include task_goal and task_category in metadata if available
        if (
            req.type in ["error", "fix", "decision", "change"]
            or req.importance == "high"
        ):
            metadata = req.metadata or {}
            if wm.current_task:
                metadata["task_goal"] = wm.current_task
                # Simple category inference or default
                if "task_category" not in metadata:
                    metadata["task_category"] = "General"

            tags = [req.type]
            # Run context memory addition in background to avoid blocking
            background_tasks.add_task(
                get_state().context_memory.add,
                req.content,
                tags=tags,
                metadata=metadata,
            )

        return {"status": "success"}
    except Exception as e:
        logger.error(f"Record failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/add")
async def add_memory(req: AddRequest):
    try:
        result = get_state().context_memory.add(
            req.content, tags=req.tags, metadata=req.metadata
        )
        return result
    except Exception as e:
        logger.error(f"Add failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/query")
async def query_memory(req: QueryRequest):
    try:
        results = get_state().context_memory.query(
            req.query, limit=req.limit, tags=req.tags
        )
        return results
    except Exception as e:
        logger.error(f"Query failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/status")
async def get_status(session_id: Optional[str] = None):
    try:
        status = {
            "server": {
                "pid": os.getpid(),
                "uptime": time.time() - get_state().started_at,
                "sessions": list(get_state().sessions.keys()),
            },
            "context": {
                "initialized": get_state().context_memory._initialized,
                "current_task": get_state().context_memory._current_task,
            },
            # Backward compatibility
            "initialized": get_state().context_memory._initialized,
            "current_task": get_state().context_memory._current_task,
        }

        if session_id and session_id in get_state().sessions:
            wm = get_state().sessions[session_id]
            status["working"] = {
                "items_count": wm.get_items_count(),
                "last_checkpoint": (
                    wm.last_checkpoint.isoformat() if wm.last_checkpoint else None
                ),
            }

        return status
    except Exception as e:
        logger.error(f"Status failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/compaction-context")
async def get_compaction_context(session_id: Optional[str] = None):
    try:
        logger.info(f"Compaction context requested for session: {session_id}")
        summary_parts = ["## Preserved Context from Context Manager"]

        # 1. Current Task
        if get_state().context_memory._current_task:
            logger.info(
                f"Including current task: {get_state().context_memory._current_task}"
            )
            summary_parts.append(
                f"### Current Task\n{get_state().context_memory._current_task}"
            )

        # 2. Working Memory Summary (if session_id provided)
        if session_id and session_id in get_state().sessions:
            wm = get_state().sessions[session_id]
            wm_summary = wm.summarize_since_checkpoint()
            if wm_summary:
                logger.info(
                    f"Including working memory summary ({len(wm_summary)} chars)"
                )
                summary_parts.append(f"### Working Memory Summary\n{wm_summary}")
            else:
                logger.info("Working memory summary is empty")

        # 3. Relevant Memories (based on current task)
        current_task = get_state().context_memory._current_task
        if current_task:
            relevant = get_state().context_memory.get_for_task(current_task, limit=3)
            if relevant:
                logger.info(f"Including {len(relevant)} relevant past memories")
                summary_parts.append("### Relevant Past Memories")
                for i, m in enumerate(relevant, 1):
                    summary_parts.append(f"{i}. {m['content'][:200]}...")
            else:
                logger.info("No relevant past memories found for current task")

        final_context = "\n\n".join(summary_parts)
        logger.info(
            f"Returning compaction context (total length: {len(final_context)})"
        )

        from fastapi.responses import PlainTextResponse

        return PlainTextResponse(final_context)
    except Exception as e:
        logger.error(f"Compaction context failed: {e}", exc_info=True)
        return "## Preserved Context\n(Error retrieving context)"


def run_server(host="127.0.0.1", port=0):
    """Run the server using uvicorn."""
    if port == 0:
        import socket

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(("", 0))
            port = s.getsockname()[1]

    get_state().port = port
    logger.info(f"Server port assigned: {port}")
    logger.info(f"Server API Key: {get_state().api_key}")

    uvicorn.run(app, host=host, port=port)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=0)
    args = parser.parse_args()
    run_server(port=args.port)
