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
from .models.semantic import (
    CompleteEpisodeRequest,
    Decision,
    EpisodeContext,
    EpisodeResponse,
    IntentInfo,
    RecordDecisionRequest,
    RecordIntentRequest,
    RecordLearningRequest,
    RecordSemanticRequest,
    SemanticRecord,
    SemanticRecordResponse,
    StartEpisodeRequest,
)
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


# ═══════════════════════════════════════════════════════════════
# Problem Tracking Request Models (Phase 2)
# ═══════════════════════════════════════════════════════════════


class StartProblemRequest(BaseModel):
    session_id: str
    error_message: str


class ProblemAttemptRequest(BaseModel):
    problem_id: str
    solution: str


class ResolveProblemRequest(BaseModel):
    problem_id: str
    solution: str


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

        # Task 3.6: context_start 연동 - 에피소드 자동 시작
        # 기존 활성 에피소드가 있으면 중단하고 새 에피소드 시작 (BoundaryDetector 규칙)
        episode_manager = get_state().episode_manager
        
        # EpisodeContext 생성
        context = EpisodeContext(
            goal=req.task,
            initial_tool="context_start"
        )
        
        # 새 에피소드 시작 (기존 활성 에피소드 자동 중단)
        episode = episode_manager.start_episode(req.session_id, req.task, context)
        logger.info(f"Auto-started episode {episode.id} from context_start")

        # Retrieve related memories
        relevant = await get_state().context_memory.get_for_task(req.task, limit=5)

        # Format result
        result = {
            "status": "success",
            "task": req.task,
            "episode_id": episode.id,
            "timestamp": time.time(),
            "relevant_memories": [],
            "context_summary": f"Starting new task: {req.task}",
        }

        if relevant:
            # Format memories for response
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

            # Try to generate LLM summary
            llm_summary = None
            try:
                llm_summary = await get_state().llm_client.summarize_memories(
                    req.task, relevant
                )
            except Exception as e:
                logger.warning(f"LLM summarization failed: {e}")

            if llm_summary:
                result["context_summary"] = llm_summary
            else:
                # Fallback summary with explicit instruction
                summary_lines = [
                    f"## Task Started: {req.task}",
                    "### Related Memories (Raw List)",
                    "⚠️ **System Warning**: LLM summarization failed.",
                    "**ACTION REQUIRED**: Read the raw memories below and manually summarize key insights into your `context_intent`.",
                    "",
                ]
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
        await get_state().context_memory.query_and_update(summary)

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
            checkpoint_result = await get_state().context_memory.query_and_update(summary)

        # 3. Cleanup
        wm.clear()
        get_state().context_memory.set_current_task(None)
        
        # Task 3.6: context_end 연동 - 에피소드 완료 처리
        episode_manager = get_state().episode_manager
        active_episode = episode_manager.get_active_episode(req.session_id)
        if active_episode:
            episode_manager.complete_episode(
                req.session_id, 
                outcome="completed_by_user_end_command"
            )
            logger.info(f"Auto-completed episode {active_episode.id} from context_end")

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
        results = await get_state().context_memory.query(
            req.query, limit=req.limit, tags=req.tags
        )
        return results
    except Exception as e:
        logger.error(f"Query failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


# ═══════════════════════════════════════════════════════════════
# Semantic Memory API Endpoints (Task 1.8)
# ═══════════════════════════════════════════════════════════════


@app.post("/record-intent")
async def record_intent(req: RecordIntentRequest):
    """POST /record-intent - 작업 의도 기록

    에이전트가 context_intent 도구를 호출할 때 사용합니다.
    세션의 현재 작업 의도를 저장합니다.
    """
    try:
        intent_info = IntentInfo(
            goal=req.goal,
            user_request_summary=req.user_request_summary,
            context=req.context,
            assumptions=req.assumptions or [],
            agent_thoughts=req.agent_thoughts or [],
            source="agent",
            confidence=0.9,
        )

        result = get_state().semantic_store.save_active_intent(
            session_id=req.session_id,
            intent_info=intent_info,
        )

        if result["status"] == "success":
            return SemanticRecordResponse(
                status="success",
                intent_id=result["intent_id"],
                agent_thoughts=intent_info.agent_thoughts,
                message=f"Intent recorded: {req.goal}",
            )
        else:
            raise HTTPException(status_code=500, detail=result.get("message"))
    except Exception as e:
        logger.error(f"Record intent failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/record-decision")
async def record_decision(req: RecordDecisionRequest):
    """POST /record-decision - 의사결정 기록

    에이전트가 context_decision 도구를 호출할 때 사용합니다.
    라이브러리 선택, 아키텍처 패턴 결정 등을 기록합니다.
    """
    try:
        decision = Decision(
            decision_type=req.decision_type,
            choice=req.choice,
            alternatives=req.alternatives or [],
            rationale=req.rationale,
            is_user_preference=req.is_user_preference,
        )

        result = get_state().semantic_store.save_decision(
            session_id=req.session_id,
            decision=decision,
        )

        if result["status"] == "success":
            return SemanticRecordResponse(
                status="success",
                decision_id=result["decision_id"],
                message=f"Decision recorded: {req.choice} ({req.decision_type})",
            )
        else:
            raise HTTPException(status_code=500, detail=result.get("message"))
    except Exception as e:
        logger.error(f"Record decision failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/record-learning")
async def record_learning(req: RecordLearningRequest):
    """POST /record-learning - 학습 사항 기록

    에이전트가 context_learning 도구를 호출할 때 사용합니다.
    오류 해결 방법, 프로젝트 특성 등을 기록합니다.
    """
    try:
        result = get_state().semantic_store.save_learning(
            session_id=req.session_id,
            learning=req.learning,
            category=req.category,
        )
        
        # Task 3.5: 학습 사항을 활성 에피소드에도 추가
        episode_manager = get_state().episode_manager
        episode_manager.add_learning(req.session_id, req.learning)

        if result["status"] == "success":
            return SemanticRecordResponse(
                status="success",
                learning_id=result["learning_id"],
                message=f"Learning recorded: {req.learning[:50]}...",
            )
        else:
            raise HTTPException(status_code=500, detail=result.get("message"))
    except Exception as e:
        logger.error(f"Record learning failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/record-semantic")
async def record_semantic(req: RecordSemanticRequest):
    """POST /record-semantic - 의미 기반 도구 실행 기록

    도구 실행 후 Intent-Action-Outcome 삼중체를 저장합니다.
    선택적으로 의사결정 정보를 포함할 수 있습니다.
    """
    try:
        # Decision 객체 생성 (있는 경우)
        decision = None
        if req.decision:
            decision = Decision(
                decision_type=req.decision.get("decision_type", "unknown"),
                choice=req.decision.get("choice", ""),
                alternatives=req.decision.get("alternatives", []),
                rationale=req.decision.get("rationale", ""),
                is_user_preference=req.decision.get("is_user_preference", False),
            )

        # 활성 Intent 조회 (있으면 IntentInfo로 포함)
        active_intent = get_state().semantic_store.get_active_intent(req.session_id)

        # SemanticRecord 생성
        record = SemanticRecord(
            session_id=req.session_id,
            intent=req.intent,
            intent_info=active_intent,
            action=req.action,
            outcome=req.outcome,
            tool_name=req.tool_name,
            tool_args=req.tool_args or {},
            success=req.success,
            decision=decision,
            importance=req.importance,
            episode_id=None,
            problem_resolution=None,
        )
        
        # Task 3.5: 활성 에피소드 연동
        # EpisodeManager를 통해 에피소드 ID 할당 및 tools_used 업데이트
        episode_manager = get_state().episode_manager
        
        # Boundary Detection (Task 3.3)
        # 경계 감지 수행
        active_episode = episode_manager.get_active_episode(req.session_id)
        if get_state().boundary_detector.should_start_new_episode(
            current_episode=active_episode,
            new_input=req.intent, # Use intent as new input for now
            tool_name=req.tool_name
        ):
            # 새 에피소드가 필요하면?
            # 현재는 context_start 외에는 자동 시작하지 않고, 
            # 기존 에피소드에 계속 기록하거나 에피소드 없이 기록 (None)
            # 향후 LLM 기반 감지 시 여기서 자동 분기 가능
            pass
            
        episode_manager.add_record(req.session_id, record)

        # 저장
        result = get_state().semantic_store.add(record)

        if result["status"] == "success":
            return SemanticRecordResponse(
                status="success",
                record_id=result["record_id"],
                episode_id=record.episode_id,
                importance=record.importance,
                message=f"Semantic record created: {record.action}",
            )
        else:
            raise HTTPException(status_code=500, detail=result.get("message"))
    except Exception as e:
        logger.error(f"Record semantic failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


# ═══════════════════════════════════════════════════════════════
# Problem Tracking API Endpoints (Phase 2: Task 2.4, 2.5)
# ═══════════════════════════════════════════════════════════════


@app.post("/problem/start")
async def start_problem(req: StartProblemRequest):
    """POST /problem/start - 문제 추적 시작

    오류 발생 시 호출하여 문제 추적을 시작합니다.
    """
    try:
        tracker = get_state().problem_tracker

        problem_id = tracker.start_tracking(
            session_id=req.session_id,
            error=req.error_message,
        )

        problem = tracker.get_active_problem(req.session_id)

        return {
            "status": "success",
            "problem_id": problem_id,
            "error_type": problem.error_type if problem else "Unknown",
            "message": f"Problem tracking started for: {req.error_message[:50]}...",
        }
    except Exception as e:
        logger.error(f"Start problem failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/problem/attempt")
async def add_problem_attempt(req: ProblemAttemptRequest):
    """POST /problem/attempt - 해결 시도 기록

    문제 해결을 시도한 작업을 기록합니다.
    """
    try:
        tracker = get_state().problem_tracker

        success = tracker.add_attempt(
            problem_id=req.problem_id,
            solution=req.solution,
        )

        if success:
            return {
                "status": "success",
                "message": f"Attempt recorded: {req.solution[:50]}...",
            }
        else:
            raise HTTPException(
                status_code=404,
                detail=f"Problem not found or already resolved: {req.problem_id}",
            )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Add attempt failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/problem/resolve")
async def resolve_problem(req: ResolveProblemRequest):
    """POST /problem/resolve - 문제 해결 완료

    문제가 해결되었음을 기록합니다.
    """
    try:
        tracker = get_state().problem_tracker

        resolution = tracker.resolve(
            problem_id=req.problem_id,
            solution=req.solution,
        )

        if resolution:
            # Task 3.5: 해결된 문제를 에피소드에 연결
            session_id = tracker.get_session_for_problem(req.problem_id)
            if session_id: # Note: resolve removes from problem_sessions, so we might miss it
                # Logic improvement: tracker.resolve removes from active list but we need session_id
                # This should be handled inside tracker or we pass session_id
                pass
            
            # Since resolve deletes from active list, we can't easily get session_id unless we change tracker
            # For now, we'll skip explicit linking here, relying on run-time linking if needed
            
            return {
                "status": "resolved",
                "problem_id": resolution.id,
                "error_type": resolution.error_type,
                "error_message": resolution.error_message,
                "attempted_solutions": resolution.attempted_solutions,
                "successful_solution": resolution.successful_solution,
                "message": "Problem resolved successfully",
            }
        else:
            raise HTTPException(
                status_code=404,
                detail=f"Problem not found: {req.problem_id}",
            )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Resolve problem failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/problem/active")
async def get_active_problem(session_id: str):
    """GET /problem/active - 활성 문제 조회

    세션의 현재 활성 문제를 조회합니다.
    """
    try:
        tracker = get_state().problem_tracker

        problem = tracker.get_active_problem(session_id)

        if problem:
            return {
                "has_active_problem": True,
                "problem": {
                    "id": problem.id,
                    "error_type": problem.error_type,
                    "error_message": problem.error_message,
                    "attempted_solutions": problem.attempted_solutions,
                    "status": problem.status,
                },
            }
        else:
            return {
                "has_active_problem": False,
                "problem": None,
            }
    except Exception as e:
        logger.error(f"Get active problem failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


# ═══════════════════════════════════════════════════════════════
# Episode Management API Endpoints (Phase 3: Task 3.4)
# ═══════════════════════════════════════════════════════════════


@app.post("/episode/start")
async def start_episode(req: StartEpisodeRequest):
    """POST /episode/start - 새 에피소드 시작"""
    try:
        episode_manager = get_state().episode_manager
        
        # Context dict -> EpisodeContext object
        context_data = req.context or {}
        goal = context_data.get("goal") or req.goal

        context = EpisodeContext(
            goal=goal,
            user_request_summary=context_data.get("user_request_summary"),
            assumptions=context_data.get("assumptions", []),
            agent_thoughts=context_data.get("agent_thoughts", []),
            initial_tool=context_data.get("initial_tool"),
            tools_used=context_data.get("tools_used", [])
        )
        
        episode = episode_manager.start_episode(
            session_id=req.session_id,
            goal=req.goal,
            context=context
        )
        
        return EpisodeResponse(
            status="success",
            episode_id=episode.id,
            goal=episode.goal,
            message=f"Episode started: {episode.goal}"
        )
    except Exception as e:
        logger.error(f"Start episode failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/episode/complete")
async def complete_episode(req: CompleteEpisodeRequest):
    """POST /episode/complete - 에피소드 완료"""
    try:
        episode_manager = get_state().episode_manager
        
        episode = episode_manager.complete_episode(
            session_id=req.session_id,
            outcome=req.outcome
        )
        
        if episode:
            return EpisodeResponse(
                status="completed",
                episode_id=episode.id,
                goal=episode.goal,
                message=f"Episode completed: {episode.goal}"
            )
        else:
            return EpisodeResponse(
                status="error",
                message="No active episode found"
            )
    except Exception as e:
        logger.error(f"Complete episode failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/episode/active")
async def get_active_episode(session_id: str):
    """GET /episode/active - 활성 에피소드 조회"""
    try:
        episode_manager = get_state().episode_manager
        
        episode = episode_manager.get_active_episode(session_id)
        
        if episode:
            return {
                "has_active_episode": True,
                "episode": {
                    "id": episode.id,
                    "goal": episode.goal,
                    "status": episode.status,
                    "start_time": episode.start_time.isoformat() if episode.start_time else None,
                    "tools_used": episode.tools_used
                }
            }
        else:
            return {
                "has_active_episode": False,
                "episode": None
            }
    except Exception as e:
        logger.error(f"Get active episode failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/episode/{episode_id}")
async def get_episode(episode_id: str):
    """GET /episode/{episode_id} - 에피소드 조회"""
    try:
        episode_manager = get_state().episode_manager
        
        episode = episode_manager.get_episode(episode_id)
        
        if episode:
            # Serialize
            return {
                "id": episode.id,
                "session_id": episode.session_id,
                "goal": episode.goal,
                "status": episode.status,
                "outcome": episode.outcome,
                "learnings": episode.learnings,
                "tools_used": episode.tools_used,
                "start_time": episode.start_time.isoformat() if episode.start_time else None,
                "end_time": episode.end_time.isoformat() if episode.end_time else None,
                "context": {
                    "goal": episode.context.goal,
                    "user_request_summary": episode.context.user_request_summary,
                    "assumptions": episode.context.assumptions,
                    "agent_thoughts": episode.context.agent_thoughts,
                }
            }
        else:
            raise HTTPException(status_code=404, detail="Episode not found")
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get episode failed: {e}", exc_info=True)
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
            relevant = await get_state().context_memory.get_for_task(current_task, limit=3)
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
