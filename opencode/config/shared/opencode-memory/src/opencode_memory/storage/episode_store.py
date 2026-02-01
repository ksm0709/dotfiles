"""
Episode Store

SQLite 기반 Episode CRUD 저장소

Task 3.1: Episode 데이터 모델 및 스토어
"""

import json
import logging
import sqlite3
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from opencode_memory.models.semantic import (
    Episode,
    EpisodeContext,
)

logger = logging.getLogger(__name__)


# SQL 스키마 정의
EPISODES_SCHEMA = """
CREATE TABLE IF NOT EXISTS episodes (
    id TEXT PRIMARY KEY,
    session_id TEXT NOT NULL,
    goal TEXT NOT NULL,
    status TEXT CHECK(status IN ('active', 'completed', 'failed')) DEFAULT 'active',

    -- 컨텍스트
    context_goal TEXT NOT NULL,
    context_user_request_summary TEXT,
    context_assumptions TEXT,  -- JSON array
    context_agent_thoughts TEXT,  -- JSON array
    context_initial_tool TEXT,
    context_tools_used TEXT,  -- JSON array

    -- 결과
    outcome TEXT,
    learnings TEXT,  -- JSON array

    -- 시간
    start_time DATETIME DEFAULT CURRENT_TIMESTAMP,
    last_updated DATETIME DEFAULT CURRENT_TIMESTAMP,
    end_time DATETIME,

    -- 메타
    tools_used TEXT,  -- JSON array
    record_count INTEGER DEFAULT 0
);

CREATE INDEX IF NOT EXISTS idx_episodes_session ON episodes(session_id);
CREATE INDEX IF NOT EXISTS idx_episodes_status ON episodes(status);
CREATE INDEX IF NOT EXISTS idx_episodes_session_status ON episodes(session_id, status);
"""


class EpisodeStore:
    """Episode SQLite 저장소

    CRUD 메서드:
    - save(episode): 에피소드 저장
    - get(episode_id): 에피소드 조회
    - get_active(session_id): 활성 에피소드 조회
    - update(episode): 에피소드 업데이트
    - list_by_session(session_id, limit): 세션별 에피소드 목록
    """

    def __init__(self, db_path: str):
        """
        Args:
            db_path: SQLite 데이터베이스 파일 경로
        """
        self.db_path = db_path
        self._initialized = False
        self._ensure_directory()

    def _ensure_directory(self):
        """데이터베이스 디렉토리 생성"""
        db_dir = Path(self.db_path).parent
        db_dir.mkdir(parents=True, exist_ok=True)

    def initialize(self):
        """스토어 초기화 - 테이블 생성"""
        if self._initialized:
            return

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.executescript(EPISODES_SCHEMA)
            self._ensure_column(cursor, "episodes", "context_agent_thoughts", "TEXT")
            conn.commit()
            self._initialized = True
            logger.info(f"EpisodeStore initialized at {self.db_path}")
        except Exception as e:
            logger.error(f"Failed to initialize EpisodeStore: {e}")
            raise
        finally:
            conn.close()

    def _get_connection(self) -> sqlite3.Connection:
        """SQLite 연결 획득"""
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        return conn

    def _ensure_column(
        self,
        cursor: sqlite3.Cursor,
        table_name: str,
        column_name: str,
        column_definition: str,
    ) -> None:
        """테이블에 특정 컬럼이 없으면 추가"""
        cursor.execute(f"PRAGMA table_info({table_name})")
        columns = {row[1] for row in cursor.fetchall()}
        if column_name not in columns:
            cursor.execute(
                f"ALTER TABLE {table_name} ADD COLUMN {column_name} {column_definition}"
            )

    def _serialize_episode(self, episode: Episode) -> dict:
        """Episode를 DB 저장용 dict로 변환"""
        return {
            "id": episode.id,
            "session_id": episode.session_id,
            "goal": episode.goal,
            "status": episode.status,
            "context_goal": episode.context.goal,
            "context_user_request_summary": episode.context.user_request_summary,
            "context_assumptions": json.dumps(
                episode.context.assumptions or [], ensure_ascii=False
            ),
            "context_agent_thoughts": json.dumps(
                episode.context.agent_thoughts or [], ensure_ascii=False
            ),
            "context_initial_tool": episode.context.initial_tool,
            "context_tools_used": json.dumps(
                episode.context.tools_used or [], ensure_ascii=False
            ),
            "outcome": episode.outcome,
            "learnings": json.dumps(episode.learnings or [], ensure_ascii=False),
            "start_time": (
                episode.start_time.isoformat() if episode.start_time else None
            ),
            "last_updated": (
                episode.last_updated.isoformat() if episode.last_updated else None
            ),
            "end_time": episode.end_time.isoformat() if episode.end_time else None,
            "tools_used": json.dumps(episode.tools_used or [], ensure_ascii=False),
            "record_count": episode.record_count,
        }

    def _deserialize_episode(self, row: sqlite3.Row) -> Episode:
        """DB Row를 Episode로 변환"""
        data = dict(row)

        # JSON 필드 파싱
        context_assumptions = json.loads(data.get("context_assumptions") or "[]")
        context_agent_thoughts = json.loads(data.get("context_agent_thoughts") or "[]")
        context_tools_used = json.loads(data.get("context_tools_used") or "[]")
        learnings = json.loads(data.get("learnings") or "[]")
        tools_used = json.loads(data.get("tools_used") or "[]")

        # 시간 파싱
        start_time = datetime.now()
        if data.get("start_time"):
            try:
                start_time = datetime.fromisoformat(data["start_time"])
            except (ValueError, TypeError):
                pass

        last_updated = start_time
        if data.get("last_updated"):
            try:
                last_updated = datetime.fromisoformat(data["last_updated"])
            except (ValueError, TypeError):
                pass

        end_time = None
        if data.get("end_time"):
            try:
                end_time = datetime.fromisoformat(data["end_time"])
            except (ValueError, TypeError):
                pass

        # EpisodeContext 생성
        context = EpisodeContext(
            goal=data.get("context_goal", data["goal"]),
            user_request_summary=data.get("context_user_request_summary"),
            assumptions=context_assumptions,
            agent_thoughts=context_agent_thoughts,
            initial_tool=data.get("context_initial_tool"),
            tools_used=context_tools_used,
        )

        return Episode(
            id=data["id"],
            session_id=data["session_id"],
            goal=data["goal"],
            status=data.get("status", "active"),
            context=context,
            outcome=data.get("outcome"),
            learnings=learnings,
            start_time=start_time,
            last_updated=last_updated,
            end_time=end_time,
            tools_used=tools_used,
            record_count=data.get("record_count", 0),
        )

    # ═══════════════════════════════════════════════════════════════
    # CRUD Operations
    # ═══════════════════════════════════════════════════════════════

    def save(self, episode: Episode) -> Dict[str, Any]:
        """에피소드 저장

        Args:
            episode: Episode 인스턴스

        Returns:
            dict: {"status": "success", "episode_id": "..."}
        """
        self.initialize()

        data = self._serialize_episode(episode)

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                """
                INSERT INTO episodes (
                    id, session_id, goal, status,
                    context_goal, context_user_request_summary,
                    context_assumptions, context_agent_thoughts,
                    context_initial_tool, context_tools_used,
                    outcome, learnings,
                    start_time, last_updated, end_time,
                    tools_used, record_count
                ) VALUES (
                    :id, :session_id, :goal, :status,
                    :context_goal, :context_user_request_summary,
                    :context_assumptions, :context_agent_thoughts,
                    :context_initial_tool, :context_tools_used,
                    :outcome, :learnings,
                    :start_time, :last_updated, :end_time,
                    :tools_used, :record_count
                )
                """,
                data,
            )
            conn.commit()
            logger.debug(f"Saved episode: {episode.id}")
            return {"status": "success", "episode_id": episode.id}
        except Exception as e:
            logger.error(f"Failed to save episode: {e}")
            return {"status": "error", "message": str(e)}
        finally:
            conn.close()

    def get(self, episode_id: str) -> Optional[Episode]:
        """에피소드 조회

        Args:
            episode_id: 에피소드 ID

        Returns:
            Episode or None
        """
        self.initialize()

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                "SELECT * FROM episodes WHERE id = ?",
                (episode_id,),
            )
            row = cursor.fetchone()
            if row:
                return self._deserialize_episode(row)
            return None
        finally:
            conn.close()

    def get_active(self, session_id: str) -> Optional[Episode]:
        """활성 에피소드 조회

        Args:
            session_id: 세션 ID

        Returns:
            활성 Episode or None
        """
        self.initialize()

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                """
                SELECT * FROM episodes
                WHERE session_id = ? AND status = 'active'
                ORDER BY start_time DESC
                LIMIT 1
                """,
                (session_id,),
            )
            row = cursor.fetchone()
            if row:
                return self._deserialize_episode(row)
            return None
        finally:
            conn.close()

    def update(self, episode: Episode) -> Dict[str, Any]:
        """에피소드 업데이트

        Args:
            episode: 업데이트할 Episode

        Returns:
            dict: {"status": "success"} or {"status": "error", "message": "..."}
        """
        self.initialize()

        data = self._serialize_episode(episode)

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                """
                UPDATE episodes SET
                    session_id = :session_id,
                    goal = :goal,
                    status = :status,
                    context_goal = :context_goal,
                    context_user_request_summary = :context_user_request_summary,
                    context_assumptions = :context_assumptions,
                    context_agent_thoughts = :context_agent_thoughts,
                    context_initial_tool = :context_initial_tool,
                    context_tools_used = :context_tools_used,
                    outcome = :outcome,
                    learnings = :learnings,
                    start_time = :start_time,
                    last_updated = :last_updated,
                    end_time = :end_time,
                    tools_used = :tools_used,
                    record_count = :record_count
                WHERE id = :id
                """,
                data,
            )
            conn.commit()
            logger.debug(f"Updated episode: {episode.id}")
            return {"status": "success"}
        except Exception as e:
            logger.error(f"Failed to update episode: {e}")
            return {"status": "error", "message": str(e)}
        finally:
            conn.close()

    def list_by_session(
        self,
        session_id: str,
        limit: int = 50,
        offset: int = 0,
    ) -> List[Episode]:
        """세션별 에피소드 목록 조회

        Args:
            session_id: 세션 ID
            limit: 최대 개수
            offset: 시작 위치

        Returns:
            list[Episode]
        """
        self.initialize()

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                """
                SELECT * FROM episodes
                WHERE session_id = ?
                ORDER BY start_time DESC
                LIMIT ? OFFSET ?
                """,
                (session_id, limit, offset),
            )
            rows = cursor.fetchall()
            return [self._deserialize_episode(row) for row in rows]
        finally:
            conn.close()

    def delete(self, episode_id: str) -> Dict[str, Any]:
        """에피소드 삭제

        Args:
            episode_id: 삭제할 에피소드 ID

        Returns:
            dict: {"status": "success"} or {"status": "error", "message": "..."}
        """
        self.initialize()

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                "DELETE FROM episodes WHERE id = ?",
                (episode_id,),
            )
            conn.commit()
            return {"status": "success"}
        except Exception as e:
            logger.error(f"Failed to delete episode: {e}")
            return {"status": "error", "message": str(e)}
        finally:
            conn.close()
