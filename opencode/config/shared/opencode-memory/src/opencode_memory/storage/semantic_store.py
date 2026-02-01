"""
Semantic Record Store

SQLite 기반 SemanticRecord CRUD 저장소

Task 1.3: SemanticRecordStore 구현
"""

import json
import logging
import sqlite3
import uuid
from datetime import datetime
from typing import Any, Dict, List, Optional

from opencode_memory.models.semantic import (
    Decision,
    IntentInfo,
    ProblemResolution,
    SemanticRecord,
)
from opencode_memory.storage.schema import SemanticSchema

logger = logging.getLogger(__name__)


class SemanticRecordStore:
    """Semantic Record SQLite 저장소

    CRUD 메서드:
    - add(record): 레코드 추가
    - get(record_id): 레코드 조회
    - list_by_session(session_id, limit): 세션별 레코드 목록
    - search(query, filters, limit): 레코드 검색
    """

    def __init__(self, db_path: str):
        """
        Args:
            db_path: SQLite 데이터베이스 파일 경로
        """
        self.db_path = db_path
        self.schema = SemanticSchema(db_path)
        self._initialized = False

    def initialize(self):
        """스토어 초기화"""
        if not self._initialized:
            self.schema.initialize()
            self._initialized = True

    def _get_connection(self) -> sqlite3.Connection:
        """SQLite 연결 획득"""
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        return conn

    def _serialize_record(self, record: SemanticRecord) -> dict:
        """SemanticRecord를 DB 저장용 dict로 변환"""
        return {
            "id": record.id,
            "session_id": record.session_id,
            "episode_id": record.episode_id,
            "timestamp": record.timestamp.isoformat() if record.timestamp else None,
            "intent": record.intent,
            "action": record.action,
            "outcome": record.outcome,
            "tool_name": record.tool_name,
            "tool_args": json.dumps(record.tool_args or {}, ensure_ascii=False),
            "success": 1 if record.success else 0,
            "intent_info": (
                json.dumps(record.intent_info.model_dump(), ensure_ascii=False)
                if record.intent_info
                else None
            ),
            "decision": (
                json.dumps(
                    record.decision.model_dump(), ensure_ascii=False, default=str
                )
                if record.decision
                else None
            ),
            "problem_resolution": (
                json.dumps(record.problem_resolution.model_dump(), ensure_ascii=False)
                if record.problem_resolution
                else None
            ),
            "learnings": json.dumps(record.learnings or [], ensure_ascii=False),
            "related_records": json.dumps(
                record.related_records or [], ensure_ascii=False
            ),
            "importance": record.importance,
        }

    def _deserialize_record(self, row: sqlite3.Row) -> SemanticRecord:
        """DB Row를 SemanticRecord로 변환"""
        data = dict(row)

        # JSON 필드 파싱
        tool_args = json.loads(data.get("tool_args") or "{}")
        learnings = json.loads(data.get("learnings") or "[]")
        related_records = json.loads(data.get("related_records") or "[]")

        # 중첩 객체 파싱
        intent_info = None
        if data.get("intent_info"):
            intent_info = IntentInfo(**json.loads(data["intent_info"]))

        decision = None
        if data.get("decision"):
            decision_data = json.loads(data["decision"])
            # timestamp 문자열을 datetime으로 변환
            if "timestamp" in decision_data and isinstance(
                decision_data["timestamp"], str
            ):
                decision_data["timestamp"] = datetime.fromisoformat(
                    decision_data["timestamp"]
                )
            decision = Decision(**decision_data)

        problem_resolution = None
        if data.get("problem_resolution"):
            problem_resolution = ProblemResolution(
                **json.loads(data["problem_resolution"])
            )

        # timestamp 파싱
        timestamp = datetime.now()
        if data.get("timestamp"):
            try:
                timestamp = datetime.fromisoformat(data["timestamp"])
            except (ValueError, TypeError):
                pass

        return SemanticRecord(
            id=data["id"],
            session_id=data["session_id"],
            episode_id=data.get("episode_id"),
            timestamp=timestamp,
            intent=data["intent"],
            action=data["action"],
            outcome=data["outcome"],
            tool_name=data["tool_name"],
            tool_args=tool_args,
            success=bool(data.get("success", 1)),
            intent_info=intent_info,
            decision=decision,
            problem_resolution=problem_resolution,
            learnings=learnings,
            related_records=related_records,
            importance=data.get("importance", "low"),
        )

    # ═══════════════════════════════════════════════════════════════
    # CRUD Operations
    # ═══════════════════════════════════════════════════════════════

    def add(self, record: SemanticRecord) -> Dict:
        """레코드 추가

        Args:
            record: SemanticRecord 인스턴스

        Returns:
            dict: {"status": "success", "record_id": "..."}
        """
        self.initialize()

        data = self._serialize_record(record)

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                """
                INSERT INTO semantic_records (
                    id, session_id, episode_id, timestamp,
                    intent, action, outcome,
                    tool_name, tool_args, success,
                    intent_info, decision, problem_resolution,
                    learnings, related_records, importance
                ) VALUES (
                    :id, :session_id, :episode_id, :timestamp,
                    :intent, :action, :outcome,
                    :tool_name, :tool_args, :success,
                    :intent_info, :decision, :problem_resolution,
                    :learnings, :related_records, :importance
                )
                """,
                data,
            )
            conn.commit()
            logger.debug(f"Added semantic record: {record.id}")
            return {"status": "success", "record_id": record.id}
        except Exception as e:
            logger.error(f"Failed to add record: {e}")
            return {"status": "error", "message": str(e)}
        finally:
            conn.close()

    def get(self, record_id: str) -> Optional[SemanticRecord]:
        """레코드 조회

        Args:
            record_id: 레코드 ID

        Returns:
            SemanticRecord or None
        """
        self.initialize()

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                "SELECT * FROM semantic_records WHERE id = ?",
                (record_id,),
            )
            row = cursor.fetchone()
            if row:
                return self._deserialize_record(row)
            return None
        finally:
            conn.close()

    def list_by_session(
        self,
        session_id: str,
        limit: int = 100,
        offset: int = 0,
    ) -> List[SemanticRecord]:
        """세션별 레코드 목록 조회

        Args:
            session_id: 세션 ID
            limit: 최대 개수
            offset: 시작 위치

        Returns:
            list[SemanticRecord]
        """
        self.initialize()

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                """
                SELECT * FROM semantic_records
                WHERE session_id = ?
                ORDER BY timestamp DESC
                LIMIT ? OFFSET ?
                """,
                (session_id, limit, offset),
            )
            rows = cursor.fetchall()
            return [self._deserialize_record(row) for row in rows]
        finally:
            conn.close()

    def list_by_episode(
        self,
        episode_id: str,
        limit: int = 100,
    ) -> List[SemanticRecord]:
        """에피소드별 레코드 목록 조회

        Args:
            episode_id: 에피소드 ID
            limit: 최대 개수

        Returns:
            list[SemanticRecord]
        """
        self.initialize()

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                """
                SELECT * FROM semantic_records
                WHERE episode_id = ?
                ORDER BY timestamp ASC
                LIMIT ?
                """,
                (episode_id, limit),
            )
            rows = cursor.fetchall()
            return [self._deserialize_record(row) for row in rows]
        finally:
            conn.close()

    def search(
        self,
        query: str,
        session_id: Optional[str] = None,
        tool_name: Optional[str] = None,
        importance: Optional[str] = None,
        limit: int = 10,
    ) -> List[SemanticRecord]:
        """레코드 검색 (키워드 기반)

        Args:
            query: 검색 쿼리
            session_id: 세션 필터
            tool_name: 도구 필터
            importance: 중요도 필터
            limit: 최대 개수

        Returns:
            list[SemanticRecord]
        """
        self.initialize()

        conn = self._get_connection()
        try:
            cursor = conn.cursor()

            # 기본 쿼리
            sql = """
                SELECT * FROM semantic_records
                WHERE (
                    intent LIKE ? OR
                    action LIKE ? OR
                    outcome LIKE ?
                )
            """
            params: List[Any] = [f"%{query}%", f"%{query}%", f"%{query}%"]

            # 필터 추가
            if session_id:
                sql += " AND session_id = ?"
                params.append(session_id)

            if tool_name:
                sql += " AND tool_name = ?"
                params.append(tool_name)

            if importance:
                sql += " AND importance = ?"
                params.append(importance)

            sql += " ORDER BY timestamp DESC LIMIT ?"
            params.append(limit)

            cursor.execute(sql, params)
            rows = cursor.fetchall()
            return [self._deserialize_record(row) for row in rows]
        finally:
            conn.close()

    def update(self, record: SemanticRecord) -> Dict:
        """레코드 업데이트

        Args:
            record: 업데이트할 SemanticRecord

        Returns:
            dict: {"status": "success"} or {"status": "error", "message": "..."}
        """
        self.initialize()

        data = self._serialize_record(record)

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                """
                UPDATE semantic_records SET
                    session_id = :session_id,
                    episode_id = :episode_id,
                    timestamp = :timestamp,
                    intent = :intent,
                    action = :action,
                    outcome = :outcome,
                    tool_name = :tool_name,
                    tool_args = :tool_args,
                    success = :success,
                    intent_info = :intent_info,
                    decision = :decision,
                    problem_resolution = :problem_resolution,
                    learnings = :learnings,
                    related_records = :related_records,
                    importance = :importance
                WHERE id = :id
                """,
                data,
            )
            conn.commit()
            return {"status": "success"}
        except Exception as e:
            logger.error(f"Failed to update record: {e}")
            return {"status": "error", "message": str(e)}
        finally:
            conn.close()

    def delete(self, record_id: str) -> Dict:
        """레코드 삭제

        Args:
            record_id: 삭제할 레코드 ID

        Returns:
            dict: {"status": "success"} or {"status": "error", "message": "..."}
        """
        self.initialize()

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                "DELETE FROM semantic_records WHERE id = ?",
                (record_id,),
            )
            conn.commit()
            return {"status": "success"}
        except Exception as e:
            logger.error(f"Failed to delete record: {e}")
            return {"status": "error", "message": str(e)}
        finally:
            conn.close()

    # ═══════════════════════════════════════════════════════════════
    # Active Intent/Decision/Learning 관리
    # ═══════════════════════════════════════════════════════════════

    def save_active_intent(
        self,
        session_id: str,
        intent_info: IntentInfo,
    ) -> Dict:
        """현재 세션의 활성 Intent 저장

        Args:
            session_id: 세션 ID
            intent_info: IntentInfo 객체

        Returns:
            dict: {"status": "success", "intent_id": "..."}
        """
        self.initialize()

        intent_id = str(uuid.uuid4())

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            # UPSERT (기존 세션의 intent 업데이트 또는 새로 삽입)
            cursor.execute(
                """
                INSERT OR REPLACE INTO active_intents (
                    id, session_id, goal, user_request_summary,
                    context, assumptions, agent_thoughts,
                    source, confidence, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    intent_id,
                    session_id,
                    intent_info.goal,
                    intent_info.user_request_summary,
                    intent_info.context,
                    json.dumps(intent_info.assumptions or [], ensure_ascii=False),
                    json.dumps(intent_info.agent_thoughts or [], ensure_ascii=False),
                    intent_info.source,
                    intent_info.confidence,
                    datetime.now().isoformat(),
                ),
            )
            conn.commit()
            return {"status": "success", "intent_id": intent_id}
        except Exception as e:
            logger.error(f"Failed to save active intent: {e}")
            return {"status": "error", "message": str(e)}
        finally:
            conn.close()

    def get_active_intent(self, session_id: str) -> Optional[IntentInfo]:
        """현재 세션의 활성 Intent 조회"""
        self.initialize()

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                "SELECT * FROM active_intents WHERE session_id = ?",
                (session_id,),
            )
            row = cursor.fetchone()
            if row:
                data = dict(row)
                return IntentInfo(
                    goal=data["goal"],
                    user_request_summary=data.get("user_request_summary"),
                    context=data.get("context"),
                    assumptions=json.loads(data.get("assumptions") or "[]"),
                    agent_thoughts=json.loads(data.get("agent_thoughts") or "[]"),
                    source=data.get("source", "unknown"),
                    confidence=data.get("confidence", 0.5),
                )
            return None
        finally:
            conn.close()

    def save_decision(
        self,
        session_id: str,
        decision: Decision,
    ) -> Dict:
        """의사결정 저장

        Args:
            session_id: 세션 ID
            decision: Decision 객체

        Returns:
            dict: {"status": "success", "decision_id": "..."}
        """
        self.initialize()

        decision_id = str(uuid.uuid4())

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                """
                INSERT INTO active_decisions (
                    id, session_id, decision_type, choice,
                    alternatives, rationale, is_user_preference
                ) VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    decision_id,
                    session_id,
                    decision.decision_type,
                    decision.choice,
                    json.dumps(decision.alternatives or [], ensure_ascii=False),
                    decision.rationale,
                    1 if decision.is_user_preference else 0,
                ),
            )
            conn.commit()
            return {"status": "success", "decision_id": decision_id}
        except Exception as e:
            logger.error(f"Failed to save decision: {e}")
            return {"status": "error", "message": str(e)}
        finally:
            conn.close()

    def save_learning(
        self,
        session_id: str,
        learning: str,
        category: Optional[str] = None,
    ) -> Dict:
        """학습 사항 저장

        Args:
            session_id: 세션 ID
            learning: 학습 내용
            category: 카테고리 (project, pattern, preference, error_solution)

        Returns:
            dict: {"status": "success", "learning_id": "..."}
        """
        self.initialize()

        learning_id = str(uuid.uuid4())

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                """
                INSERT INTO active_learnings (
                    id, session_id, learning, category
                ) VALUES (?, ?, ?, ?)
                """,
                (learning_id, session_id, learning, category),
            )
            conn.commit()
            return {"status": "success", "learning_id": learning_id}
        except Exception as e:
            logger.error(f"Failed to save learning: {e}")
            return {"status": "error", "message": str(e)}
        finally:
            conn.close()

    def get_session_learnings(self, session_id: str) -> List[str]:
        """세션의 학습 사항 목록 조회"""
        self.initialize()

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                "SELECT learning FROM active_learnings WHERE session_id = ?",
                (session_id,),
            )
            rows = cursor.fetchall()
            return [row["learning"] for row in rows]
        finally:
            conn.close()

    def get_recent_decisions(
        self,
        session_id: str,
        limit: int = 5,
    ) -> List[Decision]:
        """세션의 최근 의사결정 목록 조회"""
        self.initialize()

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                """
                SELECT * FROM active_decisions
                WHERE session_id = ?
                ORDER BY created_at DESC
                LIMIT ?
                """,
                (session_id, limit),
            )
            rows = cursor.fetchall()
            decisions = []
            for row in rows:
                data = dict(row)
                decisions.append(
                    Decision(
                        decision_type=data["decision_type"],
                        choice=data["choice"],
                        alternatives=json.loads(data.get("alternatives") or "[]"),
                        rationale=data["rationale"],
                        is_user_preference=bool(data.get("is_user_preference", 0)),
                    )
                )
            return decisions
        finally:
            conn.close()
