"""
Semantic Memory Database Schema

SQLite 스키마 정의 및 마이그레이션

Task 1.2: 데이터베이스 스키마 추가
"""

import logging
import sqlite3
from pathlib import Path

logger = logging.getLogger(__name__)


# SQL 스키마 정의
SEMANTIC_RECORDS_SCHEMA = """
CREATE TABLE IF NOT EXISTS semantic_records (
    id TEXT PRIMARY KEY,
    session_id TEXT NOT NULL,
    episode_id TEXT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,

    -- 핵심 삼중체
    intent TEXT NOT NULL,
    action TEXT NOT NULL,
    outcome TEXT NOT NULL,

    -- 도구 정보
    tool_name TEXT NOT NULL,
    tool_args TEXT,  -- JSON
    success BOOLEAN DEFAULT 1,

    -- 추가 정보
    intent_info TEXT,  -- JSON (IntentInfo 객체)
    decision TEXT,  -- JSON (Decision 객체)
    problem_resolution TEXT,  -- JSON (ProblemResolution 객체)
    learnings TEXT,  -- JSON array
    related_records TEXT,  -- JSON array of record IDs

    -- 중요도
    importance TEXT CHECK(importance IN ('low', 'medium', 'high', 'critical')) DEFAULT 'low'
);
"""

# 인덱스 정의
SEMANTIC_RECORDS_INDEXES = """
CREATE INDEX IF NOT EXISTS idx_records_session ON semantic_records(session_id);
CREATE INDEX IF NOT EXISTS idx_records_episode ON semantic_records(episode_id);
CREATE INDEX IF NOT EXISTS idx_records_tool ON semantic_records(tool_name);
CREATE INDEX IF NOT EXISTS idx_records_importance ON semantic_records(importance);
CREATE INDEX IF NOT EXISTS idx_records_timestamp ON semantic_records(timestamp);
"""

# Active Intents 테이블 (세션별 현재 의도 추적)
ACTIVE_INTENTS_SCHEMA = """
CREATE TABLE IF NOT EXISTS active_intents (
    id TEXT PRIMARY KEY,
    session_id TEXT NOT NULL UNIQUE,
    goal TEXT NOT NULL,
    user_request_summary TEXT,
    context TEXT,
    assumptions TEXT,  -- JSON array
    source TEXT DEFAULT 'unknown',
    confidence REAL DEFAULT 0.5,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
"""

# Active Decisions 테이블
ACTIVE_DECISIONS_SCHEMA = """
CREATE TABLE IF NOT EXISTS active_decisions (
    id TEXT PRIMARY KEY,
    session_id TEXT NOT NULL,
    decision_type TEXT NOT NULL,
    choice TEXT NOT NULL,
    alternatives TEXT,  -- JSON array
    rationale TEXT NOT NULL,
    is_user_preference BOOLEAN DEFAULT 0,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
"""

# Active Learnings 테이블
ACTIVE_LEARNINGS_SCHEMA = """
CREATE TABLE IF NOT EXISTS active_learnings (
    id TEXT PRIMARY KEY,
    session_id TEXT NOT NULL,
    learning TEXT NOT NULL,
    category TEXT,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
"""


class SemanticSchema:
    """Semantic Memory 데이터베이스 스키마 관리"""

    def __init__(self, db_path: str):
        """
        Args:
            db_path: SQLite 데이터베이스 파일 경로
        """
        self.db_path = db_path
        self._ensure_directory()

    def _ensure_directory(self):
        """데이터베이스 디렉토리 생성"""
        db_dir = Path(self.db_path).parent
        db_dir.mkdir(parents=True, exist_ok=True)

    def _get_connection(self) -> sqlite3.Connection:
        """SQLite 연결 획득"""
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        return conn

    def initialize(self):
        """스키마 초기화 - 테이블 및 인덱스 생성"""
        logger.info(f"Initializing semantic schema at {self.db_path}")

        conn = self._get_connection()
        try:
            cursor = conn.cursor()

            # 테이블 생성
            cursor.executescript(SEMANTIC_RECORDS_SCHEMA)
            cursor.executescript(ACTIVE_INTENTS_SCHEMA)
            cursor.executescript(ACTIVE_DECISIONS_SCHEMA)
            cursor.executescript(ACTIVE_LEARNINGS_SCHEMA)

            # 인덱스 생성
            cursor.executescript(SEMANTIC_RECORDS_INDEXES)

            conn.commit()
            logger.info("Semantic schema initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize schema: {e}")
            raise
        finally:
            conn.close()

    def is_initialized(self) -> bool:
        """스키마가 초기화되었는지 확인"""
        if not Path(self.db_path).exists():
            return False

        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(
                "SELECT name FROM sqlite_master WHERE type='table' AND name='semantic_records'"
            )
            return cursor.fetchone() is not None
        finally:
            conn.close()

    def get_table_info(self, table_name: str) -> list:
        """테이블 정보 조회"""
        conn = self._get_connection()
        try:
            cursor = conn.cursor()
            cursor.execute(f"PRAGMA table_info({table_name})")
            return cursor.fetchall()
        finally:
            conn.close()

    def migrate(self):
        """스키마 마이그레이션 (향후 확장용)"""
        if not self.is_initialized():
            self.initialize()
            return

        # 향후 마이그레이션 로직 추가
        logger.info("Schema migration completed (no changes needed)")
