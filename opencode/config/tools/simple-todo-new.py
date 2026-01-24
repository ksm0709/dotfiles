#!/usr/bin/env python3
"""
Session-aware Simple Todo Manager
OpenCode Context API와 통합된 세션 관리

OpenSpec 시나리오 기반 TDD 구현:
- Session ID Persistence
- Project-Relative Path Resolution  
- Hybrid Architecture Integration
- Backward Compatibility
"""

import argparse
import json
import os
import uuid
import re
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Any, Optional


class ProjectPathResolver:
    """프로젝트 상대 경로 해결기"""
    
    @staticmethod
    def get_project_root() -> Path:
        """
        .opencode 디렉토리를 찾아 프로젝트 루트 감지
        
        Returns:
            Path: 프로젝트 루트 경로
        """
        current = Path.cwd()
        
        # 상위 디렉토리로 거슬러 올라가며 .opencode 디렉토리 탐지
        while current.parent != current:
            if (current / ".opencode").exists():
                return current
            current = current.parent
        
        # fallback: 현재 디렉토리 반환
        return Path.cwd()
    
    @staticmethod
    def get_sessions_base_path() -> Path:
        """
        세션 기본 경로 가져오기
        
        Returns:
            Path: 세션 저장 기본 경로
        """
        project_root = ProjectPathResolver.get_project_root()
        return project_root / ".opencode" / "sessions"
    
    @staticmethod
    def sanitize_session_id(session_id: str) -> str:
        """
        세션 ID 보안 정제
        
        Args:
            session_id: 원본 세션 ID
            
        Returns:
            str: 정제된 세션 ID
        """
        # 경로 탈출 문자 제거
        sanitized = re.sub(r'[./\\]', '_', session_id)
        
        # 빈 문자열 방지
        if not sanitized.strip():
            sanitized = "default"
        
        return sanitized


class SimpleTodoManager:
    """
    OpenSpec 시나리오를 만족하는 세션 인지 Todo Manager
    
    Features:
    - Session ID persistence across multiple calls
    - Project-relative path resolution
    - Path security validation
    - CLI interface support
    - Backward compatibility
    """
    
    def __init__(self, agent_name: str, session_id: str):
        self.agent_name = agent_name
        self.session_id = ProjectPathResolver.sanitize_session_id(session_id)
        
        # 프로젝트 상대 경로 기반 디렉토리 생성
        self.sessions_base_path = ProjectPathResolver.get_sessions_base_path()
        self.todo_dir = self.sessions_base_path / f"{self.agent_name}_{self.session_id}"
        self.todo_file = self.todo_dir / "todos.json"
        
        # 디렉토리 생성
        self.todo_dir.mkdir(parents=True, exist_ok=True)
        
        # 기존 파일이 없으면 초기화
        if not self.todo_file.exists():
            self.save_todos([])
    
    def load_todos(self) -> List[Dict[str, Any]]:
        """Todo 목록 로드"""
        try:
            with open(self.todo_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return []
    
    def save_todos(self, todos: List[Dict[str, Any]]) -> bool:
        """Todo 목록 저장"""
        try:
            with open(self.todo_file, 'w', encoding='utf-8') as f:
                json.dump(todos, f, ensure_ascii=False, indent=2)
            return True
        except Exception:
            return False
    
    def add_todo(self, content: str, priority: str = "medium") -> str:
        """
        새로운 Todo 추가
        
        Args:
            content: Todo 내용
            priority: 우선순위 (high, medium, low)
            
        Returns:
            str: 생성된 Todo ID
        """
        todos = self.load_todos()
        
        new_todo = {
            "id": f"{self.agent_name[:3]}-{len(todos)+1:03d}",
            "content": content,
            "priority": priority,
            "status": "pending",
            "created_at": datetime.now().isoformat(),
            "updated_at": datetime.now().isoformat(),
            "session_id": self.session_id
        }
        
        todos.append(new_todo)
        self.save_todos(todos)
        
        return new_todo['id']
    
    def update_status(self, todo_id: str, status: str) -> bool:
        """
        Todo 상태 업데이트
        
        Args:
            todo_id: Todo ID
            status: 새로운 상태
            
        Returns:
            bool: 업데이트 성공 여부
        """
        todos = self.load_todos()
        
        for todo in todos:
            if todo['id'] == todo_id:
                todo['status'] = status
                todo['updated_at'] = datetime.now().isoformat()
                self.save_todos(todos)
                return True
        
        return False
    
    def list_todos(self, status: str = None) -> List[Dict[str, Any]]:
        """
        Todo 목록 조회
        
        Args:
            status: 필터링할 상태 (可选)
            
        Returns:
            List[Dict]: Todo 목록
        """
        todos = self.load_todos()
        
        if status:
            todos = [todo for todo in todos if todo['status'] == status]
        
        return todos
    
    def get_session_info(self) -> Dict[str, Any]:
        """
        세션 정보 조회
        
        Returns:
            Dict: 세션 정보
        """
        todos = self.load_todos()
        
        return {
            "agent_name": self.agent_name,
            "session_id": self.session_id,
            "opencode_session_id": self.session_id,
            "todo_dir": str(self.todo_dir),
            "project_root": str(ProjectPathResolver.get_project_root()),
            "total_todos": len(todos),
            "pending": len([t for t in todos if t['status'] == 'pending']),
            "in_progress": len([t for t in todos if t['status'] == 'in_progress']),
            "completed": len([t for t in todos if t['status'] == 'completed']),
            "created_at": datetime.fromtimestamp(self.todo_dir.stat().st_ctime).isoformat() if self.todo_dir.exists() else None
        }


def main():
    """CLI 인터페이스 메인 함수"""
    parser = argparse.ArgumentParser(description="Session-aware Simple Todo Manager")
    parser.add_argument("--agent", required=True, help="Agent name")
    parser.add_argument("--session", required=True, help="OpenCode Session ID")
    parser.add_argument("--action", required=True, choices=["add", "update", "list", "info"], help="Action")
    parser.add_argument("--content", help="Todo content")
    parser.add_argument("--priority", default="medium", choices=["high", "medium", "low"], help="Priority")
    parser.add_argument("--task-id", help="Task ID for update")
    parser.add_argument("--status", choices=["pending", "in_progress", "completed"], help="Status for update")
    
    args = parser.parse_args()
    
    try:
        # Session ID를 사용하여 Todo Manager 생성
        todos = SimpleTodoManager(args.agent, args.session)
        
        if args.action == "add":
            if not args.content:
                print("ERROR: Content is required for add action")
                return 1
            
            task_id = todos.add_todo(args.content, args.priority)
            print(f"SUCCESS:{task_id}")
            
        elif args.action == "update":
            if not args.task_id or not args.status:
                print("ERROR: Task ID and status are required for update action")
                return 1
            
            success = todos.update_status(args.task_id, args.status)
            print(f"SUCCESS:{success}")
            
        elif args.action == "list":
            todo_list = todos.list_todos()
            print(f"SUCCESS:{json.dumps(todo_list, ensure_ascii=False)}")
            
        elif args.action == "info":
            info = todos.get_session_info()
            print(f"SUCCESS:{json.dumps(info, ensure_ascii=False)}")
        
        return 0
        
    except Exception as e:
        print(f"ERROR:{str(e)}")
        return 1


if __name__ == "__main__":
    exit(main())