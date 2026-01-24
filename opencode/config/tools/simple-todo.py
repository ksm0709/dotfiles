#!/usr/bin/env python3
"""
Simple Todo Manager for Subagents
Subagent들이 직접 사용할 수 있는 간단한 Todo 관리 도구

세션 ID 유지 메커니즘을 통해 서브에이전트가 여러 번 호출되어도
동일한 세션을 유지하고 상태를 공유합니다.
"""

import argparse
import json
import os
import sys
import uuid
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Any, Optional

# SessionRegistry import
try:
    # Try to import from dotfiles shared location
    sys.path.insert(0, str(Path.home() / "dotfiles" / "opencode" / "config" / "shared" / "context"))
    from session_registry import SessionRegistry, get_session_id, create_session
    SESSION_REGISTRY_AVAILABLE = True
except ImportError:
    # Fallback to basic session management
    SESSION_REGISTRY_AVAILABLE = False
    print("Warning: SessionRegistry not available, using basic session management")

def detect_project_root() -> Path:
    """프로젝트 루트 디렉토리 감지 (Git 루트 또는 현재 디렉토리)"""
    current_dir = Path.cwd()
    
    # Git 루트 디렉토리 찾기
    git_dir = current_dir
    while git_dir != git_dir.parent:
        if (git_dir / ".git").exists():
            return git_dir
        git_dir = git_dir.parent
    
    # Git 루트를 찾지 못하면 현재 디렉토리 반환
    return current_dir

def validate_path_security(path: str, base_path: Path = None) -> bool:
    """경로 보안 검증 (디렉토리 트래버설 공격 방지)"""
    if base_path is None:
        base_path = detect_project_root()
    
    try:
        resolved_path = Path(path).resolve()
        return resolved_path.is_relative_to(base_path)
    except (ValueError, OSError):
        return False

def get_project_relative_path(absolute_path: Path) -> str:
    """프로젝트 루트 기반 상대 경로 반환"""
    project_root = detect_project_root()
    try:
        return str(absolute_path.relative_to(project_root))
    except ValueError:
        return str(absolute_path)

def format_json_output(data: Any, pretty: bool = True) -> str:
    """JSON 형식으로 출력 포맷팅"""
    if pretty:
        return json.dumps(data, ensure_ascii=False, indent=2)
    return json.dumps(data, ensure_ascii=False)

class CLIResult:
    """CLI 실행 결과를 표준화하는 클래스"""
    def __init__(self, success: bool, data: Any = None, error: str = None, metadata: Dict[str, Any] = None):
        self.success = success
        self.data = data
        self.error = error
        self.metadata = metadata or {}
        self.timestamp = datetime.now().isoformat()
    
    def to_dict(self) -> Dict[str, Any]:
        """결과를 딕셔너리로 변환"""
        result = {
            "success": self.success,
            "timestamp": self.timestamp,
            "metadata": self.metadata
        }
        
        if self.success:
            result["data"] = self.data
        else:
            result["error"] = self.error
        
        return result
    
    def to_json(self, pretty: bool = True) -> str:
        """결과를 JSON으로 변환"""
        return format_json_output(self.to_dict(), pretty)

class SimpleTodoManager:
    def __init__(self, agent_name: str, session_id: str = None, task_description: str = None):
        self.agent_name = agent_name
        
        # SessionRegistry를 통해 세션 ID 관리
        if SESSION_REGISTRY_AVAILABLE:
            try:
                registry = SessionRegistry.get_instance()
                session_info = registry.get_or_create_session(
                    session_id=session_id,
                    agent_name=agent_name,
                    task_description=task_description,
                    metadata={"tool": "simple_todo", "agent_type": "subagent"}
                )
                self.session_id = session_info.session_id
                self._session_registry = registry
                
                print(f"🗂️ [{self.agent_name}] 지속적인 세션 시작: {self.session_id}")
                print(f"📝 작업: {task_description or '없음'}")
                
            except Exception as e:
                print(f"Warning: SessionRegistry failed: {e}")
                self.session_id = session_id or str(uuid.uuid4())[:8]
                self._session_registry = None
        else:
            # 기존 방식으로 fallback
            if session_id is None:
                self.session_id = str(uuid.uuid4())[:8]
            else:
                self.session_id = session_id
            self._session_registry = None
        
        # 세션별 독립된 디렉토리 생성 (공유 기능 추가)
        # 프로젝트 루트 기반 경로 사용
        project_root = detect_project_root()
        base_dir = project_root / ".opencode" / "sessions"
        shared_dir = base_dir / "shared_sessions"
        
        # 공유 세션 디렉토리 (동일한 session_id를 사용하는 모든 인스턴스가 공유)
        self.shared_todo_dir = shared_dir / f"session_{self.session_id}"
        self.shared_todo_file = self.shared_todo_dir / "todos.json"
        
        # 에이전트별 개인 디렉토리 (호환성 유지)
        self.private_todo_dir = base_dir / f"{self.agent_name}_{self.session_id}"
        self.private_todo_file = self.private_todo_dir / "todos.json"
        
        # 디렉토리 생성
        self.shared_todo_dir.mkdir(parents=True, exist_ok=True)
        self.private_todo_dir.mkdir(parents=True, exist_ok=True)
        
        # 공유 파일을 기본으로 사용, 개인 파일은 백업용
        self.todo_dir = str(self.shared_todo_dir)
        self.todo_file = str(self.shared_todo_file)
        
        # 기존 파일이 없으면 초기화
        if not os.path.exists(self.todo_file):
            self.save_todos([])
        
        print(f"📁 공유 저장 위치: {self.shared_todo_dir}")
        print(f"📁 개인 저장 위치: {self.private_todo_dir}")
    
    def _touch_session(self):
        """세션 접근 시간 업데이트"""
        if self._session_registry:
            try:
                session_info = self._session_registry.get_session(self.session_id)
                if session_info:
                    session_info.touch()
                    self._session_registry._save_registry()
            except Exception:
                pass
    
    def load_todos(self) -> List[Dict[str, Any]]:
        """Todo 목록 로드"""
        self._touch_session()
        
        try:
            with open(self.todo_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except:
            return []
    
    def save_todos(self, todos: List[Dict[str, Any]]) -> bool:
        """Todo 목록 저장"""
        try:
            with open(self.todo_file, 'w', encoding='utf-8') as f:
                json.dump(todos, f, ensure_ascii=False, indent=2)
            return True
        except:
            return False
    
    def add_todo(self, content: str, priority: str = "medium") -> str:
        """새로운 Todo 추가"""
        self._touch_session()
        
        todos = self.load_todos()
        
        new_todo = {
            "id": f"{self.agent_name[:3]}-{len(todos)+1:03d}",
            "content": content,
            "priority": priority,
            "status": "pending",
            "created_at": datetime.now().isoformat(),
            "updated_at": datetime.now().isoformat(),
            "session_id": self.session_id,
            "agent_name": self.agent_name
        }
        
        todos.append(new_todo)
        self.save_todos(todos)
        
        # 개인 파일에도 백업 저장
        try:
            with open(self.private_todo_file, 'w', encoding='utf-8') as f:
                json.dump(todos, f, ensure_ascii=False, indent=2)
        except Exception:
            pass
        
        print(f"✅ [{self.agent_name}] Todo 추가: {new_todo['id']} - {content}")
        return new_todo['id']
    
    def update_status(self, todo_id: str, status: str) -> bool:
        """Todo 상태 업데이트"""
        todos = self.load_todos()
        
        for todo in todos:
            if todo['id'] == todo_id:
                todo['status'] = status
                todo['updated_at'] = datetime.now().isoformat()
                self.save_todos(todos)
                print(f"📝 [{self.agent_name}] Todo 업데이트: {todo_id} -> {status}")
                return True
        
        print(f"❌ [{self.agent_name}] Todo를 찾을 수 없음: {todo_id}")
        return False
    
    def list_todos(self, status: str = None) -> List[Dict[str, Any]]:
        """Todo 목록 조회"""
        todos = self.load_todos()
        
        if status:
            todos = [todo for todo in todos if todo['status'] == status]
        
        if todos:
            print(f"📋 [{self.agent_name}] Todo 목록:")
            for todo in todos:
                status_icon = {"pending": "⏳", "in_progress": "🔄", "completed": "✅"}.get(todo['status'], "❓")
                priority_color = {"high": "🔴", "medium": "🟡", "low": "🟢"}.get(todo['priority'], "⚪")
                print(f"  {status_icon} {priority_color} {todo['id']}: {todo['content']}")
        else:
            print(f"📋 [{self.agent_name}] Todo가 없습니다")
        
        return todos
    
    def get_pending_todos(self) -> List[Dict[str, Any]]:
        """진행 중인 Todo 목록"""
        return self.list_todos("pending")
    
    def get_completed_todos(self) -> List[Dict[str, Any]]:
        """완료된 Todo 목록"""
        return self.list_todos("completed")
    
    def clear_completed(self) -> int:
        """완료된 Todo 삭제"""
        todos = self.load_todos()
        original_count = len(todos)
        
        todos = [todo for todo in todos if todo['status'] != 'completed']
        cleared_count = original_count - len(todos)
        
        if cleared_count > 0:
            self.save_todos(todos)
            print(f"🗑️ [{self.agent_name}] 완료된 Todo {cleared_count}개 삭제")
        
        return cleared_count
    
    def get_session_info(self) -> Dict[str, Any]:
        """세션 정보 조회"""
        todos = self.load_todos()
        
        return {
            "agent_name": self.agent_name,
            "session_id": self.session_id,
            "todo_dir": self.todo_dir,
            "total_todos": len(todos),
            "pending": len([t for t in todos if t['status'] == 'pending']),
            "in_progress": len([t for t in todos if t['status'] == 'in_progress']),
            "completed": len([t for t in todos if t['status'] == 'completed']),
            "created_at": datetime.fromtimestamp(os.path.getctime(self.todo_file)).isoformat() if os.path.exists(self.todo_file) else None
        }
    
    def cleanup_session(self) -> bool:
        """세션 정리 (디렉토리 삭제)"""
        try:
            import shutil
            
            # 개인 디렉토리만 정리 (공유 디렉토리는 다른 인스턴스가 사용할 수 있음)
            if os.path.exists(self.private_todo_dir):
                shutil.rmtree(self.private_todo_dir)
                print(f"🗑️ [{self.agent_name}] 개인 세션 정리 완료: {self.session_id}")
            
            # SessionRegistry에서 세션 종료
            if self._session_registry:
                try:
                    self._session_registry.end_session(self.session_id)
                    print(f"🗑️ [{self.agent_name}] 레지스트리 세션 종료: {self.session_id}")
                except Exception as e:
                    print(f"Warning: Failed to end session in registry: {e}")
            
            return True
        except Exception as e:
            print(f"❌ [{self.agent_name}] 세션 정리 실패: {e}")
        return False
    
    def get_shared_session_info(self) -> Dict[str, Any]:
        """공유 세션 정보 조회"""
        self._touch_session()
        
        todos = self.load_todos()
        
        # 다른 에이전트의 Todo도 포함하여 통계
        agent_stats = {}
        for todo in todos:
            agent = todo.get('agent_name', self.agent_name)
            if agent not in agent_stats:
                agent_stats[agent] = {'pending': 0, 'in_progress': 0, 'completed': 0}
            agent_stats[agent][todo['status']] += 1
        
        return {
            "session_id": self.session_id,
            "shared_todo_dir": str(self.shared_todo_dir),
            "private_todo_dir": str(self.private_todo_dir),
            "total_todos": len(todos),
            "agent_stats": agent_stats,
            "is_persistent": SESSION_REGISTRY_AVAILABLE and self._session_registry is not None,
            "created_at": datetime.fromtimestamp(os.path.getctime(self.shared_todo_dir)).isoformat() if os.path.exists(self.shared_todo_dir) else None
        }
    
    def sync_with_shared(self) -> int:
        """개인 Todo를 공유 세션과 동기화"""
        try:
            # 개인 파일에서 Todo 로드
            private_todos = []
            if os.path.exists(self.private_todo_file):
                with open(self.private_todo_file, 'r', encoding='utf-8') as f:
                    private_todos = json.load(f)
            
            # 공유 파일에서 Todo 로드
            shared_todos = self.load_todos()
            
            # 개인 Todo에만 있는 항목을 공유에 추가
            added_count = 0
            for private_todo in private_todos:
                # 동일한 ID가 공유에 없는 경우에만 추가
                if not any(st['id'] == private_todo['id'] for st in shared_todos):
                    shared_todos.append(private_todo)
                    added_count += 1
            
            if added_count > 0:
                self.save_todos(shared_todos)
                print(f"🔄 [{self.agent_name}] {added_count}개 Todo를 공유 세션에 동기화")
            
            return added_count
        except Exception as e:
            print(f"❌ [{self.agent_name}] 동기화 실패: {e}")
            return 0
    
    def cli_add_todo(self, content: str, priority: str = "medium") -> CLIResult:
        """CLI용 Todo 추가 메서드"""
        try:
            if not content or not content.strip():
                return CLIResult(False, error="Todo 내용은 비어있을 수 없습니다")
            
            todo_id = self.add_todo(content.strip(), priority)
            todos = self.load_todos()
            
            # 추가된 Todo 찾기
            added_todo = next((t for t in todos if t['id'] == todo_id), None)
            
            return CLIResult(
                success=True,
                data=added_todo,
                metadata={
                    "action": "add_todo",
                    "agent_name": self.agent_name,
                    "session_id": self.session_id
                }
            )
        except Exception as e:
            return CLIResult(False, error=f"Todo 추가 실패: {str(e)}")
    
    def cli_update_status(self, todo_id: str, status: str) -> CLIResult:
        """CLI용 Todo 상태 업데이트 메서드"""
        try:
            if not todo_id or not todo_id.strip():
                return CLIResult(False, error="Todo ID는 비어있을 수 없습니다")
            
            if not status or not status.strip():
                return CLIResult(False, error="상태값은 비어있을 수 없습니다")
            
            valid_statuses = ["pending", "in_progress", "completed"]
            if status not in valid_statuses:
                return CLIResult(False, error=f"유효하지 않은 상태값: {status}. 가능한 값: {', '.join(valid_statuses)}")
            
            success = self.update_status(todo_id.strip(), status)
            if success:
                todos = self.load_todos()
                updated_todo = next((t for t in todos if t['id'] == todo_id), None)
                
                return CLIResult(
                    success=True,
                    data=updated_todo,
                    metadata={
                        "action": "update_status",
                        "todo_id": todo_id,
                        "new_status": status
                    }
                )
            else:
                return CLIResult(False, error=f"Todo를 찾을 수 없습니다: {todo_id}")
        
        except Exception as e:
            return CLIResult(False, error=f"Todo 상태 업데이트 실패: {str(e)}")
    
    def cli_list_todos(self, status: str = None) -> CLIResult:
        """CLI용 Todo 목록 조회 메서드"""
        try:
            todos = self.list_todos(status)
            
            return CLIResult(
                success=True,
                data={
                    "todos": todos,
                    "total_count": len(todos),
                    "filter_status": status
                },
                metadata={
                    "action": "list_todos",
                    "agent_name": self.agent_name,
                    "session_id": self.session_id
                }
            )
        except Exception as e:
            return CLIResult(False, error=f"Todo 목록 조회 실패: {str(e)}")
    
    def cli_get_session_info(self) -> CLIResult:
        """CLI용 세션 정보 조회 메서드"""
        try:
            session_info = self.get_session_info()
            shared_info = self.get_shared_session_info()
            
            return CLIResult(
                success=True,
                data={
                    "session_info": session_info,
                    "shared_info": shared_info,
                    "project_root": str(detect_project_root()),
                    "relative_paths": {
                        "todo_dir": get_project_relative_path(Path(self.todo_dir)),
                        "shared_todo_dir": get_project_relative_path(self.shared_todo_dir),
                        "private_todo_dir": get_project_relative_path(self.private_todo_dir)
                    }
                },
                metadata={
                    "action": "get_session_info"
                }
            )
        except Exception as e:
            return CLIResult(False, error=f"세션 정보 조회 실패: {str(e)}")

    @staticmethod
    def list_all_sessions(agent_name: str = None) -> List[Dict[str, Any]]:
        """모든 세션 목록 조회"""
        # 프로젝트 루트 기반 경로 사용
        project_root = detect_project_root()
        sessions_dir = project_root / ".opencode" / "sessions"
        sessions = []
        
        if not sessions_dir.exists():
            return sessions
        
        for session_dir in sessions_dir.iterdir():
            if session_dir.is_dir():
                todo_file = session_dir / "todos.json"
                
                # agent_name 필터링
                if agent_name and not session_dir.name.startswith(f"{agent_name}_"):
                    continue
                
                session_info = {
                    "session_dir": session_dir.name,
                    "path": str(session_dir),
                    "exists": todo_file.exists()
                }
                
                # 세션 디렉토리 이름 파싱 (agent_session_id 형식)
                if '_' in session_dir.name:
                    parts = session_dir.name.rsplit('_', 1)
                    if len(parts) == 2:
                        session_info["agent_name"] = parts[0]
                        session_info["session_id"] = parts[1]
                
                if todo_file.exists():
                    try:
                        with open(todo_file, 'r', encoding='utf-8') as f:
                            todos = json.load(f)
                        session_info["total_todos"] = len(todos)
                        session_info["pending"] = len([t for t in todos if t['status'] == 'pending'])
                        session_info["completed"] = len([t for t in todos if t['status'] == 'completed'])
                        session_info["modified_at"] = datetime.fromtimestamp(todo_file.stat().st_mtime).isoformat()
                    except:
                        pass
                
                sessions.append(session_info)
        
        return sorted(sessions, key=lambda x: x.get('modified_at', ''), reverse=True)

def create_cli_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서 생성"""
    parser = argparse.ArgumentParser(
        prog="simple-todo",
        description="Simple Todo Manager - Subagent용 Todo 관리 CLI 도구",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
사용 예시:
  # Todo 추가
  python simple-todo.py --agent PM --session abc123 --action add --content "프로젝트 계획" --priority high
  
  # Todo 목록 조회
  python simple-todo.py --agent PM --session abc123 --action list
  
  # Todo 상태 업데이트
  python simple-todo.py --agent PM --session abc123 --action update --todo-id PM-001 --status in_progress
  
  # 세션 정보 조회
  python simple-todo.py --agent PM --session abc123 --action info
  
  # JSON 출력 (기본)
  python simple-todo.py --agent PM --session abc123 --action list --output json
  
  # 간결한 JSON 출력
  python simple-todo.py --agent PM --session abc123 --action list --output json --no-pretty
        """
    )
    
    # 필수 인자
    parser.add_argument(
        "--agent", 
        required=True,
        help="에이전트 이름 (예: PM, Dev, Tester)"
    )
    
    parser.add_argument(
        "--action", 
        required=True,
        choices=["add", "update", "list", "info"],
        help="실행할 액션"
    )
    
    # 선택적 인자
    parser.add_argument(
        "--session", 
        help="세션 ID (지정하지 않으면 자동 생성)"
    )
    
    parser.add_argument(
        "--content",
        help="Todo 내용 (add 액션에 필요)"
    )
    
    parser.add_argument(
        "--priority",
        choices=["low", "medium", "high"],
        default="medium",
        help="Todo 우선순위 (기본값: medium)"
    )
    
    parser.add_argument(
        "--todo-id",
        help="Todo ID (update 액션에 필요)"
    )
    
    parser.add_argument(
        "--status",
        choices=["pending", "in_progress", "completed"],
        help="Todo 상태 (update 액션에 필요)"
    )
    
    parser.add_argument(
        "--filter-status",
        choices=["pending", "in_progress", "completed"],
        help="목록 필터링할 상태 (list 액션용)"
    )
    
    parser.add_argument(
        "--output",
        choices=["json", "text"],
        default="json",
        help="출력 형식 (기본값: json)"
    )
    
    parser.add_argument(
        "--no-pretty",
        action="store_true",
        help="JSON 출력 시 간결한 형식 사용"
    )
    
    parser.add_argument(
        "--task-description",
        help="세션 작업 설명"
    )
    
    parser.add_argument(
        "--legacy",
        action="store_true",
        help="레거시 API 사용 (하위 호환성)"
    )
    
    return parser

def execute_cli_action(args: argparse.Namespace) -> CLIResult:
    """CLI 액션 실행"""
    try:
        # SimpleTodoManager 인스턴스 생성
        todo_manager = SimpleTodoManager(
            agent_name=args.agent,
            session_id=args.session,
            task_description=args.task_description
        )
        
        # 액션에 따른 처리
        if args.action == "add":
            if not args.content:
                return CLIResult(False, error="add 액션에는 --content 인자가 필요합니다")
            
            return todo_manager.cli_add_todo(args.content, args.priority)
        
        elif args.action == "update":
            if not args.todo_id:
                return CLIResult(False, error="update 액션에는 --todo-id 인자가 필요합니다")
            
            if not args.status:
                return CLIResult(False, error="update 액션에는 --status 인자가 필요합니다")
            
            return todo_manager.cli_update_status(args.todo_id, args.status)
        
        elif args.action == "list":
            return todo_manager.cli_list_todos(args.filter_status)
        
        elif args.action == "info":
            return todo_manager.cli_get_session_info()
        
        else:
            return CLIResult(False, error=f"지원하지 않는 액션: {args.action}")
    
    except Exception as e:
        return CLIResult(False, error=f"CLI 액션 실행 실패: {str(e)}")

def main():
    """메인 CLI 진입점"""
    try:
        # CLI 인자 파싱
        parser = create_cli_parser()
        
        # 인자가 없으면 도움말 표시
        if len(sys.argv) == 1:
            parser.print_help()
            return
        
        args = parser.parse_args()
        
        # 레거시 모드 처리
        if args.legacy:
            print("⚠️ 레거시 모드로 실행합니다 (하위 호환성)")
            # 기존 방식으로 처리
            todo_manager = SimpleTodoManager(args.agent, args.session, args.task_description)
            
            if args.action == "add" and args.content:
                todo_manager.add_todo(args.content, args.priority)
            elif args.action == "list":
                todo_manager.list_todos(args.filter_status)
            elif args.action == "info":
                info = todo_manager.get_session_info()
                print(f"세션 정보: {json.dumps(info, ensure_ascii=False, indent=2)}")
            
            return
        
        # CLI 액션 실행
        result = execute_cli_action(args)
        
        # 결과 출력
        if args.output == "json":
            print(result.to_json(pretty=not args.no_pretty))
        else:  # text
            if result.success:
                print(f"✅ 성공: {result.metadata.get('action', '알 수 없음')}")
                if result.data:
                    print(f"데이터: {json.dumps(result.data, ensure_ascii=False, indent=2)}")
            else:
                print(f"❌ 실패: {result.error}")
                sys.exit(1)
    
    except KeyboardInterrupt:
        print("\n⚠️ 사용자에 의해 중단되었습니다")
        sys.exit(130)
    except Exception as e:
        error_result = CLIResult(False, error=f"치명적 오류: {str(e)}")
        print(error_result.to_json())
        sys.exit(1)

# 레거시 호환성을 위한 테스트 코드
def run_legacy_tests():
    """레거시 테스트 실행 (하위 호환성 확인)"""
    print("=== Simple Todo Manager - 지속적인 세션 ID 유지 메커니즘 ===\n")
    
    # 테스트 1: 자동 세션 ID 생성 및 공유
    print("🧪 테스트 1: 자동 세션 ID 생성 및 공유")
    print("-" * 40)
    
    # 첫 번째 SimpleTodoManager 인스턴스
    pm_todos = SimpleTodoManager("PM", task_description="프로젝트 관리")
    pm_todos.add_todo("프로젝트 계획 수립", "high")
    pm_todos.add_todo("팀원과 회의", "medium")
    
    # 두 번째 인스턴스 (동일한 세션 ID를 자동으로 사용)
    pm_todos2 = SimpleTodoManager("PM")  # session_id를 지정하지 않음
    pm_todos2.add_todo("일일 보고 작성", "low")
    
    print(f"첫 번째 인스턴스 세션 ID: {pm_todos.session_id}")
    print(f"두 번째 인스턴스 세션 ID: {pm_todos2.session_id}")
    print(f"세션 ID 동일 여부: {pm_todos.session_id == pm_todos2.session_id}")
    
    pm_todos.list_todos()
    
    print("\n" + "="*50 + "\n")
    
    # 테스트 2: 다른 에이전트가 동일한 세션 공유
    print("🧪 테스트 2: 다른 에이전트가 동일한 세션 공유")
    print("-" * 40)
    
    # PM이 생성한 세션 ID를 Dev가 사용
    dev_todos = SimpleTodoManager("Dev", session_id=pm_todos.session_id, task_description="개발 작업")
    dev_todos.add_todo("코드 구현", "high")
    dev_todos.add_todo("테스트 작성", "medium")
    
    # 공유 세션 정보 조회
    shared_info = dev_todos.get_shared_session_info()
    print(f"공유 세션 정보: {shared_info}")
    
    dev_todos.list_todos()
    
    print("\n" + "="*50 + "\n")
    
    # 테스트 3: 세션 레지스트리 기능
    print("🧪 테스트 3: 세션 레지스트리 기능")
    print("-" * 40)
    
    if SESSION_REGISTRY_AVAILABLE:
        registry = SessionRegistry.get_instance()
        active_sessions = registry.list_active_sessions()
        
        print(f"활성 세션 수: {len(active_sessions)}")
        for session_id, session_info in active_sessions.items():
            print(f"  세션 {session_id}:")
            print(f"    - 에이전트: {session_info.agent_name}")
            print(f"    - 작업: {session_info.task_description}")
            print(f"    - 생성 시간: {datetime.fromtimestamp(session_info.created_at).strftime('%H:%M:%S')}")
            print(f"    - 프로세스 ID: {session_info.process_id}")
    else:
        print("SessionRegistry를 사용할 수 없습니다")
    
    print("\n" + "="*50 + "\n")
    
    # 테스트 4: Todo 상태 업데이트 및 동기화
    print("🧪 테스트 4: Todo 상태 업데이트 및 동기화")
    print("-" * 40)
    
    pm_todos.update_status("PM-001", "in_progress")
    pm_todos.update_status("PM-002", "completed")
    dev_todos.update_status("Dev-001", "in_progress")
    
    # 동기화 테스트
    sync_count = pm_todos.sync_with_shared()
    print(f"동기화된 Todo 수: {sync_count}")
    
    # 최종 목록 조회
    print("\n📋 최종 Todo 목록:")
    pm_todos.list_todos()
    
    print("\n" + "="*50 + "\n")
    
    # 테스트 5: 세션 정리
    print("🧪 테스트 5: 세션 정리")
    print("-" * 40)
    
    project_root = detect_project_root()
    sessions_dir = project_root / ".opencode" / "sessions" / "shared_sessions"
    
    print("세션 정리 전:")
    if sessions_dir.exists():
        shared_sessions = [d.name for d in sessions_dir.iterdir() if d.is_dir()]
        print(f"공유 세션 디렉토리: {shared_sessions}")
    
    # 세션 정리
    pm_todos.cleanup_session()
    dev_todos2 = SimpleTodoManager("Dev")  # 새로운 Dev 인스턴스
    dev_todos2.cleanup_session()
    
    print("\n세션 정리 후:")
    if sessions_dir.exists():
        shared_sessions = [d.name for d in sessions_dir.iterdir() if d.is_dir()]
        print(f"공유 세션 디렉토리: {shared_sessions}")
    
    print("\n✅ 모든 테스트 완료!")
    print("\n🎯 주요 기능:")
    print("  ✅ 자동 세션 ID 생성 및 유지")
    print("  ✅ 환경 변수 기반 세션 공유")
    print("  ✅ 파일 기반 세션 레지스트리")
    print("  ✅ 다중 에이전트 세션 공유")
    print("  ✅ 세션 상태 동기화")
    print("  ✅ 자동 세션 정리")

# 메인 진입점
if __name__ == "__main__":
    # CLI 모드 확인
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        run_legacy_tests()
    else:
        main()