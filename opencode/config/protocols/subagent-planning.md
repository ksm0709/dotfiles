# Subagent Planning Protocol

## 요청 형식
Subagent가 PM에게 계획 생성/업데이트를 요청하는 표준 형식

```json
{
  "action": "create_plan|update_plan|complete_task",
  "agent": "senior-sw-engineer|py-code-reviewer|qa",
  "tasks": [
    {
      "content": "작업 내용",
      "priority": "high|medium|low",
      "estimated_time": "30m|1h|2h",
      "dependencies": ["task_id_1", "task_id_2"]
    }
  ]
}
```

## PM Proxy 처리
PM이 Subagent 요청을 todowrite 형식으로 변환

```python
def proxy_subagent_request(request):
    agent_prefix = f"[{request['agent']}]"
    converted_tasks = []
    
    for task in request['tasks']:
        converted_task = {
            "content": f"{agent_prefix} {task['content']}",
            "priority": task['priority'],
            "id": f"{request['agent']}-{uuid4().hex[:6]}"
        }
        converted_tasks.append(converted_task)
    
    return todowrite(converted_tasks)
```

## 상태 동기화
- Subagent 작업 완료 시 PM에 알림
- PM이 todoread로 현재 상태 조회
- 다음 작업 자동 할당