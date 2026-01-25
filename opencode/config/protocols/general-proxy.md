# General Proxy Protocol

## 개요
어떤 primary agent든 subagent들의 계획 요청을 proxy 처리할 수 있는 일반화된 통신 프로토콜

## 핵심 원칙
1. **Agent Agnostic**: 특정 agent에 종속되지 않음
2. **Dynamic Proxy**: 상황에 따라 최적의 proxy 자동 선택
3. **Capability-Based**: agent의 도구 사용 가능성 기반 분배
4. **Fallback Safe**: proxy 없을 시 대안 제공

## 통신 형식

### 1. 계획 생성 요청
```json
{
  "action": "create_plan",
  "requester": "senior-sw-engineer",
  "tasks": [
    {
      "content": "API 엔드포인트 구현",
      "priority": "high",
      "estimated_time": "2h",
      "dependencies": []
    }
  ],
  "preferences": {
    "preferred_proxy": "pm",
    "fallback_allowed": true
  }
}
```

### 2. 상태 업데이트 요청
```json
{
  "action": "update_status",
  "requester": "py-code-reviewer",
  "task_id": "pyr-abc123",
  "status": "completed",
  "metadata": {
    "completion_time": "45m",
    "issues_found": 3
  }
}
```

### 3. Proxy 응답
```json
{
  "success": true,
  "proxy": "general",
  "requester": "senior-sw-engineer",
  "tasks_created": 2,
  "task_ids": ["sse-abc123", "sse-def456"],
  "execution_time": "0.2s"
}
```

## Agent Capability Matrix

| Agent | todowrite | todoread | bash | webfetch | Priority |
|-------|-----------|----------|------|----------|----------|
| pm | ✅ | ✅ | ✅ | ❌ | 1 |
| general | ✅ | ✅ | ❌ | ✅ | 2 |
| planner | ✅ | ✅ | ❌ | ❌ | 3 |
| senior-sw-engineer | ❌ | ❌ | ✅ | ❌ | N/A |
| py-code-reviewer | ❌ | ❌ | ❌ | ❌ | N/A |

## Proxy 선택 알고리즘

### 1. 기본 선택 (Priority-based)
```python
def select_proxy(required_tools):
    for agent in sorted(capable_proxies, key=priority):
        if all(tool in capable_proxies[agent]['tools'] for tool in required_tools):
            if is_agent_available(agent):
                return agent
    return None
```

### 2. 로드 밸런싱 (Load-based)
```python
def select_proxy_with_load(required_tools):
    candidates = get_capable_agents(required_tools)
    return min(candidates, key=lambda x: get_current_load(x))
```

### 3. 전문성 기반 (Expertise-based)
```python
def select_proxy_by_expertise(task_type):
    expertise_map = {
        'coding': 'pm',
        'research': 'general',
        'planning': 'planner'
    }
    return expertise_map.get(task_type, select_proxy([]))
```

## Fallback 전략

### 1. Primary Proxy 실패 시
1. 다음 우선순위 capable proxy 시도
2. 모든 proxy 실패 시 파일 기반 백업
3. 사용자에게 수동 처리 요청

### 2. 부하 분산
```python
def distribute_tasks(tasks):
    proxies = get_available_proxies()
    return {
        proxy: assign_tasks(proxy, tasks) 
        for proxy in proxies
    }
```

## 보안 및 검증

### 1. 요청자 검증
```python
def validate_requester(requester, action):
    allowed_actions = agent_permissions.get(requester, [])
    return action in allowed_actions
```

### 2. 작업 내용 검증
```python
def validate_task_content(task):
    # 악의적인 명령어, PII, 독성 콘텐츠 검증
    return is_safe_content(task['content'])
```

## 구현 단계

### Phase 1: Basic Proxy (현재)
- 단일 proxy 기반 요청 처리
- 간단한 fallback 메커니즘

### Phase 2: Dynamic Detection (1개월)
- Agent capability 자동 감지
- 로드 밸런싱 기반 proxy 선택

### Phase 3: Intelligent Coordination (3개월)
- 다중 proxy 조율
- 예측 기반 사전 할당