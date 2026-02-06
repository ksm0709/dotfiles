import { Parser } from '../../src/lib/parser';
import { TaskList, TaskStatus } from '../../src/types';

describe('Parser', () => {
  let parser: Parser;

  beforeEach(() => {
    parser = new Parser();
  });

  describe('parseTaskList', () => {
    it('should parse basic task list header', () => {
      const content = `# Task List: Test Project

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: abc-123

---

## 작업 목록 (Task List)

- [ ] 1. First task
- [x] 2. Second task

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Planning  
**상태**: in_progress  
**완료율**: 50% (1/2)

---

## 변경 이력 (Change Log)

| 시간 | 작업 | 상태 |
|------|------|------|
| 2026-01-30 10:00 | 작업 목록 생성 | created |
`;

      const result = parser.parseTaskList(content);

      expect(result.title).toBe('Test Project');
      expect(result.agent).toBe('test-agent');
      expect(result.createdAt).toBe('2026-01-30 10:00:00');
      expect(result.sessionId).toBe('abc-123');
      expect(result.tasks).toHaveLength(2);
    });

    it('should parse task status correctly', () => {
      const content = `# Task List: Test

**에이전트**: agent  
**생성일**: 2026-01-30  
**세션 ID**: abc

---

## 작업 목록 (Task List)

- [ ] 1. Pending task
- [x] 2. Completed task
- [ ] 3. Another pending

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Testing  
**상태**: in_progress  
**완료율**: 33% (1/3)
`;

      const result = parser.parseTaskList(content);

      expect(result.tasks[0].status).toBe('pending');
      expect(result.tasks[1].status).toBe('completed');
      expect(result.tasks[2].status).toBe('pending');
    });

    it('should parse task details', () => {
      const content = `# Task List: Test

**에이전트**: agent  
**생성일**: 2026-01-30  
**세션 ID**: abc

---

## 작업 목록 (Task List)

- [ ] 1. Main task
  - Detail 1
  - Detail 2
  - Detail 3

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Planning  
**상태**: pending  
**완료율**: 0% (0/1)
`;

      const result = parser.parseTaskList(content);

      expect(result.tasks[0].details).toHaveLength(3);
      expect(result.tasks[0].details).toContain('Detail 1');
      expect(result.tasks[0].details).toContain('Detail 2');
      expect(result.tasks[0].details).toContain('Detail 3');
    });

    it('should parse flat subtasks with hierarchical IDs (1.1, 1.2, etc.)', () => {
      const content = `# Task List: Test

**에이전트**: agent  
**생성일**: 2026-01-30  
**세션 ID**: abc

---

## 작업 목록 (Task List)

- [ ] 1. Parent task
- [ ] 1.1. Subtask 1
- [x] 1.2. Subtask 2
- [ ] 1.3. Subtask 3

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Implementation  
**상태**: in_progress  
**완료율**: 25% (1/4)
`;

      const result = parser.parseTaskList(content);

      // Flat structure: all tasks at same level
      expect(result.tasks).toHaveLength(4);
      expect(result.tasks[0].id).toBe('1');
      expect(result.tasks[1].id).toBe('1.1');
      expect(result.tasks[2].id).toBe('1.2');
      expect(result.tasks[3].id).toBe('1.3');
    });

    it('should parse current phase and memo', () => {
      const content = `# Task List: Test

**에이전트**: agent  
**생성일**: 2026-01-30  
**세션 ID**: abc

---

## 작업 목록 (Task List)

- [ ] 1. Task

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Testing Phase  
**상태**: in_progress  
**완료율**: 50% (1/2)  
**메모**: This is a test memo
`;

      const result = parser.parseTaskList(content);

      expect(result.currentPhase).toBe('Testing Phase');
      expect(result.memo).toBe('This is a test memo');
    });

    it('should handle empty task list', () => {
      const content = `# Task List: Empty

**에이전트**: agent  
**생성일**: 2026-01-30  
**세션 ID**: abc

---

## 작업 목록 (Task List)

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: 미정  
**상태**: pending  
**완료율**: 0% (0/0)
`;

      const result = parser.parseTaskList(content);

      expect(result.tasks).toHaveLength(0);
    });
  });

  describe('generateTaskList', () => {
    it('should generate valid markdown', () => {
      const taskList: TaskList = {
        title: 'Test Project',
        agent: 'test-agent',
        createdAt: '2026-01-30 10:00:00',
        sessionId: 'abc-123',
        tasks: [
          {
            id: '1',
            title: 'First task',
            status: 'pending' as TaskStatus,
            details: ['Detail 1', 'Detail 2'],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '2',
            title: 'Second task',
            status: 'completed' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ],
        currentPhase: 'Planning'
      };

      const markdown = parser.generateTaskList(taskList);

      expect(markdown).toContain('# Task List: Test Project');
      expect(markdown).toContain('**에이전트**: test-agent');
      expect(markdown).toContain('- [ ] 1. First task');
      expect(markdown).toContain('- [x] 2. Second task');
      expect(markdown).toContain('  - Detail 1');
      expect(markdown).toContain('  - Detail 2');
      expect(markdown).toContain('**현재 단계**: Planning');
      expect(markdown).toContain('**완료율**: 50% (1/2)');
    });

    it('should include flat subtasks with hierarchical IDs in markdown', () => {
      const taskList: TaskList = {
        title: 'Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Parent',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '1.1',
            title: 'Child 1',
            status: 'completed' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const markdown = parser.generateTaskList(taskList);

      // Flat structure: no indentation for subtasks
      expect(markdown).toContain('- [ ] 1. Parent');
      expect(markdown).toContain('- [x] 1.1. Child 1');
    });

    it('should include memo if present', () => {
      const taskList: TaskList = {
        title: 'Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [],
        currentPhase: 'Testing',
        memo: 'Important note'
      };

      const markdown = parser.generateTaskList(taskList);

      expect(markdown).toContain('**메모**: Important note');
    });
  });

  describe('updateTaskStatus', () => {
    it('should update top-level task status', () => {
      const taskList: TaskList = {
        title: 'Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Task 1',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const result = parser.updateTaskStatus(taskList, '1', 'completed');

      expect(result).toBe(true);
      expect(taskList.tasks[0].status).toBe('completed');
      expect(taskList.tasks[0].updatedAt).toBeDefined();
    });

    it('should update subtask status by ID in flat structure', () => {
      const taskList: TaskList = {
        title: 'Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Parent',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '1.1',
            title: 'Child',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const result = parser.updateTaskStatus(taskList, '1.1', 'completed');

      expect(result).toBe(true);
      expect(taskList.tasks[1].status).toBe('completed');
    });

    it('should return false for non-existent task', () => {
      const taskList: TaskList = {
        title: 'Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: []
      };

      const result = parser.updateTaskStatus(taskList, '999', 'completed');

      expect(result).toBe(false);
    });
  });

  describe('addTask', () => {
    it('should add task with auto-generated ID', () => {
      const taskList: TaskList = {
        title: 'Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Existing',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const result = parser.addTask(taskList, 'New Task', ['Detail 1']);

      expect(result).toBe(true);
      expect(taskList.tasks).toHaveLength(2);
      expect(taskList.tasks[1].id).toBe('2');
      expect(taskList.tasks[1].title).toBe('New Task');
      expect(taskList.tasks[1].details).toContain('Detail 1');
    });

    it('should add subtask with hierarchical ID when specified', () => {
      const taskList: TaskList = {
        title: 'Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Parent',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      // In flat structure, "subtask" is just another task with hierarchical ID
      // The agent should specify the ID like "1.1" manually
      const result = parser.addTask(taskList, 'Child Task', []);

      expect(result).toBe(true);
      expect(taskList.tasks).toHaveLength(2);
      expect(taskList.tasks[1].id).toBe('2'); // Auto-generated, not hierarchical
    });

    it('should handle adding first task', () => {
      const taskList: TaskList = {
        title: 'Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: []
      };

      const result = parser.addTask(taskList, 'First Task', []);

      expect(result).toBe(true);
      expect(taskList.tasks).toHaveLength(1);
      expect(taskList.tasks[0].id).toBe('1');
    });
  });

  describe('removeTask', () => {
    it('should remove task by ID in flat structure', () => {
      const taskList: TaskList = {
        title: 'Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'To Remove',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const result = parser.removeTask(taskList, '1');

      expect(result).toBe(true);
      expect(taskList.tasks).toHaveLength(0);
    });

    it('should remove subtask by ID in flat structure', () => {
      const taskList: TaskList = {
        title: 'Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Parent',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '1.1',
            title: 'To Remove',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const result = parser.removeTask(taskList, '1.1');

      expect(result).toBe(true);
      expect(taskList.tasks).toHaveLength(1);
    });

    it('should return false for non-existent task', () => {
      const taskList: TaskList = {
        title: 'Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: []
      };

      const result = parser.removeTask(taskList, '999');

      expect(result).toBe(false);
    });
  });

  describe('round-trip parsing', () => {
    it('should correctly parse generated markdown', () => {
      const original: TaskList = {
        title: 'Round Trip Test',
        agent: 'test-agent',
        createdAt: '2026-01-30 10:00:00',
        sessionId: 'abc-123',
        tasks: [
          {
            id: '1',
            title: 'Task with details',
            status: 'in_progress' as TaskStatus,
            details: ['Detail A', 'Detail B'],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '1.1',
            title: 'Subtask in flat structure',
            status: 'completed' as TaskStatus,
            details: ['Sub detail'],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ],
        currentPhase: 'Testing',
        memo: 'Test memo'
      };

      const markdown = parser.generateTaskList(original);
      const parsed = parser.parseTaskList(markdown);

      expect(parsed.title).toBe(original.title);
      expect(parsed.agent).toBe(original.agent);
      expect(parsed.tasks).toHaveLength(2);
      expect(parsed.tasks[0].title).toBe('Task with details');
      // Note: Markdown format now supports three statuses: completed ([x]), in_progress ([~]), and pending ([ ])
      // 'in_progress' status is now preserved through round-trip (ADDED-001)
      expect(parsed.tasks[0].status).toBe('in_progress');
      expect(parsed.tasks[1].id).toBe('1.1');
      expect(parsed.currentPhase).toBe('Testing');
    });
  });
});
