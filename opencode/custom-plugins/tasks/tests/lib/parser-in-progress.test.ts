import { Parser } from '../../src/lib/parser';
import { TaskList, TaskStatus } from '../../src/types';

describe('Parser - In Progress Status Support', () => {
  let parser: Parser;

  beforeEach(() => {
    parser = new Parser();
  });

  describe('parseTaskList with [~] checkbox', () => {
    it('should parse [~] as in_progress status (ADDED-001)', () => {
      const content = `# Task List: Test Project

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: abc-123

---

## 작업 목록 (Task List)

- [ ] 1. Pending task
- [~] 2. In Progress task
- [x] 3. Completed task

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Implementation  
**상태**: in_progress  
**완료율**: 33% (1/3)
`;

      const result = parser.parseTaskList(content);

      expect(result.tasks).toHaveLength(3);
      expect(result.tasks[0].status).toBe('pending');
      expect(result.tasks[1].status).toBe('in_progress');
      expect(result.tasks[2].status).toBe('completed');
    });

    it('should parse in_progress status with hierarchical IDs', () => {
      const content = `# Task List: Test

**에이전트**: agent  
**생성일**: 2026-01-30  
**세션 ID**: abc

---

## 작업 목록 (Task List)

- [ ] 1. Parent task
- [~] 1.1. Subtask in progress
- [x] 1.2. Subtask completed

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Development  
**상태**: in_progress  
**완료율**: 33% (1/3)
`;

      const result = parser.parseTaskList(content);

      expect(result.tasks).toHaveLength(3);
      expect(result.tasks[0].status).toBe('pending');
      expect(result.tasks[1].id).toBe('1.1');
      expect(result.tasks[1].status).toBe('in_progress');
      expect(result.tasks[2].id).toBe('1.2');
      expect(result.tasks[2].status).toBe('completed');
    });
  });

  describe('generateTaskList with in_progress status', () => {
    it('should generate [~] for in_progress tasks (ADDED-001)', () => {
      const taskList: TaskList = {
        title: 'Test Project',
        agent: 'test-agent',
        createdAt: '2026-01-30 10:00:00',
        sessionId: 'abc-123',
        tasks: [
          {
            id: '1',
            title: 'Pending task',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '2',
            title: 'In Progress task',
            status: 'in_progress' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '3',
            title: 'Completed task',
            status: 'completed' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ],
        currentPhase: 'Implementation'
      };

      const markdown = parser.generateTaskList(taskList);

      expect(markdown).toContain('- [ ] 1. Pending task');
      expect(markdown).toContain('- [~] 2. In Progress task');
      expect(markdown).toContain('- [x] 3. Completed task');
    });

    it('should generate [~] for in_progress subtasks with hierarchical IDs', () => {
      const taskList: TaskList = {
        title: 'Nested Test',
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
            title: 'Child in progress',
            status: 'in_progress' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const markdown = parser.generateTaskList(taskList);

      expect(markdown).toContain('- [ ] 1. Parent');
      expect(markdown).toContain('- [~] 1.1. Child in progress');
    });
  });

  describe('updateTaskStatus to in_progress', () => {
    it('should update task status to in_progress (ADDED-001)', () => {
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

      const result = parser.updateTaskStatus(taskList, '1', 'in_progress');

      expect(result).toBe(true);
      expect(taskList.tasks[0].status).toBe('in_progress');
      expect(taskList.tasks[0].updatedAt).toBeDefined();
    });

    it('should update subtask status to in_progress in flat structure', () => {
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

      const result = parser.updateTaskStatus(taskList, '1.1', 'in_progress');

      expect(result).toBe(true);
      expect(taskList.tasks[1].status).toBe('in_progress');
    });
  });

  describe('round-trip with in_progress status', () => {
    it('should preserve in_progress status through round-trip (ADDED-001)', () => {
      const original: TaskList = {
        title: 'Round Trip Test',
        agent: 'test-agent',
        createdAt: '2026-01-30 10:00:00',
        sessionId: 'abc-123',
        tasks: [
          {
            id: '1',
            title: 'Pending task',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '2',
            title: 'In Progress task',
            status: 'in_progress' as TaskStatus,
            details: ['Detail A'],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '3',
            title: 'Completed task',
            status: 'completed' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const markdown = parser.generateTaskList(original);
      const parsed = parser.parseTaskList(markdown);

      expect(parsed.tasks).toHaveLength(3);
      expect(parsed.tasks[0].status).toBe('pending');
      expect(parsed.tasks[1].status).toBe('in_progress');
      expect(parsed.tasks[2].status).toBe('completed');
    });
  });

  describe('formatTask checkbox generation', () => {
    it('should generate correct checkbox for each status', () => {
      const testCases: Array<{ status: TaskStatus; expectedCheckbox: string }> = [
        { status: 'pending' as TaskStatus, expectedCheckbox: '[ ]' },
        { status: 'in_progress' as TaskStatus, expectedCheckbox: '[~]' },
        { status: 'completed' as TaskStatus, expectedCheckbox: '[x]' }
      ];

      testCases.forEach(({ status, expectedCheckbox }) => {
        const taskList: TaskList = {
          title: 'Checkbox Test',
          agent: 'agent',
          createdAt: '2026-01-30',
          sessionId: 'abc',
          tasks: [
            {
              id: '1',
              title: 'Test task',
              status: status,
              details: [],
              createdAt: '2026-01-30T10:00:00.000Z',
              updatedAt: '2026-01-30T10:00:00.000Z'
            }
          ]
        };

        const markdown = parser.generateTaskList(taskList);
        expect(markdown).toContain(`- ${expectedCheckbox} 1. Test task`);
      });
    });
  });
});
