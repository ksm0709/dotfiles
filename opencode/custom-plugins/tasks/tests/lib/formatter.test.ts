import { Formatter } from '../../src/lib/formatter';
import { TaskList, TaskDetail, TaskStatusSummary, TaskStatus } from '../../src/types';

describe('Formatter', () => {
  let formatter: Formatter;

  beforeEach(() => {
    formatter = new Formatter();
  });

  describe('formatAsMarkdown', () => {
    it('should format task list as markdown', () => {
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
            subtasks: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '2',
            title: 'Second task',
            status: 'completed' as TaskStatus,
            details: [],
            subtasks: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ],
        currentPhase: 'Planning'
      };

      const markdown = formatter.formatAsMarkdown(taskList);

      expect(markdown).toContain('# Task List: Test Project');
      expect(markdown).toContain('**에이전트**: test-agent');
      expect(markdown).toContain('**생성일**: 2026-01-30 10:00:00');
      expect(markdown).toContain('**세션 ID**: abc-123');
      expect(markdown).toContain('- [ ] ⏳ **1**. First task');
      expect(markdown).toContain('- [x] ✅ **2**. Second task');
      expect(markdown).toContain('  - Detail 1');
      expect(markdown).toContain('  - Detail 2');
    });

    it('should format nested subtasks', () => {
      const taskList: TaskList = {
        title: 'Nested Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Parent task',
            status: 'in_progress' as TaskStatus,
            details: [],
            subtasks: [
              {
                id: '1.1',
                title: 'Child 1',
                status: 'completed' as TaskStatus,
                details: [],
                subtasks: [
                  {
                    id: '1.1.1',
                    title: 'Grandchild',
                    status: 'pending' as TaskStatus,
                    details: [],
                    createdAt: '2026-01-30T10:00:00.000Z',
                    updatedAt: '2026-01-30T10:00:00.000Z'
                  }
                ],
                createdAt: '2026-01-30T10:00:00.000Z',
                updatedAt: '2026-01-30T10:00:00.000Z'
              }
            ],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const markdown = formatter.formatAsMarkdown(taskList);

      expect(markdown).toContain('- [ ] 🔄 **1**. Parent task');
      expect(markdown).toContain('  - [x] ✅ **1.1**. Child 1');
      expect(markdown).toContain('    - [ ] ⏳ **1.1.1**. Grandchild');
    });

    it('should include memo if present', () => {
      const taskList: TaskList = {
        title: 'Memo Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [],
        currentPhase: 'Testing',
        memo: 'This is an important memo'
      };

      const markdown = formatter.formatAsMarkdown(taskList);

      expect(markdown).toContain('**메모**: This is an important memo');
    });

    it('should show default phase when not set', () => {
      const taskList: TaskList = {
        title: 'No Phase',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: []
      };

      const markdown = formatter.formatAsMarkdown(taskList);

      expect(markdown).toContain('**현재 단계**: 미정');
    });

    it('should calculate completion stats correctly', () => {
      const taskList: TaskList = {
        title: 'Stats Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Task 1',
            status: 'completed' as TaskStatus,
            details: [],
            subtasks: [
              {
                id: '1.1',
                title: 'Subtask 1',
                status: 'completed' as TaskStatus,
                details: [],
                createdAt: '2026-01-30T10:00:00.000Z',
                updatedAt: '2026-01-30T10:00:00.000Z'
              }
            ],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '2',
            title: 'Task 2',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const markdown = formatter.formatAsMarkdown(taskList);

      // 2 completed out of 3 total tasks (including subtask)
      expect(markdown).toContain('**완료율**: 67% (2/3)');
      expect(markdown).toContain('**상태**: in_progress');
    });

    it('should handle 100% completion', () => {
      const taskList: TaskList = {
        title: 'Complete Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Task 1',
            status: 'completed' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const markdown = formatter.formatAsMarkdown(taskList);

      expect(markdown).toContain('**완료율**: 100% (1/1)');
      expect(markdown).toContain('**상태**: completed');
    });

    it('should handle empty task list', () => {
      const taskList: TaskList = {
        title: 'Empty Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: []
      };

      const markdown = formatter.formatAsMarkdown(taskList);

      expect(markdown).toContain('**완료율**: 0% (0/0)');
      expect(markdown).toContain('**상태**: pending');
    });
  });

  describe('formatAsJSON', () => {
    it('should format as pretty-printed JSON', () => {
      const taskList: TaskList = {
        title: 'JSON Test',
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

      const json = formatter.formatAsJSON(taskList);

      // Should be valid JSON
      expect(() => JSON.parse(json)).not.toThrow();

      // Should be pretty-printed (indented)
      expect(json).toContain('\n');
      expect(json).toContain('  ');

      // Should contain all data
      const parsed = JSON.parse(json);
      expect(parsed.title).toBe('JSON Test');
      expect(parsed.tasks).toHaveLength(1);
    });
  });

  describe('formatAsTable', () => {
    it('should format as markdown table', () => {
      const taskList: TaskList = {
        title: 'Table Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Task 1',
            status: 'completed' as TaskStatus,
            details: ['Detail A'],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '2',
            title: 'Task 2',
            status: 'in_progress' as TaskStatus,
            details: ['Detail B', 'Detail C'],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const table = formatter.formatAsTable(taskList);

      expect(table).toContain('# Table Test');
      expect(table).toContain('| ID | 상태 | 제목 | 세부사항 |');
      expect(table).toContain('✅ 완료');
      expect(table).toContain('🔄 진행중');
      expect(table).toContain('| 1 |');
      expect(table).toContain('| 2 |');
    });

    it('should truncate long details', () => {
      const taskList: TaskList = {
        title: 'Truncate Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Task 1',
            status: 'pending' as TaskStatus,
            details: ['This is a very long detail that should be truncated at 30 chars'],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const table = formatter.formatAsTable(taskList);

      // Details are truncated at 30 characters
      expect(table).toContain('This is a very long detail tha');
    });

    it('should format nested subtasks with indentation', () => {
      const taskList: TaskList = {
        title: 'Nested Table Test',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Parent',
            status: 'pending' as TaskStatus,
            details: [],
            subtasks: [
              {
                id: '1.1',
                title: 'Child',
                status: 'completed' as TaskStatus,
                details: [],
                createdAt: '2026-01-30T10:00:00.000Z',
                updatedAt: '2026-01-30T10:00:00.000Z'
              }
            ],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const table = formatter.formatAsTable(taskList);

      expect(table).toContain('Parent');
      expect(table).toContain('  Child'); // Indented
    });

    it('should handle empty task list', () => {
      const taskList: TaskList = {
        title: 'Empty Table',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: []
      };

      const table = formatter.formatAsTable(taskList);

      expect(table).toContain('# Empty Table');
      expect(table).toContain('| ID | 상태 | 제목 | 세부사항 |');
    });
  });

  describe('formatStatusSummary', () => {
    it('should format completed status', () => {
      const summary: TaskStatusSummary = {
        agent: 'test-agent',
        title: 'Completed Project',
        status: 'completed' as TaskStatus,
        completionRate: 100,
        completedCount: 5,
        totalCount: 5,
        currentPhase: 'Done'
      };

      const formatted = formatter.formatStatusSummary(summary);

      expect(formatted).toContain('📋 Task Status Summary');
      expect(formatted).toContain('에이전트: test-agent');
      expect(formatted).toContain('작업: Completed Project');
      expect(formatted).toContain('✅ completed');
      expect(formatted).toContain('완료율: 100% (5/5)');
      expect(formatted).toContain('현재 단계: Done');
      expect(formatted).toContain('██████████'); // Full progress bar
    });

    it('should format in_progress status', () => {
      const summary: TaskStatusSummary = {
        agent: 'test-agent',
        title: 'In Progress Project',
        status: 'in_progress' as TaskStatus,
        completionRate: 50,
        completedCount: 2,
        totalCount: 4
      };

      const formatted = formatter.formatStatusSummary(summary);

      expect(formatted).toContain('🔄 in_progress');
      expect(formatted).toContain('50%');
      expect(formatted).toContain('██████░░░░'); // Half-filled progress bar
      expect(formatted).not.toContain('현재 단계'); // No current phase
    });

    it('should format pending status', () => {
      const summary: TaskStatusSummary = {
        agent: 'test-agent',
        title: 'Pending Project',
        status: 'pending' as TaskStatus,
        completionRate: 0,
        completedCount: 0,
        totalCount: 3
      };

      const formatted = formatter.formatStatusSummary(summary);

      expect(formatted).toContain('⏳ pending');
      expect(formatted).toContain('0%');
      expect(formatted).toContain('░░░░░░░░░░'); // Empty progress bar
    });

    it('should handle 0% completion rate', () => {
      const summary: TaskStatusSummary = {
        agent: 'test-agent',
        title: 'Zero Project',
        status: 'pending' as TaskStatus,
        completionRate: 0,
        completedCount: 0,
        totalCount: 10
      };

      const formatted = formatter.formatStatusSummary(summary);

      expect(formatted).toContain('[░░░░░░░░░░░░░░░░░░░░] 0%');
    });

    it('should handle 100% completion rate', () => {
      const summary: TaskStatusSummary = {
        agent: 'test-agent',
        title: 'Full Project',
        status: 'completed' as TaskStatus,
        completionRate: 100,
        completedCount: 10,
        totalCount: 10
      };

      const formatted = formatter.formatStatusSummary(summary);

      expect(formatted).toContain('100%');
      expect(formatted).toContain('█');
    });

    it('should handle intermediate completion rates', () => {
      const summary: TaskStatusSummary = {
        agent: 'test-agent',
        title: 'Partial Project',
        status: 'in_progress' as TaskStatus,
        completionRate: 75,
        completedCount: 3,
        totalCount: 4
      };

      const formatted = formatter.formatStatusSummary(summary);

      // 75% of 20 characters = 15 filled
      expect(formatted).toContain('75%');
    });
  });
});
