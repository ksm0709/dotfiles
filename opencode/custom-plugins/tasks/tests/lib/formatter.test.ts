import { Formatter } from '../../src/lib/formatter';
import { TaskList, TaskDetail, TaskStatusSummary, TaskStatus, BatchResult, StatusSummary } from '../../src/types';

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

    it('should format flat tasks with hierarchical IDs (1.1, 1.1.1)', () => {
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
          },
          {
            id: '1.1.1',
            title: 'Grandchild',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const markdown = formatter.formatAsMarkdown(taskList);

      // Flat structure: no indentation, all at same level
      expect(markdown).toContain('- [ ] 🔄 **1**. Parent task');
      expect(markdown).toContain('- [x] ✅ **1.1**. Child 1');
      expect(markdown).toContain('- [ ] ⏳ **1.1.1**. Grandchild');
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

    it('should calculate completion stats correctly for flat structure', () => {
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
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '1.1',
            title: 'Subtask 1',
            status: 'completed' as TaskStatus,
            details: [],
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

      // 2 completed out of 3 total tasks (flat structure, no nesting)
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

    it('should format flat tasks without indentation', () => {
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
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '1.1',
            title: 'Child',
            status: 'completed' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const table = formatter.formatAsTable(taskList);

      // Flat structure: no indentation
      expect(table).toContain('Parent');
      expect(table).toContain('Child');
      expect(table).not.toContain('  Child');
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

  describe('formatTaskListWithStatus', () => {
    it('should format task list with status summary', () => {
      const taskList: TaskList = {
        title: 'Status Test',
        agent: 'test-agent',
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
          },
          {
            id: '2',
            title: 'Task 2',
            status: 'in_progress' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '3',
            title: 'Task 3',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const summary: StatusSummary = {
        agent: 'test-agent',
        title: 'Status Test',
        total: 3,
        completed: 1,
        inProgress: 1,
        pending: 1,
        completionRate: 33
      };

      const formatted = formatter.formatTaskListWithStatus(taskList, summary);

      expect(formatted).toContain('# 📋 Status Test');
      expect(formatted).toContain('**에이전트**: test-agent');
      expect(formatted).toContain('## 📊 진행 상황');
      expect(formatted).toContain('| 상태 | 개수 | 비율 |');
      expect(formatted).toContain('| ✅ 완료 | 1 |');
      expect(formatted).toContain('| 🔄 진행중 | 1 |');
      expect(formatted).toContain('| ⏳ 대기 | 1 |');
      expect(formatted).toContain('### 진행률');
      expect(formatted).toContain('## 📋 작업 목록');
      expect(formatted).toContain('- [x] ✅ **1**. Task 1');
      expect(formatted).toContain('- [ ] 🔄 **2**. Task 2');
      expect(formatted).toContain('- [ ] ⏳ **3**. Task 3');
    });
  });

  describe('formatProgressBar', () => {
    it('should format 0% progress bar', () => {
      const bar = formatter.formatProgressBar(0);
      expect(bar).toBe('[░░░░░░░░░░░░░░░░░░░░] 0%');
    });

    it('should format 50% progress bar', () => {
      const bar = formatter.formatProgressBar(50);
      expect(bar).toContain('50%');
      expect(bar).toContain('█');
      expect(bar).toContain('░');
    });

    it('should format 100% progress bar', () => {
      const bar = formatter.formatProgressBar(100);
      expect(bar).toBe('[████████████████████] 100%');
    });

    it('should format custom length progress bar', () => {
      const bar = formatter.formatProgressBar(50, 10);
      expect(bar).toBe('[█████░░░░░] 50%');
    });

    it('should clamp percentage to 0-100 range', () => {
      const barNegative = formatter.formatProgressBar(-10);
      const barOver100 = formatter.formatProgressBar(150);
      
      expect(barNegative).toBe('[░░░░░░░░░░░░░░░░░░░░] 0%');
      expect(barOver100).toBe('[████████████████████] 100%');
    });
  });

  describe('calculateStatusSummary', () => {
    it('should calculate correct status summary', () => {
      const taskList: TaskList = {
        title: 'Calc Test',
        agent: 'test-agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Completed Task',
            status: 'completed' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '2',
            title: 'In Progress Task',
            status: 'in_progress' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          },
          {
            id: '3',
            title: 'Pending Task',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const summary = formatter.calculateStatusSummary(taskList);

      expect(summary.agent).toBe('test-agent');
      expect(summary.title).toBe('Calc Test');
      expect(summary.total).toBe(3);
      expect(summary.completed).toBe(1);
      expect(summary.inProgress).toBe(1);
      expect(summary.pending).toBe(1);
      expect(summary.completionRate).toBe(33);
    });

    it('should count flat tasks without subtask nesting', () => {
      const taskList: TaskList = {
        title: 'Flat Calc Test',
        agent: 'test-agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [
          {
            id: '1',
            title: 'Parent',
            status: 'in_progress' as TaskStatus,
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
          },
          {
            id: '1.2',
            title: 'Child 2',
            status: 'pending' as TaskStatus,
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ]
      };

      const summary = formatter.calculateStatusSummary(taskList);

      // Flat structure: all 3 tasks counted as separate
      expect(summary.total).toBe(3);
      expect(summary.completed).toBe(1);
      expect(summary.inProgress).toBe(1);
      expect(summary.pending).toBe(1);
      expect(summary.completionRate).toBe(33);
    });

    it('should handle empty task list', () => {
      const taskList: TaskList = {
        title: 'Empty',
        agent: 'test-agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: []
      };

      const summary = formatter.calculateStatusSummary(taskList);

      expect(summary.total).toBe(0);
      expect(summary.completed).toBe(0);
      expect(summary.inProgress).toBe(0);
      expect(summary.pending).toBe(0);
      expect(summary.completionRate).toBe(0);
    });
  });

  describe('formatBatchResult', () => {
    it('should format successful batch results', () => {
      const results: BatchResult[] = [
        {
          success: true,
          operation: { type: 'add', title: 'Task 1' },
          message: 'Task added: Task 1'
        },
        {
          success: true,
          operation: { type: 'complete', id: '1' },
          taskId: '1',
          message: 'Task 1 marked as completed'
        }
      ];

      const summary = { total: 2, succeeded: 2, failed: 0 };

      const taskList: TaskList = {
        title: 'Batch Test',
        agent: 'test-agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: []
      };

      const statusSummary: StatusSummary = {
        agent: 'test-agent',
        title: 'Batch Test',
        total: 2,
        completed: 1,
        inProgress: 0,
        pending: 1,
        completionRate: 50
      };

      const formatted = formatter.formatBatchResult(results, summary, taskList, statusSummary);

      expect(formatted).toContain('# 📦 배치 작업 결과');
      expect(formatted).toContain('## 📊 요약');
      expect(formatted).toContain('- **총 작업**: 2');
      expect(formatted).toContain('- **✅ 성공**: 2');
      expect(formatted).toContain('- **❌ 실패**: 0');
      expect(formatted).toContain('## ✅ 작업 상세');
      expect(formatted).toContain('✅ **ADD**: Task added: Task 1');
      expect(formatted).toContain('✅ **COMPLETE**: Task 1 marked as completed');
      expect(formatted).toContain('## 📋 현재 작업 현황');
    });

    it('should include failed operations section when there are failures', () => {
      const results: BatchResult[] = [
        {
          success: false,
          operation: { type: 'add', title: 'Failed Task' },
          message: 'Add failed',
          error: 'Database error'
        },
        {
          success: true,
          operation: { type: 'complete', id: '1' },
          taskId: '1',
          message: 'Task completed'
        }
      ];

      const summary = { total: 2, succeeded: 1, failed: 1 };

      const taskList: TaskList = {
        title: 'Batch Test',
        agent: 'test-agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: []
      };

      const statusSummary: StatusSummary = {
        agent: 'test-agent',
        title: 'Batch Test',
        total: 1,
        completed: 1,
        inProgress: 0,
        pending: 0,
        completionRate: 100
      };

      const formatted = formatter.formatBatchResult(results, summary, taskList, statusSummary);

      expect(formatted).toContain('## ❌ 실패한 작업');
      expect(formatted).toContain('➕ 추가: Failed Task');
      expect(formatted).toContain('- **오류**: Add failed');
      expect(formatted).toContain('- **상세**: Database error');
    });
  });

  describe('formatAddResult', () => {
    it('should format add result with status', () => {
      const task: TaskDetail = {
        id: '4',
        title: 'New Task',
        status: 'pending',
        details: [],
        createdAt: '2026-01-30T10:00:00.000Z',
        updatedAt: '2026-01-30T10:00:00.000Z'
      };

      const taskList: TaskList = {
        title: 'Add Test',
        agent: 'test-agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: [task]
      };

      const summary: StatusSummary = {
        agent: 'test-agent',
        title: 'Add Test',
        total: 1,
        completed: 0,
        inProgress: 0,
        pending: 1,
        completionRate: 0
      };

      const formatted = formatter.formatAddResult(task, taskList, summary);

      expect(formatted).toContain('✅ 작업 추가 완료');
      expect(formatted).toContain('**제목**: New Task');
      expect(formatted).toContain('**ID**: 4');
      expect(formatted).toContain('**상태**: ⏳ 대기');
      expect(formatted).toContain('---');
    });
  });

  describe('formatUpdateResult', () => {
    it('should format update result with status', () => {
      const taskList: TaskList = {
        title: 'Update Test',
        agent: 'test-agent',
        createdAt: '2026-01-30',
        sessionId: 'abc',
        tasks: []
      };

      const summary: StatusSummary = {
        agent: 'test-agent',
        title: 'Update Test',
        total: 1,
        completed: 1,
        inProgress: 0,
        pending: 0,
        completionRate: 100
      };

      const formatted = formatter.formatUpdateResult('1', 'completed', taskList, summary);

      expect(formatted).toContain('✅ 작업 상태 업데이트 완료');
      expect(formatted).toContain('**ID**: 1');
      expect(formatted).toContain('✅ completed');
      expect(formatted).toContain('---');
    });
  });
});
