import { TaskStatus, TaskDetail, TaskList, TaskStatusSummary } from '../../src/types';

describe('Types', () => {
  describe('TaskStatus', () => {
    it('should accept valid status values', () => {
      const statuses: TaskStatus[] = ['pending', 'in_progress', 'completed'];
      
      statuses.forEach(status => {
        expect(status).toBeDefined();
      });
    });

    it('should work in task objects', () => {
      const task: TaskDetail = {
        id: '1',
        title: 'Test Task',
        status: 'in_progress',
        details: [],
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString()
      };

      expect(task.status).toBe('in_progress');
    });
  });

  describe('TaskDetail', () => {
    it('should create valid task structure', () => {
      const task: TaskDetail = {
        id: '1.2',
        title: 'Test Task',
        status: 'pending',
        details: ['Detail 1', 'Detail 2'],
        subtasks: [
          {
            id: '1.2.1',
            title: 'Subtask',
            status: 'completed',
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ],
        createdAt: '2026-01-30T10:00:00.000Z',
        updatedAt: '2026-01-30T10:00:00.000Z'
      };

      expect(task.id).toBe('1.2');
      expect(task.details).toHaveLength(2);
      expect(task.subtasks).toHaveLength(1);
    });

    it('should allow optional subtasks', () => {
      const task: TaskDetail = {
        id: '1',
        title: 'Simple Task',
        status: 'pending',
        details: [],
        createdAt: '2026-01-30T10:00:00.000Z',
        updatedAt: '2026-01-30T10:00:00.000Z'
      };

      expect(task.subtasks).toBeUndefined();
    });
  });

  describe('TaskList', () => {
    it('should create valid task list structure', () => {
      const taskList: TaskList = {
        title: 'Project Tasks',
        agent: 'test-agent',
        createdAt: '2026-01-30 10:00:00',
        sessionId: 'abc-123',
        tasks: [
          {
            id: '1',
            title: 'Task 1',
            status: 'pending',
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ],
        currentPhase: 'Planning',
        memo: 'Important notes'
      };

      expect(taskList.title).toBe('Project Tasks');
      expect(taskList.tasks).toHaveLength(1);
      expect(taskList.currentPhase).toBe('Planning');
      expect(taskList.memo).toBe('Important notes');
    });

    it('should allow optional fields', () => {
      const taskList: TaskList = {
        title: 'Simple List',
        agent: 'agent',
        createdAt: '2026-01-30',
        sessionId: 'xyz',
        tasks: []
      };

      expect(taskList.currentPhase).toBeUndefined();
      expect(taskList.memo).toBeUndefined();
    });
  });

  describe('TaskStatusSummary', () => {
    it('should create valid summary structure', () => {
      const summary: TaskStatusSummary = {
        agent: 'test-agent',
        title: 'Project',
        status: 'in_progress',
        completionRate: 75,
        completedCount: 3,
        totalCount: 4,
        currentPhase: 'Implementation'
      };

      expect(summary.completionRate).toBe(75);
      expect(summary.completedCount).toBe(3);
      expect(summary.totalCount).toBe(4);
    });

    it('should allow optional currentPhase', () => {
      const summary: TaskStatusSummary = {
        agent: 'agent',
        title: 'Project',
        status: 'pending',
        completionRate: 0,
        completedCount: 0,
        totalCount: 5
      };

      expect(summary.currentPhase).toBeUndefined();
    });
  });
});
