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
    it('should create valid task structure with flat ID', () => {
      const task: TaskDetail = {
        id: '1',
        title: 'Test Task',
        status: 'pending',
        details: ['Detail 1', 'Detail 2'],
        createdAt: '2026-01-30T10:00:00.000Z',
        updatedAt: '2026-01-30T10:00:00.000Z'
      };

      expect(task.id).toBe('1');
      expect(task.details).toHaveLength(2);
    });

    it('should create task with hierarchical ID (1.1, 1.2 format)', () => {
      // In flat structure, hierarchical IDs are just string IDs
      const parentTask: TaskDetail = {
        id: '1',
        title: 'Parent Task',
        status: 'in_progress',
        details: [],
        createdAt: '2026-01-30T10:00:00.000Z',
        updatedAt: '2026-01-30T10:00:00.000Z'
      };

      const childTask: TaskDetail = {
        id: '1.1',
        title: 'Child Task',
        status: 'completed',
        details: [],
        createdAt: '2026-01-30T10:00:00.000Z',
        updatedAt: '2026-01-30T10:00:00.000Z'
      };

      expect(parentTask.id).toBe('1');
      expect(childTask.id).toBe('1.1');
      // No parent-child relationship in flat structure
    });
  });

  describe('TaskList', () => {
    it('should create valid task list structure with flat tasks', () => {
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
          },
          {
            id: '1.1',
            title: 'Subtask 1.1',
            status: 'completed',
            details: [],
            createdAt: '2026-01-30T10:00:00.000Z',
            updatedAt: '2026-01-30T10:00:00.000Z'
          }
        ],
        currentPhase: 'Planning',
        memo: 'Important notes'
      };

      expect(taskList.title).toBe('Project Tasks');
      expect(taskList.tasks).toHaveLength(2);
      expect(taskList.tasks[0].id).toBe('1');
      expect(taskList.tasks[1].id).toBe('1.1');
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
