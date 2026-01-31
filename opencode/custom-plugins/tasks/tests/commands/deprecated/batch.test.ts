import { batchCommand, BatchArgs } from '../../src/commands/batch';
import { addTaskCommand } from '../../src/commands/add-task';
import { updateCommand } from '../../src/commands/update';
import { completeCommand } from '../../src/commands/complete';
import { removeCommand } from '../../src/commands/remove';
import { Storage } from '../../src/lib/storage';
import { Parser } from '../../src/lib/parser';
import { Formatter } from '../../src/lib/formatter';
import { BatchOperation, TaskList, TaskDetail, ToolResponse } from '../../src/types';

// Mocks
jest.mock('../../src/commands/add-task');
jest.mock('../../src/commands/update');
jest.mock('../../src/commands/complete');
jest.mock('../../src/commands/remove');
jest.mock('../../src/lib/storage');
jest.mock('../../src/lib/parser');
jest.mock('../../src/lib/formatter');

describe('batchCommand', () => {
  let mockStorage: jest.Mocked<Storage>;
  let mockParser: jest.Mocked<Parser>;
  let mockFormatter: jest.Mocked<Formatter>;
  let mockAddTaskCommand: jest.MockedFunction<typeof addTaskCommand>;
  let mockUpdateCommand: jest.MockedFunction<typeof updateCommand>;
  let mockCompleteCommand: jest.MockedFunction<typeof completeCommand>;
  let mockRemoveCommand: jest.MockedFunction<typeof removeCommand>;

  const mockTaskList: TaskList = {
    title: 'Test Project',
    agent: 'test-agent',
    sessionId: 'test-session',
    createdAt: '2026-01-30',
    tasks: [
      { id: '1', title: 'Task 1', status: 'pending', details: [], createdAt: '2026-01-30', updatedAt: '2026-01-30' },
      { id: '2', title: 'Task 2', status: 'in_progress', details: [], createdAt: '2026-01-30', updatedAt: '2026-01-30' },
      { id: '3', title: 'Task 3', status: 'completed', details: [], createdAt: '2026-01-30', updatedAt: '2026-01-30' }
    ]
  };

  beforeEach(() => {
    jest.clearAllMocks();

    mockStorage = new Storage() as jest.Mocked<Storage>;
    mockParser = new Parser() as jest.Mocked<Parser>;
    mockFormatter = new Formatter() as jest.Mocked<Formatter>;
    mockAddTaskCommand = addTaskCommand as jest.MockedFunction<typeof addTaskCommand>;
    mockUpdateCommand = updateCommand as jest.MockedFunction<typeof updateCommand>;
    mockCompleteCommand = completeCommand as jest.MockedFunction<typeof completeCommand>;
    mockRemoveCommand = removeCommand as jest.MockedFunction<typeof removeCommand>;

    (Storage as jest.MockedClass<typeof Storage>).mockImplementation(() => mockStorage);
    (Parser as jest.MockedClass<typeof Parser>).mockImplementation(() => mockParser);
    (Formatter as jest.MockedClass<typeof Formatter>).mockImplementation(() => mockFormatter);

    // Mock storage methods
    mockStorage.listTaskFiles.mockResolvedValue(['test-project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');

    // Mock parser methods
    mockParser.parseTaskList.mockReturnValue(mockTaskList);

    // Mock formatter methods
    mockFormatter.calculateStatusSummary.mockReturnValue({
      agent: 'test-agent',
      title: 'Test Project',
      total: 3,
      completed: 1,
      inProgress: 1,
      pending: 1,
      completionRate: 33
    });
    mockFormatter.formatBatchResult.mockReturnValue('# 📦 배치 작업 결과\n\n## 📊 요약\n\n- 총 작업: 5\n- ✅ 성공: 5\n- ❌ 실패: 0');

    // Mock command methods
    mockAddTaskCommand.mockResolvedValue({
      success: true,
      title: 'New Task',
      details: [],
      message: 'Task added: New Task',
      currentStatus: mockTaskList,
      statusSummary: {
        agent: 'test-agent',
        title: 'Test Project',
        total: 4,
        completed: 1,
        inProgress: 1,
        pending: 2,
        completionRate: 25
      },
      formattedOutput: '✅ Task added',
      response: {
        title: 'New Task',
        output: '✅ Task added',
        metadata: {
          operation: 'add',
          taskId: '4',
          taskList: mockTaskList,
          tasks: mockTaskList.tasks,
          summary: {
            agent: 'test-agent',
            title: 'Test Project',
            total: 4,
            completed: 1,
            inProgress: 1,
            pending: 2,
            completionRate: 25
          },
          message: 'Task added: New Task'
        }
      }
    });

    mockUpdateCommand.mockResolvedValue({
      success: true,
      taskId: '1',
      status: 'completed',
      message: 'Task 1 status updated to: completed',
      currentStatus: mockTaskList,
      statusSummary: {
        agent: 'test-agent',
        title: 'Test Project',
        total: 3,
        completed: 2,
        inProgress: 0,
        pending: 1,
        completionRate: 67
      },
      formattedOutput: '✅ Task updated',
      response: {
        title: 'Updated: Task 1 → completed',
        output: '✅ Task updated',
        metadata: {
          operation: 'update',
          taskId: '1',
          status: 'completed',
          taskList: mockTaskList,
          tasks: mockTaskList.tasks,
          summary: {
            agent: 'test-agent',
            title: 'Test Project',
            total: 3,
            completed: 2,
            inProgress: 0,
            pending: 1,
            completionRate: 67
          },
          message: 'Task 1 status updated to: completed'
        }
      }
    });

    mockCompleteCommand.mockResolvedValue({
      success: true,
      taskId: '2',
      message: 'Task 2 marked as completed',
      currentStatus: mockTaskList,
      statusSummary: {
        agent: 'test-agent',
        title: 'Test Project',
        total: 3,
        completed: 2,
        inProgress: 0,
        pending: 1,
        completionRate: 67
      },
      formattedOutput: '✅ Task completed',
      response: {
        title: 'Completed: Task 2',
        output: '✅ Task completed',
        metadata: {
          operation: 'complete',
          taskId: '2',
          status: 'completed',
          taskList: mockTaskList,
          tasks: mockTaskList.tasks,
          summary: {
            agent: 'test-agent',
            title: 'Test Project',
            total: 3,
            completed: 2,
            inProgress: 0,
            pending: 1,
            completionRate: 67
          },
          message: 'Task 2 marked as completed'
        }
      }
    });

    mockRemoveCommand.mockResolvedValue({
      success: true,
      taskId: '3',
      taskTitle: 'Task 3',
      message: 'Task 3 "Task 3" removed',
      currentStatus: {
        ...mockTaskList,
        tasks: mockTaskList.tasks.filter(t => t.id !== '3')
      },
      statusSummary: {
        agent: 'test-agent',
        title: 'Test Project',
        total: 2,
        completed: 0,
        inProgress: 1,
        pending: 1,
        completionRate: 0
      },
      formattedOutput: '✅ Task removed',
      response: {
        title: 'Removed: Task 3',
        output: '✅ Task removed',
        metadata: {
          operation: 'remove',
          taskId: '3',
          taskList: {
            ...mockTaskList,
            tasks: mockTaskList.tasks.filter(t => t.id !== '3')
          },
          tasks: mockTaskList.tasks.filter(t => t.id !== '3'),
          summary: {
            agent: 'test-agent',
            title: 'Test Project',
            total: 2,
            completed: 0,
            inProgress: 1,
            pending: 1,
            completionRate: 0
          },
          message: 'Task 3 "Task 3" removed'
        }
      }
    });
  });

  describe('validation', () => {
    it('should throw error when operations is not an array', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: 'not-an-array' as any
      };

      await expect(batchCommand(args)).rejects.toThrow('Operations must be an array');
    });

    it('should throw error when no operations provided', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: []
      };

      await expect(batchCommand(args)).rejects.toThrow('No operations provided');
    });

    it('should throw error when operations exceed maximum limit', async () => {
      const operations: BatchOperation[] = Array(51).fill({
        type: 'complete',
        id: '1'
      });

      const args: BatchArgs = {
        sessionId: 'test-session',
        operations
      };

      await expect(batchCommand(args)).rejects.toThrow('Too many operations. Maximum is 50');
    });

    it('should throw error when type is missing', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [{ title: 'Task' }] as any
      };

      await expect(batchCommand(args)).rejects.toThrow('Operation 1: type is required');
    });

    it('should throw error when add operation missing title', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [{ type: 'add' }]
      };

      await expect(batchCommand(args)).rejects.toThrow('Operation 1: title is required for add operation');
    });

    it('should throw error when update operation missing id', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [{ type: 'update', status: 'completed' }]
      };

      await expect(batchCommand(args)).rejects.toThrow('Operation 1: id is required for update operation');
    });

    it('should throw error when complete operation missing id', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [{ type: 'complete' }]
      };

      await expect(batchCommand(args)).rejects.toThrow('Operation 1: id is required for complete operation');
    });

    it('should throw error when remove operation missing id', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [{ type: 'remove' }]
      };

      await expect(batchCommand(args)).rejects.toThrow('Operation 1: id is required for remove operation');
    });

    it('should throw error when update operation missing status', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [{ type: 'update', id: '1' }]
      };

      await expect(batchCommand(args)).rejects.toThrow('Operation 1: status is required for update operation');
    });
  });

  describe('batch operations', () => {
    it('should execute single add operation', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [{ type: 'add', title: 'New Task' }]
      };

      const result = await batchCommand(args);

      expect(mockAddTaskCommand).toHaveBeenCalledWith({
        sessionId: 'test-session',
        title: 'New Task',
        parent: undefined
      });
      expect(result.success).toBe(true);
      expect(result.results).toHaveLength(1);
      expect(result.results[0].success).toBe(true);
    });

    it('should execute single update operation', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [{ type: 'update', id: '1', status: 'completed' }]
      };

      const result = await batchCommand(args);

      expect(mockUpdateCommand).toHaveBeenCalledWith({
        sessionId: 'test-session',
        id: '1',
        status: 'completed'
      });
      expect(result.success).toBe(true);
      expect(result.results).toHaveLength(1);
    });

    it('should execute single complete operation', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [{ type: 'complete', id: '2' }]
      };

      const result = await batchCommand(args);

      expect(mockCompleteCommand).toHaveBeenCalledWith({
        sessionId: 'test-session',
        id: '2'
      });
      expect(result.success).toBe(true);
      expect(result.results).toHaveLength(1);
    });

    it('should execute single remove operation', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [{ type: 'remove', id: '3' }]
      };

      const result = await batchCommand(args);

      expect(mockRemoveCommand).toHaveBeenCalledWith({
        sessionId: 'test-session',
        id: '3',
        force: true
      });
      expect(result.success).toBe(true);
      expect(result.results).toHaveLength(1);
    });

    it('should execute 5+ operations in batch', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [
          { type: 'add', title: 'Task 1' },
          { type: 'add', title: 'Task 2' },
          { type: 'update', id: '1', status: 'in_progress' },
          { type: 'complete', id: '2' },
          { type: 'remove', id: '3' }
        ]
      };

      const result = await batchCommand(args);

      expect(result.results).toHaveLength(5);
      expect(result.summary.total).toBe(5);
      expect(result.summary.succeeded).toBe(5);
      expect(result.summary.failed).toBe(0);
      expect(result.formattedOutput).toBeDefined();
    });
  });

  describe('partial failures', () => {
    beforeEach(() => {
      // 실패하는 작업 설정
      mockAddTaskCommand.mockRejectedValueOnce(new Error('Add failed'));
      mockUpdateCommand.mockRejectedValueOnce(new Error('Update failed'));
    });

    it('should continue processing when individual operations fail', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [
          { type: 'add', title: 'Will Fail' },
          { type: 'add', title: 'Will Succeed' },
          { type: 'update', id: '1', status: 'completed' },
          { type: 'complete', id: '2' }
        ]
      };

      const result = await batchCommand(args);

      expect(result.results).toHaveLength(4);
      expect(result.success).toBe(false);  // 부분적 실패로 전체 실패
      expect(result.summary.failed).toBeGreaterThan(0);
      expect(result.summary.succeeded).toBeGreaterThan(0);
    });

    it('should include error details in failed results', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [
          { type: 'add', title: 'Will Fail' }
        ]
      };

      const result = await batchCommand(args);

      expect(result.results[0].success).toBe(false);
      expect(result.results[0].error).toBeDefined();
      expect(result.results[0].message).toContain('Add failed');
    });
  });

  describe('results', () => {
    beforeEach(() => {
      // Reset all mocks to remove any pending mockRejectedValueOnce implementations
      mockAddTaskCommand.mockReset();
      mockUpdateCommand.mockReset();
      mockCompleteCommand.mockReset();
      mockRemoveCommand.mockReset();
      
      // Reset mocks to ensure they resolve successfully for these tests
      mockAddTaskCommand.mockResolvedValue({
        success: true,
        title: 'New Task',
        details: [],
        message: 'Task added: New Task',
        currentStatus: mockTaskList,
        statusSummary: {
          agent: 'test-agent',
          title: 'Test Project',
          total: 4,
          completed: 1,
          inProgress: 1,
          pending: 2,
          completionRate: 25
        },
        formattedOutput: '✅ Task added',
        response: {
          title: 'New Task',
          output: '✅ Task added',
          metadata: {
            operation: 'add',
            taskId: '4',
            taskList: mockTaskList,
            tasks: mockTaskList.tasks,
            summary: {
              agent: 'test-agent',
              title: 'Test Project',
              total: 4,
              completed: 1,
              inProgress: 1,
              pending: 2,
              completionRate: 25
            },
            message: 'Task added: New Task'
          }
        }
      });

      mockUpdateCommand.mockResolvedValue({
        success: true,
        taskId: '1',
        status: 'completed',
        message: 'Task 1 status updated to: completed',
        currentStatus: mockTaskList,
        statusSummary: {
          agent: 'test-agent',
          title: 'Test Project',
          total: 3,
          completed: 2,
          inProgress: 0,
          pending: 1,
          completionRate: 67
        },
        formattedOutput: '✅ Task updated',
        response: {
          title: 'Updated: Task 1 → completed',
          output: '✅ Task updated',
          metadata: {
            operation: 'update',
            taskId: '1',
            status: 'completed',
            taskList: mockTaskList,
            tasks: mockTaskList.tasks,
            summary: {
              agent: 'test-agent',
              title: 'Test Project',
              total: 3,
              completed: 2,
              inProgress: 0,
              pending: 1,
              completionRate: 67
            },
            message: 'Task 1 status updated to: completed'
          }
        }
      });
    });

    it('should include current status in result', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [{ type: 'complete', id: '1' }]
      };

      const result = await batchCommand(args);

      expect(result.currentStatus).toBeDefined();
      expect(result.currentStatus.title).toBe('Test Project');
    });

    it('should include formatted output in result', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [{ type: 'add', title: 'New Task' }]
      };

      const result = await batchCommand(args);

      expect(result.formattedOutput).toBeDefined();
      expect(result.formattedOutput).toContain('배치 작업 결과');
    });

    it('should calculate correct summary statistics', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [
          { type: 'add', title: 'Task 1' },
          { type: 'add', title: 'Task 2' },
          { type: 'add', title: 'Task 3' }
        ]
      };

      const result = await batchCommand(args);

      expect(result.summary).toEqual({
        total: 3,
        succeeded: 3,
        failed: 0
      });
    });

    it('should return ToolResponse with title, output, and metadata', async () => {
      const args: BatchArgs = {
        sessionId: 'test-session',
        operations: [
          { type: 'add', title: 'Task 1' },
          { type: 'update', id: '1', status: 'in_progress' }
        ]
      };

      const result = await batchCommand(args);



      // Verify ToolResponse structure
      expect(result.response).toBeDefined();
      expect(result.response.title).toBeDefined();
      expect(result.response.output).toBeDefined();
      expect(result.response.metadata).toBeDefined();
      expect(result.response.metadata.operation).toBe('batch');
      expect(result.response.metadata.batchSummary).toBeDefined();
      expect(result.response.metadata.batchSummary?.total).toBe(2);
      expect(result.response.metadata.batchSummary?.succeeded).toBe(2);
      expect(result.response.metadata.batchSummary?.failed).toBe(0);
    });
  });
});
