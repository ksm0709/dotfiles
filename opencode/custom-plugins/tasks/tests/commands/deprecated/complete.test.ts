import { completeCommand, CompleteArgs } from '../../src/commands/complete';
import { updateCommand } from '../../src/commands/update';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';
import { TaskList, StatusSummary, ToolResponse } from '../../src/types';

// Mocks
jest.mock('../../src/commands/update');

describe('completeCommand', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let mockUpdateCommand: jest.MockedFunction<typeof updateCommand>;

  const mockTaskList: TaskList = {
    title: 'Test Project',
    agent: 'test-agent',
    sessionId: 'test-session',
    createdAt: '2026-01-30',
    tasks: [{ id: '1', title: 'Test Task', status: 'completed', details: [], createdAt: '2026-01-30', updatedAt: '2026-01-30' }]
  };

  const mockStatusSummary: StatusSummary = {
    agent: 'test-agent',
    title: 'Test Project',
    total: 1,
    completed: 1,
    inProgress: 0,
    pending: 0,
    completionRate: 100
  };

  beforeEach(async () => {
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-complete-test-'));
    originalHome = process.env.HOME;
    process.env.HOME = tempDir;

    jest.clearAllMocks();

    mockUpdateCommand = updateCommand as jest.MockedFunction<typeof updateCommand>;
  });

  afterEach(async () => {
    if (originalHome !== undefined) {
      process.env.HOME = originalHome;
    }
    try {
      await fs.rm(tempDir, { recursive: true, force: true });
    } catch (error) {
      // Ignore
    }
  });

  it('should mark task as completed', async () => {
    const args: CompleteArgs = {
      sessionId: 'test-session',
      id: '1'
    };

    const mockResponse: ToolResponse = {
      title: 'Updated: Task 1 → completed',
      output: '# ✅ 작업 상태 업데이트 완료\n\n**ID**: 1\n**새 상태**: ✅ completed',
      metadata: {
        operation: 'update',
        taskId: '1',
        status: 'completed',
        taskList: mockTaskList,
        tasks: mockTaskList.tasks,
        summary: mockStatusSummary,
        message: 'Task 1 status updated to: completed'
      }
    };

    mockUpdateCommand.mockResolvedValue({
      success: true,
      taskId: '1',
      status: 'completed',
      message: 'Task 1 status updated to: completed',
      currentStatus: mockTaskList,
      statusSummary: mockStatusSummary,
      formattedOutput: '# ✅ 작업 상태 업데이트 완료\n\n**ID**: 1\n**새 상태**: ✅ completed',
      response: mockResponse
    });

    const result = await completeCommand(args);

    expect(mockUpdateCommand).toHaveBeenCalledWith({
      sessionId: 'test-session',
      id: '1',
      status: 'completed'
    });
    expect(result.success).toBe(true);
    expect(result.message).toContain('completed');
    expect(result.formattedOutput).toBeDefined();
    expect(result.currentStatus).toBeDefined();
    expect(result.statusSummary).toBeDefined();

    // Verify ToolResponse structure
    expect(result.response).toBeDefined();
    expect(result.response.title).toContain('Completed');
    expect(result.response.output).toBeDefined();
    expect(result.response.metadata).toBeDefined();
    expect(result.response.metadata.operation).toBe('complete');
    expect(result.response.metadata.taskId).toBe('1');
    expect(result.response.metadata.status).toBe('completed');
  });

  it('should handle error from updateCommand', async () => {
    const args: CompleteArgs = {
      sessionId: 'test-session',
      id: '1'
    };

    mockUpdateCommand.mockRejectedValue(new Error('Update failed'));

    await expect(completeCommand(args)).rejects.toThrow('Update failed');
  });

  it('should handle non-existent task with status info', async () => {
    const args: CompleteArgs = {
      sessionId: 'test-session',
      id: '999'
    };

    mockUpdateCommand.mockResolvedValue({
      success: false,
      taskId: '999',
      status: 'completed',
      message: 'Task 999 not found',
      currentStatus: mockTaskList,
      statusSummary: mockStatusSummary,
      formattedOutput: '❌ Task 999 not found',
      response: {
        title: 'Task 999 not found',
        output: '❌ Task 999 not found',
        metadata: {
          operation: 'update',
          taskId: '999',
          status: 'completed',
          taskList: mockTaskList,
          tasks: mockTaskList.tasks,
          summary: mockStatusSummary,
          message: 'Task 999 not found'
        }
      }
    });

    // 실패 시에도 결과를 반환하도록 변경되었으므로 예외가 발생하지 않음
    const result = await completeCommand(args);

    expect(result.success).toBe(false);
    expect(result.message).toContain('not found');
    expect(result.formattedOutput).toBeDefined();
    expect(result.currentStatus).toBeDefined();
    expect(result.statusSummary).toBeDefined();
  });

  it('should return ToolResponse with title, output, and metadata', async () => {
    const args: CompleteArgs = {
      sessionId: 'test-session',
      id: '1'
    };

    const mockResponse: ToolResponse = {
      title: 'Completed: Task 1',
      output: '# ✅ 작업 완료 처리\n\n**ID**: 1\n**상태**: ✅ completed',
      metadata: {
        operation: 'complete',
        taskId: '1',
        status: 'completed',
        taskList: mockTaskList,
        tasks: mockTaskList.tasks,
        summary: mockStatusSummary,
        message: 'Task 1 marked as completed'
      }
    };

    mockUpdateCommand.mockResolvedValue({
      success: true,
      taskId: '1',
      status: 'completed',
      message: 'Task 1 marked as completed',
      currentStatus: mockTaskList,
      statusSummary: mockStatusSummary,
      formattedOutput: '# ✅ 작업 완료 처리\n\n**ID**: 1\n**상태**: ✅ completed',
      response: mockResponse
    });

    const result = await completeCommand(args);

    // Verify ToolResponse structure
    expect(result.response).toBeDefined();
    expect(result.response.title).toContain('Completed');
    expect(result.response.output).toBeDefined();
    expect(result.response.metadata).toBeDefined();
    expect(result.response.metadata.operation).toBe('complete');
    expect(result.response.metadata.taskId).toBe('1');
    expect(result.response.metadata.status).toBe('completed');
    expect(result.response.metadata.summary).toBeDefined();
    expect(result.response.metadata.taskList).toBeDefined();
  });
});
