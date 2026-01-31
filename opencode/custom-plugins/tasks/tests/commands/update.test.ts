import { updateCommand, UpdateArgs } from '../../src/commands/update';
import { Storage } from '../../src/lib/storage';
import { Parser } from '../../src/lib/parser';
import { Formatter } from '../../src/lib/formatter';
import { TaskStatus } from '../../src/types';

// Mocks
jest.mock('../../src/lib/storage');
jest.mock('../../src/lib/parser');
jest.mock('../../src/lib/formatter');

describe('updateCommand', () => {
  let mockStorage: jest.Mocked<Storage>;
  let mockParser: jest.Mocked<Parser>;
  let mockFormatter: jest.Mocked<Formatter>;

  beforeEach(() => {
    jest.clearAllMocks();

    mockStorage = new Storage() as jest.Mocked<Storage>;
    mockParser = new Parser() as jest.Mocked<Parser>;
    mockFormatter = new Formatter() as jest.Mocked<Formatter>;

    (Storage as jest.MockedClass<typeof Storage>).mockImplementation(() => mockStorage);
    (Parser as jest.MockedClass<typeof Parser>).mockImplementation(() => mockParser);
    (Formatter as jest.MockedClass<typeof Formatter>).mockImplementation(() => mockFormatter);

    // Mock formatter methods
    mockFormatter.calculateStatusSummary.mockReturnValue({
      agent: 'test-agent',
      title: 'Test Project',
      total: 1,
      completed: 1,
      inProgress: 0,
      pending: 0,
      completionRate: 100
    });
    mockFormatter.formatUpdateResult.mockReturnValue('# ✅ 작업 상태 업데이트 완료\n\n**ID**: 1\n**새 상태**: ✅ completed\n\n---\n\n## 📋 현황');
  });

  it('should update task status successfully', async () => {
    const args: UpdateArgs = {
      sessionId: 'test-session',
      id: '1',
      status: 'completed' as TaskStatus
    };

    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: [
        { id: '1', title: 'Task One', status: 'completed' }
      ]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockParser.updateTaskStatus.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated content');

    const result = await updateCommand(args);

    expect(mockParser.updateTaskStatus).toHaveBeenCalledWith(taskList, '1', 'completed');
    expect(mockStorage.saveTaskList).toHaveBeenCalledWith('test-session', 'project', 'updated content');
    expect(result.success).toBe(true);
    expect(result.message).toContain('updated');
    expect(result.formattedOutput).toBeDefined();
    expect(result.currentStatus).toBeDefined();
    expect(result.statusSummary).toBeDefined();

    // Verify ToolResponse structure
    expect(result.response).toBeDefined();
    expect(result.response.title).toBeDefined();
    expect(result.response.output).toBeDefined();
    expect(result.response.metadata).toBeDefined();
    expect(result.response.metadata.operation).toBe('update');
    expect(result.response.metadata.taskId).toBe('1');
    expect(result.response.metadata.status).toBe('completed');
    expect(result.response.metadata.taskList).toBeDefined();
    expect(result.response.metadata.summary).toBeDefined();
  });

  it('should update task to in_progress status', async () => {
    const args: UpdateArgs = {
      sessionId: 'test-session',
      id: '2',
      status: 'in_progress' as TaskStatus
    };

    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: [
        { id: '2', title: 'Task Two', status: 'in_progress' }
      ]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockParser.updateTaskStatus.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated content');

    const result = await updateCommand(args);

    expect(result.success).toBe(true);
    expect(result.status).toBe('in_progress');
    expect(result.formattedOutput).toBeDefined();
  });

  it('should handle task not found', async () => {
    const args: UpdateArgs = {
      sessionId: 'test-session',
      id: '999',
      status: 'completed' as TaskStatus
    };

    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: []
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockParser.updateTaskStatus.mockReturnValue(false);

    const result = await updateCommand(args);

    expect(result.success).toBe(false);
    expect(result.message).toContain('not found');
    expect(result.formattedOutput).toBeDefined();
    expect(result.currentStatus).toBeDefined();
    expect(result.statusSummary).toBeDefined();
  });

  it('should handle no task lists found', async () => {
    const args: UpdateArgs = {
      sessionId: 'non-existent-session',
      id: '1',
      status: 'completed' as TaskStatus
    };

    mockStorage.listTaskFiles.mockResolvedValue([]);

    const result = await updateCommand(args);

    expect(result.success).toBe(false);
    expect(result.message).toContain('No task lists found');
    expect(result.formattedOutput).toBeDefined();
    expect(result.currentStatus).toBeDefined();
    expect(result.statusSummary).toBeDefined();
  });
});
