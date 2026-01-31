import { removeCommand, RemoveArgs } from '../../src/commands/remove';
import { Storage } from '../../src/lib/storage';
import { Parser } from '../../src/lib/parser';
import { Formatter } from '../../src/lib/formatter';

// Mocks
jest.mock('../../src/lib/storage');
jest.mock('../../src/lib/parser');
jest.mock('../../src/lib/formatter');

describe('removeCommand', () => {
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
      total: 0,
      completed: 0,
      inProgress: 0,
      pending: 0,
      completionRate: 0
    });
    mockFormatter.formatTaskListWithStatus.mockReturnValue('## 📋 현황');
  });

  it('should remove task with force flag', async () => {
    const args: RemoveArgs = {
      sessionId: 'test-session',
      id: '1',
      force: true
    };

    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: [
        { id: '1', title: 'Task One', status: 'pending', details: [] }
      ]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockParser.removeTask.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated content');

    const result = await removeCommand(args);

    expect(mockParser.removeTask).toHaveBeenCalledWith(taskList, '1');
    expect(result.success).toBe(true);
    expect(result.message).toContain('removed');
    expect(result.formattedOutput).toBeDefined();
    expect(result.currentStatus).toBeDefined();
    expect(result.statusSummary).toBeDefined();
  });

  it('should handle task not found', async () => {
    const args: RemoveArgs = {
      sessionId: 'test-session',
      id: '999',
      force: true
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

    const result = await removeCommand(args);

    expect(result.success).toBe(false);
    expect(result.message).toContain('not found');
    expect(result.formattedOutput).toBeDefined();
    expect(result.currentStatus).toBeDefined();
    expect(result.statusSummary).toBeDefined();
  });

  it('should handle no task lists found', async () => {
    const args: RemoveArgs = {
      sessionId: 'non-existent-session',
      id: '1',
      force: true
    };

    mockStorage.listTaskFiles.mockResolvedValue([]);

    const result = await removeCommand(args);

    expect(result.success).toBe(false);
    expect(result.message).toContain('No task lists found');
    expect(result.formattedOutput).toBeDefined();
    expect(result.currentStatus).toBeDefined();
    expect(result.statusSummary).toBeDefined();
  });

  it('should remove nested subtask', async () => {
    const args: RemoveArgs = {
      sessionId: 'test-session',
      id: '1.1',
      force: true
    };

    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: [
        {
          id: '1',
          title: 'Parent Task',
          status: 'pending',
          subtasks: [
            { id: '1.1', title: 'Subtask', status: 'pending' }
          ]
        }
      ]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockParser.removeTask.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated content');

    const result = await removeCommand(args);

    expect(result.success).toBe(true);
    expect(result.taskId).toBe('1.1');
    expect(result.formattedOutput).toBeDefined();
  });

  it('should return ToolResponse with title, output, and metadata', async () => {
    const args: RemoveArgs = {
      sessionId: 'test-session',
      id: '1',
      force: true
    };

    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: [
        { id: '1', title: 'Task One', status: 'pending', details: [] }
      ]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockParser.removeTask.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated content');

    const result = await removeCommand(args);

    // Verify ToolResponse structure
    expect(result.response).toBeDefined();
    expect(result.response.title).toBeDefined();
    expect(result.response.output).toBeDefined();
    expect(result.response.metadata).toBeDefined();
    expect(result.response.metadata.operation).toBe('remove');
    expect(result.response.metadata.taskId).toBe('1');
    expect(result.response.metadata.taskList).toBeDefined();
    expect(result.response.metadata.summary).toBeDefined();
  });
});
