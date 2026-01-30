import { updateCommand, UpdateArgs } from '../../src/commands/update';
import { Storage } from '../../src/lib/storage';
import { Parser } from '../../src/lib/parser';
import { TaskStatus } from '../../src/types';

// Mocks
jest.mock('../../src/lib/storage');
jest.mock('../../src/lib/parser');

describe('updateCommand', () => {
  let mockStorage: jest.Mocked<Storage>;
  let mockParser: jest.Mocked<Parser>;

  beforeEach(() => {
    jest.clearAllMocks();

    mockStorage = new Storage() as jest.Mocked<Storage>;
    mockParser = new Parser() as jest.Mocked<Parser>;

    (Storage as jest.MockedClass<typeof Storage>).mockImplementation(() => mockStorage);
    (Parser as jest.MockedClass<typeof Parser>).mockImplementation(() => mockParser);
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
        { id: '1', title: 'Task One', status: 'pending' }
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
        { id: '2', title: 'Task Two', status: 'pending' }
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
  });
});
