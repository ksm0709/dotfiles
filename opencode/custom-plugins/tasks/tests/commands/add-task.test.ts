import { addTaskCommand, AddTaskArgs } from '../../src/commands/add-task';
import { Storage } from '../../src/lib/storage';
import { Parser } from '../../src/lib/parser';

// Mocks
jest.mock('../../src/lib/storage');
jest.mock('../../src/lib/parser');

describe('addTaskCommand', () => {
  let mockStorage: jest.Mocked<Storage>;
  let mockParser: jest.Mocked<Parser>;

  beforeEach(() => {
    jest.clearAllMocks();

    mockStorage = new Storage() as jest.Mocked<Storage>;
    mockParser = new Parser() as jest.Mocked<Parser>;

    (Storage as jest.MockedClass<typeof Storage>).mockImplementation(() => mockStorage);
    (Parser as jest.MockedClass<typeof Parser>).mockImplementation(() => mockParser);
  });

  it('should add a new task', async () => {
    const args: AddTaskArgs = {
      sessionId: 'test-session',
      title: 'New Task'
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
    mockParser.addTask.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated content');

    const result = await addTaskCommand(args);

    expect(mockStorage.saveTaskList).toHaveBeenCalled();
    expect(result.success).toBe(true);
    expect(result.title).toBe('New Task');
  });

  it('should add a task with details', async () => {
    const args: AddTaskArgs = {
      sessionId: 'test-session',
      title: 'Task with details',
      details: 'Detail 1, Detail 2'
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
    mockParser.addTask.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated content');

    const result = await addTaskCommand(args);

    expect(result.success).toBe(true);
    expect(result.details).toEqual(['Detail 1', 'Detail 2']);
  });

  it('should add a subtask with parent', async () => {
    const args: AddTaskArgs = {
      sessionId: 'test-session',
      title: 'Subtask',
      parent: '1'
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
    mockParser.addTask.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated content');

    const result = await addTaskCommand(args);

    expect(result.success).toBe(true);
    expect(result.parent).toBe('1');
  });

  it('should handle no task lists found', async () => {
    const args: AddTaskArgs = {
      sessionId: 'non-existent-session',
      title: 'New Task'
    };

    mockStorage.listTaskFiles.mockResolvedValue([]);

    const result = await addTaskCommand(args);

    expect(result.success).toBe(false);
    expect(result.message).toContain('No task lists found');
  });

  it('should handle read error', async () => {
    const args: AddTaskArgs = {
      sessionId: 'test-session',
      title: 'Task'
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue(null);

    await expect(addTaskCommand(args)).rejects.toThrow('Failed to read task list');
  });

  it('should handle parent not found', async () => {
    const args: AddTaskArgs = {
      sessionId: 'test-session',
      title: 'Orphan Task',
      parent: '999'
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
    mockParser.addTask.mockReturnValue(false);

    await expect(addTaskCommand(args)).rejects.toThrow('Parent task 999 not found');
  });
});
