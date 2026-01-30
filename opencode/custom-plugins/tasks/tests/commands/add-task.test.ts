import { addTaskCommand, AddTaskArgs } from '../../src/commands/add-task';
import { Storage } from '../../src/lib/storage';
import { Parser } from '../../src/lib/parser';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

// Mocks
const mockConsoleLog = jest.spyOn(console, 'log').mockImplementation();
const mockConsoleError = jest.spyOn(console, 'error').mockImplementation();
const mockProcessExit = jest.spyOn(process, 'exit').mockImplementation((() => {}) as any);

jest.mock('../../src/lib/storage');
jest.mock('../../src/lib/parser');

describe('addTaskCommand', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let mockStorage: jest.Mocked<Storage>;
  let mockParser: jest.Mocked<Parser>;

  beforeEach(async () => {
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-add-test-'));
    originalHome = process.env.HOME;
    process.env.HOME = tempDir;

    mockConsoleLog.mockClear();
    mockConsoleError.mockClear();
    mockProcessExit.mockClear();
    jest.clearAllMocks();

    mockStorage = new Storage() as jest.Mocked<Storage>;
    mockParser = new Parser() as jest.Mocked<Parser>;

    (Storage as jest.MockedClass<typeof Storage>).mockImplementation(() => mockStorage);
    (Parser as jest.MockedClass<typeof Parser>).mockImplementation(() => mockParser);
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
    mockConsoleLog.mockClear();
    mockConsoleError.mockClear();
    mockProcessExit.mockClear();
  });

  afterAll(() => {
    mockConsoleLog.mockRestore();
    mockConsoleError.mockRestore();
    mockProcessExit.mockRestore();
  });

  it('should add a new top-level task', async () => {
    const args: AddTaskArgs = {
      agent: 'test-agent',
      title: 'New Task',
      details: 'Detail 1, Detail 2'
    };

    const taskList = {
      title: 'Test',
      tasks: [{ id: '1' }]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockParser.addTask.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated content');

    await addTaskCommand(args);

    expect(mockParser.addTask).toHaveBeenCalledWith(
      taskList,
      undefined,
      'New Task',
      ['Detail 1', 'Detail 2']
    );
    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('added')
    );
    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('Detail 1, Detail 2')
    );
  });

  it('should add a subtask to parent', async () => {
    const args: AddTaskArgs = {
      agent: 'test-agent',
      title: 'Subtask',
      parent: '1'
    };

    const taskList = {
      title: 'Test',
      tasks: [{ id: '1' }]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockParser.addTask.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated');

    await addTaskCommand(args);

    expect(mockParser.addTask).toHaveBeenCalledWith(
      taskList,
      '1',
      'Subtask',
      []
    );
  });

  it('should handle task with no details', async () => {
    const args: AddTaskArgs = {
      agent: 'test-agent',
      title: 'Simple Task'
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue({ tasks: [] } as any);
    mockParser.addTask.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated');

    await addTaskCommand(args);

    expect(mockParser.addTask).toHaveBeenCalledWith(
      expect.anything(),
      undefined,
      'Simple Task',
      []
    );
  });

  it('should handle non-existent parent', async () => {
    const args: AddTaskArgs = {
      agent: 'test-agent',
      title: 'Orphan Task',
      parent: '999'
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue({ tasks: [] } as any);
    mockParser.addTask.mockReturnValue(false);

    await addTaskCommand(args);

    expect(mockConsoleError).toHaveBeenCalled();
    expect(mockProcessExit).toHaveBeenCalledWith(1);
  });

  it('should handle no task lists found', async () => {
    const args: AddTaskArgs = {
      agent: 'non-existent',
      title: 'Task'
    };

    mockStorage.listTaskFiles.mockResolvedValue([]);

    await addTaskCommand(args);

    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('No task lists found')
    );
    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('Create a task list first')
    );
  });

  it('should handle read error', async () => {
    const args: AddTaskArgs = {
      agent: 'test-agent',
      title: 'Task'
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue(null);

    await addTaskCommand(args);

    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('Failed to read')
    );
    expect(mockProcessExit).toHaveBeenCalledWith(1);
  });
});
