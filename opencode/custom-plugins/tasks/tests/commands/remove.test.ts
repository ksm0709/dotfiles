import { removeCommand, RemoveArgs } from '../../src/commands/remove';
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

describe('removeCommand', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let mockStorage: jest.Mocked<Storage>;
  let mockParser: jest.Mocked<Parser>;

  beforeEach(async () => {
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-remove-test-'));
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

  it('should remove task with force flag', async () => {
    const args: RemoveArgs = {
      agent: 'test-agent',
      id: '1',
      force: true
    };

    const taskList = {
      title: 'Test',
      tasks: [{ id: '1', title: 'Task to Remove' }]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockParser.removeTask.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated');

    await removeCommand(args);

    expect(mockParser.removeTask).toHaveBeenCalledWith(taskList, '1');
    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('removed')
    );
  });

  it('should handle task not found', async () => {
    const args: RemoveArgs = {
      agent: 'test-agent',
      id: '999',
      force: true
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue({ tasks: [] } as any);

    await removeCommand(args);

    expect(mockConsoleError).toHaveBeenCalled();
    expect(mockProcessExit).toHaveBeenCalledWith(1);
  });

  it('should handle no task lists found', async () => {
    const args: RemoveArgs = {
      agent: 'non-existent',
      id: '1',
      force: true
    };

    mockStorage.listTaskFiles.mockResolvedValue([]);

    await removeCommand(args);

    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('No task lists found')
    );
  });

  it('should remove nested subtask', async () => {
    const args: RemoveArgs = {
      agent: 'test-agent',
      id: '1.1',
      force: true
    };

    const taskList = {
      title: 'Test',
      tasks: [
        {
          id: '1',
          title: 'Parent',
          subtasks: [{ id: '1.1', title: 'Child' }]
        }
      ]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockParser.removeTask.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated');

    await removeCommand(args);

    expect(mockParser.removeTask).toHaveBeenCalledWith(taskList, '1.1');
    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('Child')
    );
  });
});
