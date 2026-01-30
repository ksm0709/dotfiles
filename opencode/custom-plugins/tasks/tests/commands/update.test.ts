import { updateCommand, UpdateArgs } from '../../src/commands/update';
import { Storage } from '../../src/lib/storage';
import { Parser } from '../../src/lib/parser';
import { TaskStatus } from '../../src/types';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

// Mocks
const mockConsoleLog = jest.spyOn(console, 'log').mockImplementation();
const mockConsoleError = jest.spyOn(console, 'error').mockImplementation();
const mockProcessExit = jest.spyOn(process, 'exit').mockImplementation((() => {}) as any);

jest.mock('../../src/lib/storage');
jest.mock('../../src/lib/parser');

describe('updateCommand', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let mockStorage: jest.Mocked<Storage>;
  let mockParser: jest.Mocked<Parser>;

  beforeEach(async () => {
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-update-test-'));
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

  it('should update task status successfully', async () => {
    const args: UpdateArgs = {
      agent: 'test-agent',
      id: '1',
      status: 'completed' as TaskStatus
    };

    const taskList = {
      title: 'Test',
      tasks: [{ id: '1', status: 'pending' }]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockParser.updateTaskStatus.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated content');

    await updateCommand(args);

    expect(mockParser.updateTaskStatus).toHaveBeenCalledWith(taskList, '1', 'completed');
    expect(mockStorage.saveTaskList).toHaveBeenCalledWith('test-agent', 'project', 'updated content');
    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('updated')
    );
  });

  it('should update task to in_progress status', async () => {
    const args: UpdateArgs = {
      agent: 'test-agent',
      id: '2',
      status: 'in_progress' as TaskStatus
    };

    const taskList = {
      title: 'Test',
      tasks: [{ id: '2', status: 'pending' }]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockParser.updateTaskStatus.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated content');

    await updateCommand(args);

    expect(mockParser.updateTaskStatus).toHaveBeenCalledWith(taskList, '2', 'in_progress');
  });

  it('should update task to pending status', async () => {
    const args: UpdateArgs = {
      agent: 'test-agent',
      id: '1',
      status: 'pending' as TaskStatus
    };

    const taskList = {
      title: 'Test',
      tasks: [{ id: '1', status: 'completed' }]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockParser.updateTaskStatus.mockReturnValue(true);
    mockParser.generateTaskList.mockReturnValue('updated content');

    await updateCommand(args);

    expect(mockParser.updateTaskStatus).toHaveBeenCalledWith(taskList, '1', 'pending');
  });

  it('should handle task not found', async () => {
    const args: UpdateArgs = {
      agent: 'test-agent',
      id: '999',
      status: 'completed' as TaskStatus
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue({ tasks: [] } as any);
    mockParser.updateTaskStatus.mockReturnValue(false);

    await updateCommand(args);

    expect(mockConsoleError).toHaveBeenCalled();
    expect(mockProcessExit).toHaveBeenCalledWith(1);
  });

  it('should handle no task lists found', async () => {
    const args: UpdateArgs = {
      agent: 'non-existent',
      id: '1',
      status: 'completed' as TaskStatus
    };

    mockStorage.listTaskFiles.mockResolvedValue([]);

    await updateCommand(args);

    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('No task lists found')
    );
  });

  it('should search multiple files until task found', async () => {
    const args: UpdateArgs = {
      agent: 'test-agent',
      id: '1',
      status: 'completed' as TaskStatus
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project1.md', 'project2.md']);
    mockStorage.readTaskList
      .mockResolvedValueOnce('content1')
      .mockResolvedValueOnce('content2');
    mockParser.parseTaskList
      .mockReturnValueOnce({ tasks: [{ id: '2' }] } as any)
      .mockReturnValueOnce({ tasks: [{ id: '1' }] } as any);
    mockParser.updateTaskStatus
      .mockReturnValueOnce(false)
      .mockReturnValueOnce(true);
    mockParser.generateTaskList.mockReturnValue('updated');

    await updateCommand(args);

    expect(mockStorage.readTaskList).toHaveBeenCalledTimes(2);
    expect(mockParser.updateTaskStatus).toHaveBeenCalledTimes(2);
  });
});
