import { statusCommand, StatusArgs } from '../../src/commands/status';
import { Storage } from '../../src/lib/storage';
import { Parser } from '../../src/lib/parser';
import { Formatter } from '../../src/lib/formatter';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

// Mocks
const mockConsoleLog = jest.spyOn(console, 'log').mockImplementation();
const mockConsoleError = jest.spyOn(console, 'error').mockImplementation();

jest.mock('../../src/lib/storage');
jest.mock('../../src/lib/parser');
jest.mock('../../src/lib/formatter');

describe('statusCommand', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let mockStorage: jest.Mocked<Storage>;
  let mockParser: jest.Mocked<Parser>;
  let mockFormatter: jest.Mocked<Formatter>;

  beforeEach(async () => {
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-status-test-'));
    originalHome = process.env.HOME;
    process.env.HOME = tempDir;

    mockConsoleLog.mockClear();
    mockConsoleError.mockClear();
    jest.clearAllMocks();

    mockStorage = new Storage() as jest.Mocked<Storage>;
    mockParser = new Parser() as jest.Mocked<Parser>;
    mockFormatter = new Formatter() as jest.Mocked<Formatter>;

    (Storage as jest.MockedClass<typeof Storage>).mockImplementation(() => mockStorage);
    (Parser as jest.MockedClass<typeof Parser>).mockImplementation(() => mockParser);
    (Formatter as jest.MockedClass<typeof Formatter>).mockImplementation(() => mockFormatter);
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
  });

  afterAll(() => {
    mockConsoleLog.mockRestore();
    mockConsoleError.mockRestore();
  });

  it('should show status summary for single task list', async () => {
    const args: StatusArgs = { agent: 'test-agent' };

    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      tasks: [
        { id: '1', status: 'completed' },
        { id: '2', status: 'pending' }
      ],
      currentPhase: 'Implementation'
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockFormatter.formatStatusSummary.mockReturnValue('Status Output');

    await statusCommand(args);

    expect(mockStorage.listTaskFiles).toHaveBeenCalledWith('test-agent');
    expect(mockFormatter.formatStatusSummary).toHaveBeenCalled();
    expect(mockConsoleLog).toHaveBeenCalledWith('Status Output');
  });

  it('should show status for multiple task lists', async () => {
    const args: StatusArgs = { agent: 'test-agent' };

    mockStorage.listTaskFiles.mockResolvedValue(['project1.md', 'project2.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList
      .mockReturnValueOnce({
        title: 'Project 1',
        agent: 'test-agent',
        tasks: [{ id: '1', status: 'completed' }],
        currentPhase: 'Done'
      } as any)
      .mockReturnValueOnce({
        title: 'Project 2',
        agent: 'test-agent',
        tasks: [{ id: '1', status: 'pending' }],
        currentPhase: 'Planning'
      } as any);
    mockFormatter.formatStatusSummary.mockReturnValue('Status');

    await statusCommand(args);

    expect(mockStorage.readTaskList).toHaveBeenCalledTimes(2);
    expect(mockFormatter.formatStatusSummary).toHaveBeenCalledTimes(2);
  });

  it('should calculate correct stats for completed tasks', async () => {
    const args: StatusArgs = { agent: 'test-agent' };

    const taskList = {
      title: 'Completed Project',
      agent: 'test-agent',
      tasks: [
        { id: '1', status: 'completed' },
        { id: '2', status: 'completed' }
      ]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockFormatter.formatStatusSummary.mockImplementation((summary) => {
      expect(summary.status).toBe('completed');
      expect(summary.completionRate).toBe(100);
      expect(summary.completedCount).toBe(2);
      expect(summary.totalCount).toBe(2);
      return 'Status';
    });

    await statusCommand(args);

    expect(mockFormatter.formatStatusSummary).toHaveBeenCalled();
  });

  it('should calculate correct stats for in_progress tasks', async () => {
    const args: StatusArgs = { agent: 'test-agent' };

    const taskList = {
      title: 'In Progress Project',
      agent: 'test-agent',
      tasks: [
        { id: '1', status: 'completed' },
        { id: '2', status: 'in_progress' },
        { id: '3', status: 'pending' }
      ]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockFormatter.formatStatusSummary.mockImplementation((summary) => {
      expect(summary.status).toBe('in_progress');
      expect(summary.completionRate).toBe(33);
      expect(summary.completedCount).toBe(1);
      expect(summary.totalCount).toBe(3);
      return 'Status';
    });

    await statusCommand(args);

    expect(mockFormatter.formatStatusSummary).toHaveBeenCalled();
  });

  it('should calculate correct stats with nested subtasks', async () => {
    const args: StatusArgs = { agent: 'test-agent' };

    const taskList = {
      title: 'Project with Subtasks',
      agent: 'test-agent',
      tasks: [
        {
          id: '1',
          status: 'completed',
          subtasks: [
            { id: '1.1', status: 'completed' },
            { id: '1.2', status: 'pending' }
          ]
        }
      ]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockFormatter.formatStatusSummary.mockImplementation((summary) => {
      expect(summary.completedCount).toBe(2);
      expect(summary.totalCount).toBe(3);
      expect(summary.completionRate).toBe(67);
      return 'Status';
    });

    await statusCommand(args);

    expect(mockFormatter.formatStatusSummary).toHaveBeenCalled();
  });

  it('should handle no task lists found', async () => {
    const args: StatusArgs = { agent: 'non-existent' };

    mockStorage.listTaskFiles.mockResolvedValue([]);

    await statusCommand(args);

    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('No task lists found')
    );
  });

  it('should skip files that cannot be read', async () => {
    const args: StatusArgs = { agent: 'test-agent' };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue(null);

    await statusCommand(args);

    expect(mockParser.parseTaskList).not.toHaveBeenCalled();
  });

  it('should include current phase in summary', async () => {
    const args: StatusArgs = { agent: 'test-agent' };

    const taskList = {
      title: 'Project',
      agent: 'test-agent',
      tasks: [],
      currentPhase: 'Testing Phase'
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockFormatter.formatStatusSummary.mockImplementation((summary) => {
      expect(summary.currentPhase).toBe('Testing Phase');
      return 'Status';
    });

    await statusCommand(args);

    expect(mockFormatter.formatStatusSummary).toHaveBeenCalled();
  });
});
