import { initCommand, InitArgs, TaskInput } from '../../src/commands/init';
import { Storage } from '../../src/lib/storage';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

// Mock console methods
const mockConsoleLog = jest.spyOn(console, 'log').mockImplementation();
const mockConsoleError = jest.spyOn(console, 'error').mockImplementation();

// Mock Storage
jest.mock('../../src/lib/storage');

describe('initCommand', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let originalXdgDataHome: string | undefined;
  let mockStorage: jest.Mocked<Storage>;

  beforeEach(async () => {
    // Create temporary directory
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-init-test-'));
    
    // Mock environment variables
    originalHome = process.env.HOME;
    originalXdgDataHome = process.env.XDG_DATA_HOME;
    delete process.env.XDG_DATA_HOME;
    process.env.HOME = tempDir;
    
    // Clear mocks
    mockConsoleLog.mockClear();
    mockConsoleError.mockClear();
    jest.clearAllMocks();

    // Setup mock storage
    mockStorage = {
      saveTaskList: jest.fn().mockResolvedValue(undefined),
      readTaskList: jest.fn(),
      listTaskFiles: jest.fn(),
      deleteTaskList: jest.fn(),
      ensureSessionDir: jest.fn().mockResolvedValue(path.join(tempDir, '.local', 'share', 'opencode', 'tasks', 'test-session')),
      getTasksDir: jest.fn().mockReturnValue(path.join(tempDir, '.local', 'share', 'opencode', 'tasks'))
    } as unknown as jest.Mocked<Storage>;

    (Storage as jest.Mock).mockImplementation(() => mockStorage);
  });

  afterEach(async () => {
    // Restore environment variables
    if (originalHome !== undefined) {
      process.env.HOME = originalHome;
    } else {
      delete process.env.HOME;
    }
    
    if (originalXdgDataHome !== undefined) {
      process.env.XDG_DATA_HOME = originalXdgDataHome;
    } else {
      delete process.env.XDG_DATA_HOME;
    }
    
    // Clean up
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

  it('should initialize a new task list with sessionId, agent, and title', async () => {
    const args: InitArgs = {
      sessionId: 'test-session',
      agent: 'test-agent',
      title: 'Test Project'
    };

    const result = await initCommand(args);

    expect(mockStorage.saveTaskList).toHaveBeenCalled();
    expect(result.title).toBe('Test Project');
    expect(result.agent).toBe('test-agent');
    expect(result.taskIds).toEqual([]);
    expect(result.totalTasks).toBe(0);
  });

  it('should initialize with provided tasks', async () => {
    const tasks: TaskInput[] = [
      { id: '1', title: 'First Task', status: 'pending' },
      { id: '2', title: 'Second Task', status: 'in_progress' }
    ];

    const args: InitArgs = {
      sessionId: 'test-session',
      agent: 'test-agent',
      title: 'Test Project',
      tasks
    };

    const result = await initCommand(args);

    expect(mockStorage.saveTaskList).toHaveBeenCalled();
    expect(result.taskIds).toEqual(['1', '2']);
    expect(result.totalTasks).toBe(2);
  });

  it('should handle empty tasks array', async () => {
    const args: InitArgs = {
      sessionId: 'test-session',
      agent: 'test-agent',
      title: 'Empty Project',
      tasks: []
    };

    const result = await initCommand(args);

    expect(result.taskIds).toEqual([]);
    expect(result.totalTasks).toBe(0);
  });

  it('should handle tasks with subtasks', async () => {
    const tasks: TaskInput[] = [
      {
        id: '1',
        title: 'Parent Task',
        status: 'in_progress',
        subtasks: [
          { id: '1.1', title: 'Subtask 1', status: 'completed' },
          { id: '1.2', title: 'Subtask 2', status: 'pending' }
        ]
      }
    ];

    const args: InitArgs = {
      sessionId: 'test-session',
      agent: 'test-agent',
      title: 'Hierarchical Project',
      tasks
    };

    const result = await initCommand(args);

    // Parent (1) + 2 subtasks = 3 total
    expect(result.taskIds).toEqual(['1', '1.1', '1.2']);
    expect(result.totalTasks).toBe(3);
  });

  it('should handle tasks with details', async () => {
    const tasks: TaskInput[] = [
      {
        id: '1',
        title: 'Task with details',
        status: 'pending',
        details: ['Detail 1', 'Detail 2']
      }
    ];

    const args: InitArgs = {
      sessionId: 'test-session',
      agent: 'test-agent',
      title: 'Detailed Project',
      tasks
    };

    await initCommand(args);

    expect(mockStorage.saveTaskList).toHaveBeenCalled();
  });

  it('should generate correct file name from agent and title', async () => {
    const args: InitArgs = {
      sessionId: 'test-session',
      agent: 'senior-sw-engineer',
      title: 'API Implementation'
    };

    await initCommand(args);

    // File name should be: senior-sw-engineer-api-implementation
    expect(mockStorage.saveTaskList).toHaveBeenCalledWith(
      'test-session',
      expect.stringContaining('senior-sw-engineer'),
      expect.any(String)
    );
  });

  it('should handle special characters in agent and title', async () => {
    const args: InitArgs = {
      sessionId: 'test-session',
      agent: 'test@agent#123',
      title: 'Special !!! Task @#$%'
    };

    await initCommand(args);

    expect(mockStorage.saveTaskList).toHaveBeenCalled();
  });

  it('should handle Korean characters', async () => {
    const args: InitArgs = {
      sessionId: 'test-session',
      agent: '테스트-에이전트',
      title: '한국어 프로젝트'
    };

    await initCommand(args);

    expect(mockStorage.saveTaskList).toHaveBeenCalled();
  });

  it('should throw error on storage failure', async () => {
    mockStorage.saveTaskList.mockRejectedValue(new Error('Storage error'));

    const args: InitArgs = {
      sessionId: 'test-session',
      agent: 'test-agent',
      title: 'Test Project'
    };

    await expect(initCommand(args)).rejects.toThrow('Storage error');
    expect(mockConsoleError).toHaveBeenCalled();
  });

  it('should not call process.exit on error', async () => {
    const exitSpy = jest.spyOn(process, 'exit').mockImplementation(() => {
      throw new Error('process.exit should not be called');
    });

    mockStorage.saveTaskList.mockRejectedValue(new Error('Storage error'));

    const args: InitArgs = {
      sessionId: 'test-session',
      agent: 'test-agent',
      title: 'Test Project'
    };

    await expect(initCommand(args)).rejects.toThrow('Storage error');
    expect(exitSpy).not.toHaveBeenCalled();

    exitSpy.mockRestore();
  });
});
