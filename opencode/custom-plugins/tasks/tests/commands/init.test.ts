import { initCommand, InitArgs } from '../../src/commands/init';
import { Storage } from '../../src/lib/storage';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

// Mock console and process.exit
const mockConsoleLog = jest.spyOn(console, 'log').mockImplementation();
const mockConsoleError = jest.spyOn(console, 'error').mockImplementation();
const mockProcessExit = jest.spyOn(process, 'exit').mockImplementation((() => {}) as any);

// Mock Storage
jest.mock('../../src/lib/storage');

describe('initCommand', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let tasksFile: string;

  beforeEach(async () => {
    // Create temporary directory
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-init-test-'));
    
    // Mock HOME
    originalHome = process.env.HOME;
    process.env.HOME = tempDir;
    
    // Create a sample tasks.md file
    tasksFile = path.join(tempDir, 'tasks.md');
    await fs.writeFile(tasksFile, `# Test Tasks

## 작업 목록

- [ ] 1. First Task
  - Detail A
- [ ] 2. Second Task
  - [ ] 2.1. Subtask
`);
    
    // Clear mocks
    mockConsoleLog.mockClear();
    mockConsoleError.mockClear();
    mockProcessExit.mockClear();
    jest.clearAllMocks();
  });

  afterEach(async () => {
    // Restore HOME
    if (originalHome !== undefined) {
      process.env.HOME = originalHome;
    }
    
    // Clean up
    try {
      await fs.rm(tempDir, { recursive: true, force: true });
    } catch (error) {
      // Ignore
    }
    
    // Clear mocks
    mockConsoleLog.mockClear();
    mockConsoleError.mockClear();
    mockProcessExit.mockClear();
  });

  afterAll(() => {
    mockConsoleLog.mockRestore();
    mockConsoleError.mockRestore();
    mockProcessExit.mockRestore();
  });

  it('should initialize a new task list from file', async () => {
    const args: InitArgs = {
      agent: 'test-agent',
      title: 'Test Project',
      file: tasksFile
    };

    await initCommand(args);

    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('initialized')
    );
    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('Total tasks')
    );
  });

  it('should count all tasks including subtasks', async () => {
    const args: InitArgs = {
      agent: 'test-agent',
      title: 'Test Project',
      file: tasksFile
    };

    await initCommand(args);

    // Should count 3 tasks (2 top-level + 1 subtask)
    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('3')
    );
  });

  it('should handle file read error', async () => {
    const args: InitArgs = {
      agent: 'test-agent',
      title: 'Test',
      file: '/non/existent/file.md'
    };

    await initCommand(args);

    expect(mockConsoleError).toHaveBeenCalled();
    expect(mockProcessExit).toHaveBeenCalledWith(1);
  });

  it('should handle empty task file', async () => {
    const emptyFile = path.join(tempDir, 'empty.md');
    await fs.writeFile(emptyFile, '# Empty Tasks\n\n## 작업 목록\n');

    const args: InitArgs = {
      agent: 'test-agent',
      title: 'Empty Project',
      file: emptyFile
    };

    await initCommand(args);

    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('Total tasks: 0')
    );
  });

  it('should handle completed tasks in source', async () => {
    const fileWithCompleted = path.join(tempDir, 'completed.md');
    await fs.writeFile(fileWithCompleted, `# Tasks

## 작업 목록

- [x] 1. Completed Task
- [ ] 2. Pending Task
`);

    const args: InitArgs = {
      agent: 'test-agent',
      title: 'Mixed Project',
      file: fileWithCompleted
    };

    await initCommand(args);

    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('initialized')
    );
  });
});
