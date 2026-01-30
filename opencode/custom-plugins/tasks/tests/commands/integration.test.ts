import { initCommand } from '../../src/commands/init';
import { listCommand } from '../../src/commands/list';
import { updateCommand } from '../../src/commands/update';
import { completeCommand } from '../../src/commands/complete';
import { addTaskCommand } from '../../src/commands/add-task';
import { removeCommand } from '../../src/commands/remove';
import { statusCommand } from '../../src/commands/status';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

// Mock console methods
const mockConsoleLog = jest.spyOn(console, 'log').mockImplementation();
const mockConsoleError = jest.spyOn(console, 'error').mockImplementation();
const mockProcessExit = jest.spyOn(process, 'exit').mockImplementation((() => {}) as any);

describe('Commands Integration', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let tasksFile: string;

  beforeEach(async () => {
    // Create temporary directory
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-cmd-test-'));
    
    // Mock HOME
    originalHome = process.env.HOME;
    process.env.HOME = tempDir;
    
    // Create a sample tasks.md file
    tasksFile = path.join(tempDir, 'tasks.md');
    await fs.writeFile(tasksFile, `# Test Tasks

## 작업 목록

- [ ] 1. Task One
  - Detail A
- [ ] 2. Task Two
  - [ ] 2.1. Subtask
- [ ] 3. Task Three
`);
    
    // Clear console mocks
    mockConsoleLog.mockClear();
    mockConsoleError.mockClear();
    mockProcessExit.mockClear();
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

  describe('initCommand', () => {
    it('should initialize a new task list', async () => {
      await initCommand({
        agent: 'test-agent',
        title: 'Test Project',
        file: tasksFile
      });

      // Verify task list was created
      const agentDir = path.join(tempDir, '.config', 'opencode', 'tasks', 'test-agent');
      const files = await fs.readdir(agentDir);
      expect(files).toHaveLength(1);
      expect(files[0]).toMatch(/test-project\.md$/);
    });

    it('should handle non-existent source file', async () => {
      await initCommand({
        agent: 'test-agent',
        title: 'Test',
        file: '/non/existent/file.md'
      });

      expect(mockConsoleError).toHaveBeenCalled();
      expect(mockProcessExit).toHaveBeenCalledWith(1);
    });
  });

  describe('listCommand', () => {
    beforeEach(async () => {
      // Initialize a task list first
      await initCommand({
        agent: 'test-agent',
        title: 'Test Project',
        file: tasksFile
      });
    });

    it('should list tasks in markdown format', async () => {
      mockConsoleLog.mockClear();
      
      await listCommand({
        agent: 'test-agent',
        format: 'markdown'
      });

      expect(mockConsoleLog).toHaveBeenCalled();
      const output = mockConsoleLog.mock.calls.map(call => call[0]).join('\n');
      expect(output).toContain('Test Project');
      expect(output).toContain('Task One');
    });

    it('should list tasks in JSON format', async () => {
      mockConsoleLog.mockClear();
      
      await listCommand({
        agent: 'test-agent',
        format: 'json'
      });

      expect(mockConsoleLog).toHaveBeenCalled();
      const output = mockConsoleLog.mock.calls.map(call => call[0]).join('\n');
      // JSON output should be parseable
      expect(() => JSON.parse(output)).not.toThrow();
    });

    it('should list tasks in table format', async () => {
      mockConsoleLog.mockClear();
      
      await listCommand({
        agent: 'test-agent',
        format: 'table'
      });

      expect(mockConsoleLog).toHaveBeenCalled();
      const output = mockConsoleLog.mock.calls.map(call => call[0]).join('\n');
      expect(output).toContain('| ID |');
      expect(output).toContain('상태');
    });

    it('should handle non-existent agent', async () => {
      await listCommand({
        agent: 'non-existent',
        format: 'markdown'
      });

      expect(mockConsoleLog).toHaveBeenCalledWith(
        expect.stringContaining('No task lists found')
      );
    });
  });

  describe('updateCommand', () => {
    beforeEach(async () => {
      await initCommand({
        agent: 'test-agent',
        title: 'Test Project',
        file: tasksFile
      });
    });

    it('should update task status', async () => {
      await updateCommand({
        agent: 'test-agent',
        id: '1',
        status: 'completed'
      });

      expect(mockConsoleLog).toHaveBeenCalledWith(
        expect.stringContaining('updated')
      );
    });

    it('should handle non-existent task', async () => {
      await updateCommand({
        agent: 'test-agent',
        id: '999',
        status: 'completed'
      });

      expect(mockConsoleError).toHaveBeenCalled();
      expect(mockProcessExit).toHaveBeenCalledWith(1);
    });

    it('should handle non-existent agent', async () => {
      await updateCommand({
        agent: 'non-existent',
        id: '1',
        status: 'completed'
      });

      expect(mockConsoleLog).toHaveBeenCalledWith(
        expect.stringContaining('No task lists found')
      );
    });
  });

  describe('completeCommand', () => {
    beforeEach(async () => {
      await initCommand({
        agent: 'test-agent',
        title: 'Test Project',
        file: tasksFile
      });
    });

    it('should mark task as completed', async () => {
      await completeCommand({
        agent: 'test-agent',
        id: '1'
      });

      expect(mockConsoleLog).toHaveBeenCalledWith(
        expect.stringContaining('completed')
      );
    });

    it('should handle non-existent task', async () => {
      await completeCommand({
        agent: 'test-agent',
        id: '999'
      });

      expect(mockConsoleError).toHaveBeenCalled();
      expect(mockProcessExit).toHaveBeenCalledWith(1);
    });
  });

  describe('addTaskCommand', () => {
    beforeEach(async () => {
      await initCommand({
        agent: 'test-agent',
        title: 'Test Project',
        file: tasksFile
      });
    });

    it('should add a new task', async () => {
      await addTaskCommand({
        agent: 'test-agent',
        title: 'New Task',
        details: 'Detail 1, Detail 2',
        parent: undefined
      });

      expect(mockConsoleLog).toHaveBeenCalledWith(
        expect.stringContaining('added')
      );
    });

    it('should add a subtask', async () => {
      await addTaskCommand({
        agent: 'test-agent',
        title: 'Subtask',
        parent: '1'
      });

      expect(mockConsoleLog).toHaveBeenCalledWith(
        expect.stringContaining('added')
      );
    });

    it('should handle non-existent parent', async () => {
      await addTaskCommand({
        agent: 'test-agent',
        title: 'Orphan Task',
        parent: '999'
      });

      expect(mockConsoleError).toHaveBeenCalled();
      expect(mockProcessExit).toHaveBeenCalledWith(1);
    });

    it('should handle non-existent agent', async () => {
      await addTaskCommand({
        agent: 'non-existent',
        title: 'Task'
      });

      expect(mockConsoleLog).toHaveBeenCalledWith(
        expect.stringContaining('No task lists found')
      );
    });
  });

  describe('removeCommand', () => {
    beforeEach(async () => {
      await initCommand({
        agent: 'test-agent',
        title: 'Test Project',
        file: tasksFile
      });
    });

    it('should remove a task with force', async () => {
      await removeCommand({
        agent: 'test-agent',
        id: '1',
        force: true
      });

      expect(mockConsoleLog).toHaveBeenCalledWith(
        expect.stringContaining('removed')
      );
    });

    it('should handle non-existent task', async () => {
      await removeCommand({
        agent: 'test-agent',
        id: '999',
        force: true
      });

      expect(mockConsoleLog).toHaveBeenCalledWith(
        expect.stringContaining('not found')
      );
      expect(mockProcessExit).toHaveBeenCalledWith(1);
    });

    it('should handle non-existent agent', async () => {
      await removeCommand({
        agent: 'non-existent',
        id: '1',
        force: true
      });

      expect(mockConsoleLog).toHaveBeenCalledWith(
        expect.stringContaining('No task lists found')
      );
    });
  });

  describe('statusCommand', () => {
    beforeEach(async () => {
      await initCommand({
        agent: 'test-agent',
        title: 'Test Project',
        file: tasksFile
      });
    });

    it('should show status summary', async () => {
      await statusCommand({
        agent: 'test-agent'
      });

      expect(mockConsoleLog).toHaveBeenCalled();
      const output = mockConsoleLog.mock.calls.map(call => call[0]).join('\n');
      expect(output).toContain('Task Status');
      expect(output).toContain('Test Project');
    });

    it('should calculate completion rate correctly', async () => {
      // Complete one task first
      await completeCommand({
        agent: 'test-agent',
        id: '1'
      });

      mockConsoleLog.mockClear();

      await statusCommand({
        agent: 'test-agent'
      });

      const output = mockConsoleLog.mock.calls.map(call => call[0]).join('\n');
      expect(output).toMatch(/\d+%/); // Should show percentage
    });

    it('should handle non-existent agent', async () => {
      await statusCommand({
        agent: 'non-existent'
      });

      expect(mockConsoleLog).toHaveBeenCalledWith(
        expect.stringContaining('No task lists found')
      );
    });
  });

  describe('End-to-end workflow', () => {
    it('should complete full task management workflow', async () => {
      const agent = 'workflow-agent';
      
      // 1. Initialize
      await initCommand({
        agent,
        title: 'Workflow Test',
        file: tasksFile
      });

      // 2. Add a task
      await addTaskCommand({
        agent,
        title: 'New Workflow Task',
        details: 'Step 1, Step 2'
      });

      // 3. Update status
      await updateCommand({
        agent,
        id: '1',
        status: 'in_progress'
      });

      // 4. Complete task
      await completeCommand({
        agent,
        id: '2'
      });

      // 5. Check status
      mockConsoleLog.mockClear();
      await statusCommand({ agent });
      
      const statusOutput = mockConsoleLog.mock.calls.map(call => call[0]).join('\n');
      expect(statusOutput).toContain('Workflow Test');

      // 6. List tasks
      mockConsoleLog.mockClear();
      await listCommand({ agent, format: 'markdown' });
      
      const listOutput = mockConsoleLog.mock.calls.map(call => call[0]).join('\n');
      expect(listOutput).toContain('New Workflow Task');

      // 7. Remove a task
      await removeCommand({
        agent,
        id: '3',
        force: true
      });

      // Verify removal
      mockConsoleLog.mockClear();
      await listCommand({ agent, format: 'markdown' });
      
      const finalOutput = mockConsoleLog.mock.calls.map(call => call[0]).join('\n');
      expect(finalOutput).not.toContain('Task Three');
    });
  });
});
