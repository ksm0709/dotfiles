import { initCommand, InitArgs, TaskInput } from '../../src/commands/init';
import { listCommand, ListArgs } from '../../src/commands/list';
import { updateCommand, UpdateArgs } from '../../src/commands/update';
import { completeCommand, CompleteArgs } from '../../src/commands/complete';
import { addTaskCommand, AddTaskArgs } from '../../src/commands/add-task';
import { removeCommand, RemoveArgs } from '../../src/commands/remove';
import { statusCommand, StatusArgs } from '../../src/commands/status';
import { Storage } from '../../src/lib/storage';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

describe('Commands Integration', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let originalXdgDataHome: string | undefined;
  let storage: Storage;

  beforeEach(async () => {
    // Create temporary directory
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-integration-test-'));
    
    // Mock environment variables
    originalHome = process.env.HOME;
    originalXdgDataHome = process.env.XDG_DATA_HOME;
    delete process.env.XDG_DATA_HOME;
    process.env.HOME = tempDir;
    
    // Clear module cache and create fresh Storage instance
    jest.resetModules();
    const storageModule = await import('../../src/lib/storage');
    const StorageClass = storageModule.Storage;
    storage = new StorageClass();
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
  });

  describe('initCommand', () => {
    it('should initialize a new task list', async () => {
      const args: InitArgs = {
        sessionId: 'test-session',
        agent: 'test-agent',
        title: 'Test Project'
      };

      const result = await initCommand(args);

      // Verify file was created
      const sessionDir = path.join(tempDir, '.local', 'share', 'opencode', 'tasks', 'test-session');
      const files = await fs.readdir(sessionDir);
      expect(files.length).toBeGreaterThan(0);
      expect(files[0]).toMatch(/test-agent-test-project\.md$/);
      
      expect(result.title).toBe('Test Project');
      expect(result.agent).toBe('test-agent');
    });

    it('should initialize with tasks', async () => {
      const tasks: TaskInput[] = [
        { id: '1', title: 'Task One', status: 'pending' },
        { id: '2', title: 'Task Two', status: 'in_progress' }
      ];

      const args: InitArgs = {
        sessionId: 'test-session',
        agent: 'test-agent',
        title: 'Test Project',
        tasks
      };

      const result = await initCommand(args);

      expect(result.taskIds).toEqual(['1', '2']);
      expect(result.totalTasks).toBe(2);
    });
  });

  describe('listCommand', () => {
    beforeEach(async () => {
      // Initialize a task list first
      const args: InitArgs = {
        sessionId: 'test-session',
        agent: 'test-agent',
        title: 'Test Project',
        tasks: [
          { id: '1', title: 'Task One', status: 'pending' },
          { id: '2', title: 'Task Two', status: 'pending' }
        ]
      };
      await initCommand(args);
    });

    it('should list tasks in markdown format', async () => {
      const args: ListArgs = {
        sessionId: 'test-session',
        format: 'markdown'
      };

      const result = await listCommand(args);

      expect(result.success).toBe(true);
      expect(result.taskLists.length).toBeGreaterThan(0);
    });

    it('should handle no task lists found', async () => {
      const args: ListArgs = {
        sessionId: 'non-existent-session'
      };

      const result = await listCommand(args);

      expect(result.success).toBe(true);
      expect(result.message).toContain('No task lists found');
    });
  });

  describe('updateCommand', () => {
    beforeEach(async () => {
      // Initialize a task list first
      const args: InitArgs = {
        sessionId: 'test-session',
        agent: 'test-agent',
        title: 'Test Project',
        tasks: [
          { id: '1', title: 'Task One', status: 'pending' },
          { id: '2', title: 'Task Two', status: 'pending' }
        ]
      };
      await initCommand(args);
    });

    it('should update task status', async () => {
      const args: UpdateArgs = {
        sessionId: 'test-session',
        id: '1',
        status: 'in_progress'
      };

      const result = await updateCommand(args);

      expect(result.success).toBe(true);
      expect(result.message).toContain('updated');
    });

    it('should handle task not found', async () => {
      const args: UpdateArgs = {
        sessionId: 'test-session',
        id: '999',
        status: 'completed'
      };

      const result = await updateCommand(args);

      expect(result.success).toBe(false);
      expect(result.message).toContain('not found');
    });
  });

  describe('completeCommand', () => {
    beforeEach(async () => {
      // Initialize a task list first
      const args: InitArgs = {
        sessionId: 'test-session',
        agent: 'test-agent',
        title: 'Test Project',
        tasks: [
          { id: '1', title: 'Task One', status: 'pending' }
        ]
      };
      await initCommand(args);
    });

    it('should mark task as completed', async () => {
      const args: CompleteArgs = {
        sessionId: 'test-session',
        id: '1'
      };

      const result = await completeCommand(args);

      expect(result.success).toBe(true);
      expect(result.message).toContain('completed');
    });
  });

  describe('addTaskCommand', () => {
    beforeEach(async () => {
      // Initialize a task list first
      const args: InitArgs = {
        sessionId: 'test-session',
        agent: 'test-agent',
        title: 'Test Project',
        tasks: [
          { id: '1', title: 'Task One', status: 'pending' }
        ]
      };
      await initCommand(args);
    });

    it('should add a new task', async () => {
      const args: AddTaskArgs = {
        sessionId: 'test-session',
        title: 'New Task'
      };

      const result = await addTaskCommand(args);

      expect(result.success).toBe(true);
      expect(result.title).toBe('New Task');
    });

    it('should handle no task lists found', async () => {
      const args: AddTaskArgs = {
        sessionId: 'non-existent-session',
        title: 'New Task'
      };

      const result = await addTaskCommand(args);

      expect(result.success).toBe(false);
      expect(result.message).toContain('No task lists found');
    });
  });

  describe('removeCommand', () => {
    beforeEach(async () => {
      // Initialize a task list first
      const args: InitArgs = {
        sessionId: 'test-session',
        agent: 'test-agent',
        title: 'Test Project',
        tasks: [
          { id: '1', title: 'Task One', status: 'pending' },
          { id: '2', title: 'Task Two', status: 'pending' }
        ]
      };
      await initCommand(args);
    });

    it('should remove a task', async () => {
      const args: RemoveArgs = {
        sessionId: 'test-session',
        id: '1',
        force: true
      };

      const result = await removeCommand(args);

      expect(result.success).toBe(true);
      expect(result.message).toContain('removed');
    });

    it('should handle task not found', async () => {
      const args: RemoveArgs = {
        sessionId: 'test-session',
        id: '999',
        force: true
      };

      const result = await removeCommand(args);

      expect(result.success).toBe(false);
      expect(result.message).toContain('not found');
    });
  });

  describe('statusCommand', () => {
    beforeEach(async () => {
      // Initialize a task list first
      const args: InitArgs = {
        sessionId: 'test-session',
        agent: 'test-agent',
        title: 'Test Project',
        tasks: [
          { id: '1', title: 'Task One', status: 'completed' },
          { id: '2', title: 'Task Two', status: 'pending' }
        ]
      };
      await initCommand(args);
    });

    it('should show status summary', async () => {
      const args: StatusArgs = {
        sessionId: 'test-session'
      };

      const result = await statusCommand(args);

      expect(result.success).toBe(true);
      expect(result.summaries.length).toBeGreaterThan(0);
    });

    it('should handle no task lists found', async () => {
      const args: StatusArgs = {
        sessionId: 'non-existent-session'
      };

      const result = await statusCommand(args);

      expect(result.success).toBe(true);
      expect(result.message).toContain('No task lists found');
    });
  });

  describe('End-to-end workflow', () => {
    it('should complete full task management workflow', async () => {
      const sessionId = 'workflow-session';
      const agent = 'workflow-agent';
      
      // 1. Initialize
      const initResult = await initCommand({
        sessionId,
        agent,
        title: 'Workflow Test',
        tasks: [
          { id: '1', title: 'Task One', status: 'pending' },
          { id: '2', title: 'Task Two', status: 'pending' },
          { id: '3', title: 'Task Three', status: 'pending' }
        ]
      });
      expect(initResult.taskIds).toContain('1');
      expect(initResult.taskIds).toContain('2');
      expect(initResult.taskIds).toContain('3');

      // 2. Add a task
      const addResult = await addTaskCommand({
        sessionId,
        title: 'New Workflow Task'
      });
      expect(addResult.success).toBe(true);

      // 3. Update status
      const updateResult = await updateCommand({
        sessionId,
        id: '1',
        status: 'in_progress'
      });
      expect(updateResult.success).toBe(true);

      // 4. Complete task
      const completeResult = await completeCommand({
        sessionId,
        id: '2'
      });
      expect(completeResult.success).toBe(true);

      // 5. Check status
      const statusResult = await statusCommand({ sessionId });
      expect(statusResult.success).toBe(true);

      // 6. List tasks
      const listResult = await listCommand({ sessionId, format: 'markdown' });
      expect(listResult.success).toBe(true);

      // 7. Remove a task
      const removeResult = await removeCommand({
        sessionId,
        id: '3',
        force: true
      });
      expect(removeResult.success).toBe(true);
    });
  });
});
