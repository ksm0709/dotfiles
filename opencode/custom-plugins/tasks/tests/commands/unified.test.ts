// tests/commands/unified.test.ts
// TDD: Unified Command Handler Tests

import { unifiedCommand } from '../../src/commands/unified';
import { Storage } from '../../src/lib/storage';
import { Parser } from '../../src/lib/parser';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

// Mock dependencies
jest.mock('../../src/lib/storage');
jest.mock('../../src/lib/parser');

describe('unifiedCommand (TDD)', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let originalXdgDataHome: string | undefined;
  let mockStorage: jest.Mocked<Storage>;
  let mockParser: jest.Mocked<Parser>;

  beforeEach(async () => {
    // Create temporary directory for test isolation
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-unified-test-'));
    
    // Mock environment variables
    originalHome = process.env.HOME;
    originalXdgDataHome = process.env.XDG_DATA_HOME;
    delete process.env.XDG_DATA_HOME;
    process.env.HOME = tempDir;
    
    // Clear all mocks
    jest.clearAllMocks();

    // Setup mock storage
    mockStorage = {
      saveTaskList: jest.fn().mockResolvedValue(undefined),
      readTaskList: jest.fn(),
      listTaskFiles: jest.fn().mockResolvedValue([]),
      deleteTaskList: jest.fn(),
      ensureSessionDir: jest.fn().mockResolvedValue(path.join(tempDir, '.local', 'share', 'opencode', 'tasks', 'test-session')),
      getTasksDir: jest.fn().mockReturnValue(path.join(tempDir, '.local', 'share', 'opencode', 'tasks'))
    } as unknown as jest.Mocked<Storage>;

    // Setup mock parser
    mockParser = {
      parseTaskList: jest.fn(),
      generateTaskList: jest.fn().mockReturnValue(''),
      addTask: jest.fn().mockReturnValue(true),
      updateTaskStatus: jest.fn().mockReturnValue(true),
      removeTask: jest.fn().mockReturnValue(true),
      findTask: jest.fn()
    } as unknown as jest.Mocked<Parser>;

    (Storage as jest.Mock).mockImplementation(() => mockStorage);
    (Parser as jest.Mock).mockImplementation(() => mockParser);
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
    
    // Cleanup temp directory
    try {
      await fs.rm(tempDir, { recursive: true, force: true });
    } catch (error) {
      // Ignore cleanup errors
    }
  });

  describe('Session Isolation', () => {
    it('should only access current session tasks', async () => {
      const sessionId = 'test-session-1';
      
      // Use mockResolvedValue (not Once) since add-task command also calls these methods
      mockStorage.listTaskFiles.mockResolvedValue(['test-agent-tasks.md']);
      mockStorage.readTaskList.mockResolvedValue(`
# Task List: Test Tasks
**에이전트**: test-agent
**세션 ID**: ${sessionId}

## 작업 목록
- [ ] ⏳ **1**. Test task
      `);
      mockParser.parseTaskList.mockReturnValue({
        title: 'Test Tasks',
        agent: 'test-agent',
        sessionId,
        tasks: [{ id: '1', title: 'Task 1', status: 'pending' as const, details: [], createdAt: new Date().toISOString(), updatedAt: new Date().toISOString() }],
        createdAt: new Date().toISOString()
      });

      const result = await unifiedCommand({
        sessionId,
        operations: [{ type: 'add', title: 'New task' }]
      });

      // Should only list files from current session
      expect(mockStorage.listTaskFiles).toHaveBeenCalledWith(sessionId);
      expect(result.currentStatus).not.toBeNull();
    });

    it('should not access other sessions data', async () => {
      const sessionId1 = 'session-1';
      const sessionId2 = 'session-2';

      // Session 1 has tasks
      mockStorage.listTaskFiles.mockImplementation((sessionId: string) => {
        if (sessionId === sessionId1) {
          return Promise.resolve(['agent-tasks.md']);
        }
        return Promise.resolve([]);
      });

      // When operating in session 2, should not see session 1's tasks
      const result = await unifiedCommand({
        sessionId: sessionId2,
        operations: [{ type: 'add', title: 'Task in session 2' }]
      });

      // Should return null status because session 2 has no tasks
      expect(result.currentStatus).toBeNull();
    });
  });

  describe('Operation: init', () => {
    it('should initialize task list with init operation', async () => {
      const sessionId = 'test-session';
      
      const result = await unifiedCommand({
        sessionId,
        operations: [{ type: 'init', agent: 'test-agent', title: 'Test Tasks' }]
      });

      expect(result.success).toBe(true);
      expect(result.results).toHaveLength(1);
      expect(result.results[0].success).toBe(true);
      expect(result.results[0].operation.type).toBe('init');
    });

    it('should fail init without required fields', async () => {
      const sessionId = 'test-session';
      
      const result = await unifiedCommand({
        sessionId,
        operations: [{ type: 'init', agent: '', title: '' }]
      });

      expect(result.success).toBe(false);
      expect(result.results[0].success).toBe(false);
      expect(result.results[0].message).toContain('Missing required fields');
    });
  });

  describe('Operation: add', () => {
    it('should add task with add operation', async () => {
      const sessionId = 'test-session';
      
      // Setup existing task list
      mockStorage.listTaskFiles.mockResolvedValueOnce(['agent-tasks.md']);
      mockStorage.readTaskList.mockResolvedValueOnce('# Task List');
      mockParser.parseTaskList.mockReturnValueOnce({
        title: 'Test Tasks',
        agent: 'test-agent',
        sessionId,
        tasks: [],
        createdAt: new Date().toISOString()
      });

      const result = await unifiedCommand({
        sessionId,
        operations: [{ type: 'add', title: 'New task' }]
      });

      expect(result.results[0].success).toBe(true);
      expect(result.results[0].operation.type).toBe('add');
    });

    it('should fail add without title', async () => {
      const sessionId = 'test-session';
      
      const result = await unifiedCommand({
        sessionId,
        operations: [{ type: 'add', title: '' }]
      });

      expect(result.results[0].success).toBe(false);
      expect(result.results[0].message).toContain('Missing required field: title');
    });
  });

  describe('Operation: update', () => {
    it('should update task status', async () => {
      const sessionId = 'test-session';
      
      mockStorage.listTaskFiles.mockResolvedValueOnce(['agent-tasks.md']);
      mockStorage.readTaskList.mockResolvedValueOnce('# Task List');
      mockParser.parseTaskList.mockReturnValueOnce({
        title: 'Test Tasks',
        agent: 'test-agent',
        sessionId,
        tasks: [{ id: '1', title: 'Task 1', status: 'pending', details: [], createdAt: new Date().toISOString(), updatedAt: new Date().toISOString() }],
        createdAt: new Date().toISOString()
      });

      const result = await unifiedCommand({
        sessionId,
        operations: [{ type: 'update', id: '1', status: 'completed' }]
      });

      expect(result.results[0].success).toBe(true);
      expect(result.results[0].operation.type).toBe('update');
    });

    it('should fail update without id or status', async () => {
      const sessionId = 'test-session';
      
      const result = await unifiedCommand({
        sessionId,
        operations: [
          { type: 'update', id: '', status: 'completed' },
          { type: 'update', id: '1', status: undefined as any }
        ]
      });

      expect(result.results[0].success).toBe(false);
      expect(result.results[1].success).toBe(false);
    });
  });

  describe('Operation: complete', () => {
    it('should complete task', async () => {
      const sessionId = 'test-session';
      
      mockStorage.listTaskFiles.mockResolvedValueOnce(['agent-tasks.md']);
      mockStorage.readTaskList.mockResolvedValueOnce('# Task List');
      mockParser.parseTaskList.mockReturnValueOnce({
        title: 'Test Tasks',
        agent: 'test-agent',
        sessionId,
        tasks: [{ id: '1', title: 'Task 1', status: 'in_progress', details: [], createdAt: new Date().toISOString(), updatedAt: new Date().toISOString() }],
        createdAt: new Date().toISOString()
      });

      const result = await unifiedCommand({
        sessionId,
        operations: [{ type: 'complete', id: '1' }]
      });

      expect(result.results[0].success).toBe(true);
      expect(result.results[0].operation.type).toBe('complete');
    });
  });

  describe('Operation: remove', () => {
    it('should remove task', async () => {
      const sessionId = 'test-session';
      
      mockStorage.listTaskFiles.mockResolvedValueOnce(['agent-tasks.md']);
      mockStorage.readTaskList.mockResolvedValueOnce('# Task List');
      mockParser.parseTaskList.mockReturnValueOnce({
        title: 'Test Tasks',
        agent: 'test-agent',
        sessionId,
        tasks: [{ id: '1', title: 'Task 1', status: 'pending', details: [], createdAt: new Date().toISOString(), updatedAt: new Date().toISOString() }],
        createdAt: new Date().toISOString()
      });

      const result = await unifiedCommand({
        sessionId,
        operations: [{ type: 'remove', id: '1' }]
      });

      expect(result.results[0].success).toBe(true);
      expect(result.results[0].operation.type).toBe('remove');
    });
  });

  describe('Batch Operations', () => {
    it('should execute multiple operations in batch', async () => {
      const sessionId = 'test-session';
      
      mockStorage.listTaskFiles.mockResolvedValue(['agent-tasks.md']);
      mockStorage.readTaskList.mockResolvedValue('# Task List');
      mockParser.parseTaskList.mockReturnValue({
        title: 'Test Tasks',
        agent: 'test-agent',
        sessionId,
        tasks: [],
        createdAt: new Date().toISOString()
      });

      const result = await unifiedCommand({
        sessionId,
        operations: [
          { type: 'add', title: 'Task 1' },
          { type: 'add', title: 'Task 2' },
          { type: 'update', id: '1', status: 'completed' }
        ]
      });

      expect(result.results).toHaveLength(3);
      expect(result.summary.total).toBe(3);
    });

    it('should allow partial failure', async () => {
      const sessionId = 'test-session';
      
      mockStorage.listTaskFiles.mockResolvedValue(['agent-tasks.md']);
      mockStorage.readTaskList.mockResolvedValue('# Task List');
      mockParser.parseTaskList.mockReturnValue({
        title: 'Test Tasks',
        agent: 'test-agent',
        sessionId,
        tasks: [],
        createdAt: new Date().toISOString()
      });

      const result = await unifiedCommand({
        sessionId,
        operations: [
          { type: 'add', title: 'Task 1' },
          { type: 'update', id: '999', status: 'completed' }, // Non-existent task
          { type: 'add', title: 'Task 2' }
        ]
      });

      // Should have attempted all operations
      expect(result.results).toHaveLength(3);
      // Partial failure - some may succeed, some may fail
      expect(result.summary.total).toBe(3);
    });

    it('should reject more than 50 operations', async () => {
      const sessionId = 'test-session';
      
      // Create 51 operations
      const operations = Array(51).fill(null).map((_, i) => ({
        type: 'add' as const,
        title: `Task ${i + 1}`
      }));

      // Should throw error for exceeding maximum operations
      await expect(
        unifiedCommand({ sessionId, operations })
      ).rejects.toThrow('Too many operations: maximum 50 allowed, got 51');
    });
  });

  describe('Response Format', () => {
    it('should return ToolResponse format', async () => {
      const sessionId = 'test-session';
      
      mockStorage.listTaskFiles.mockResolvedValue(['agent-tasks.md']);
      mockStorage.readTaskList.mockResolvedValue('# Task List');
      mockParser.parseTaskList.mockReturnValue({
        title: 'Test Tasks',
        agent: 'test-agent',
        sessionId,
        tasks: [],
        createdAt: new Date().toISOString()
      });

      const result = await unifiedCommand({
        sessionId,
        operations: [{ type: 'add', title: 'Test task' }]
      });

      // Verify ToolResponse structure
      expect(result.response).toHaveProperty('title');
      expect(result.response).toHaveProperty('output');
      expect(result.response).toHaveProperty('metadata');
      expect(result.response.metadata).toHaveProperty('operation', 'unified');
    });

    it('should include current session status', async () => {
      const sessionId = 'test-session';
      
      const mockTaskList = {
        title: 'Test Tasks',
        agent: 'test-agent',
        sessionId,
        tasks: [{ id: '1', title: 'Task 1', status: 'pending' as const, details: [], createdAt: new Date().toISOString(), updatedAt: new Date().toISOString() }],
        createdAt: new Date().toISOString()
      };

      mockStorage.listTaskFiles.mockResolvedValue(['agent-tasks.md']);
      mockStorage.readTaskList.mockResolvedValue('# Task List');
      mockParser.parseTaskList.mockReturnValue(mockTaskList);

      const result = await unifiedCommand({
        sessionId,
        operations: [{ type: 'add', title: 'New task' }]
      });

      expect(result.currentStatus).not.toBeNull();
      expect(result.response.metadata?.taskList).toBeDefined();
    });
  });
});
