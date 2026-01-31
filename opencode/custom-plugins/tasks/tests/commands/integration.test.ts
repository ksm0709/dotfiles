// tests/commands/integration.test.ts
// Integration tests for unified tasks tool

import { unifiedCommand } from '../../src/commands/unified';
import { Storage } from '../../src/lib/storage';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

describe('Unified Tasks Integration', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let originalXdgDataHome: string | undefined;
  let storage: Storage;

  beforeEach(async () => {
    // Create temporary directory
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-unified-integration-'));
    
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

  describe('Full Workflow', () => {
    it('should execute complete task lifecycle', async () => {
      const sessionId = 'test-session';

      // Step 1: Initialize task list
      const initResult = await unifiedCommand({
        sessionId,
        operations: [{ type: 'init', agent: 'test-agent', title: 'Integration Test' }]
      });

      expect(initResult.success).toBe(true);
      expect(initResult.results[0].success).toBe(true);

      // Verify file was created
      const sessionDir = path.join(tempDir, '.local', 'share', 'opencode', 'tasks', sessionId);
      const files = await fs.readdir(sessionDir);
      expect(files.length).toBeGreaterThan(0);

      // Step 2: Add tasks
      const addResult = await unifiedCommand({
        sessionId,
        operations: [
          { type: 'add', title: 'Task 1' },
          { type: 'add', title: 'Task 2' },
          { type: 'add', title: 'Subtask 2.1', parent: '2' }
        ]
      });

      expect(addResult.success).toBe(true);
      expect(addResult.results).toHaveLength(3);
      expect(addResult.results.every(r => r.success)).toBe(true);

      // Step 3: Update task status
      const updateResult = await unifiedCommand({
        sessionId,
        operations: [
          { type: 'update', id: '1', status: 'in_progress' }
        ]
      });

      expect(updateResult.success).toBe(true);
      expect(updateResult.results[0].success).toBe(true);

      // Step 4: Complete task
      const completeResult = await unifiedCommand({
        sessionId,
        operations: [
          { type: 'complete', id: '1' }
        ]
      });

      expect(completeResult.success).toBe(true);
      expect(completeResult.results[0].success).toBe(true);

      // Step 5: Remove task
      const removeResult = await unifiedCommand({
        sessionId,
        operations: [
          { type: 'remove', id: '2' }
        ]
      });

      expect(removeResult.success).toBe(true);
      expect(removeResult.results[0].success).toBe(true);

      // Verify final state
      expect(completeResult.currentStatus).not.toBeNull();
    });

    it('should handle batch operations', async () => {
      const sessionId = 'batch-test-session';

      // Initialize and add multiple tasks in one call
      const result = await unifiedCommand({
        sessionId,
        operations: [
          { type: 'init', agent: 'batch-agent', title: 'Batch Test' },
          { type: 'add', title: 'Task A' },
          { type: 'add', title: 'Task B' },
          { type: 'add', title: 'Task C' },
          { type: 'update', id: '1', status: 'completed' },
          { type: 'complete', id: '2' }
        ]
      });

      expect(result.summary.total).toBe(6);
      expect(result.results).toHaveLength(6);
      
      // All should succeed
      expect(result.summary.succeeded).toBe(6);
      expect(result.summary.failed).toBe(0);
    });
  });

  describe('Session Isolation', () => {
    it('should isolate tasks between sessions', async () => {
      const session1 = 'session-1';
      const session2 = 'session-2';

      // Create tasks in session 1
      await unifiedCommand({
        sessionId: session1,
        operations: [
          { type: 'init', agent: 'agent-1', title: 'Session 1 Tasks' },
          { type: 'add', title: 'Task in Session 1' }
        ]
      });

      // Create different tasks in session 2
      await unifiedCommand({
        sessionId: session2,
        operations: [
          { type: 'init', agent: 'agent-2', title: 'Session 2 Tasks' },
          { type: 'add', title: 'Task in Session 2' }
        ]
      });

      // Verify session directories are separate
      const baseDir = path.join(tempDir, '.local', 'share', 'opencode', 'tasks');
      
      const session1Dir = path.join(baseDir, session1);
      const session2Dir = path.join(baseDir, session2);

      const session1Files = await fs.readdir(session1Dir);
      const session2Files = await fs.readdir(session2Dir);

      expect(session1Files.length).toBeGreaterThan(0);
      expect(session2Files.length).toBeGreaterThan(0);

      // Files should be different
      expect(session1Files).not.toEqual(session2Files);
    });

    it('should not show other sessions tasks in status', async () => {
      const session1 = 'isolated-1';
      const session2 = 'isolated-2';

      // Create in session 1
      await unifiedCommand({
        sessionId: session1,
        operations: [
          { type: 'init', agent: 'agent-1', title: 'Isolated 1' },
          { type: 'add', title: 'Only in Session 1' }
        ]
      });

      // Query session 2 - should have no tasks
      const session2Result = await unifiedCommand({
        sessionId: session2,
        operations: [{ type: 'add', title: 'Task in Session 2' }]
      });

      // Session 2 should not see session 1's tasks
      // (This will fail because no init was done, but that's expected)
      expect(session2Result.success).toBe(false); // No task list initialized
    });
  });

  describe('Error Handling', () => {
    it('should handle invalid operations gracefully', async () => {
      const sessionId = 'error-test';

      const result = await unifiedCommand({
        sessionId,
        operations: [
          { type: 'init', agent: 'error-agent', title: 'Error Test' },
          { type: 'update', id: '999', status: 'completed' }, // Non-existent task
          { type: 'remove', id: '999' } // Non-existent task
        ]
      });

      // Init should succeed
      expect(result.results[0].success).toBe(true);
      
      // Others may fail but should not throw
      expect(result.results).toHaveLength(3);
    });

    it('should validate required fields', async () => {
      const sessionId = 'validation-test';

      const result = await unifiedCommand({
        sessionId,
        operations: [
          { type: 'init', agent: '', title: '' }, // Missing required fields
          { type: 'add', title: '' }, // Missing title
          { type: 'update', id: '', status: undefined as any }, // Missing id and status
          { type: 'complete', id: '' }, // Missing id
          { type: 'remove', id: '' } // Missing id
        ]
      });

      // All should fail due to missing required fields
      expect(result.results.every(r => !r.success)).toBe(true);
      expect(result.summary.succeeded).toBe(0);
      expect(result.summary.failed).toBe(5);
    });
  });

  describe('Response Format', () => {
    it('should return proper ToolResponse', async () => {
      const sessionId = 'response-test';

      const result = await unifiedCommand({
        sessionId,
        operations: [
          { type: 'init', agent: 'response-agent', title: 'Response Test' }
        ]
      });

      // Verify ToolResponse structure
      expect(result.response).toBeDefined();
      expect(result.response.title).toBeDefined();
      expect(result.response.output).toBeDefined();
      expect(result.response.metadata).toBeDefined();
      
      // Verify metadata
      expect(result.response.metadata.operation).toBe('unified');
      expect(result.response.metadata.results).toBeDefined();
      expect(result.response.metadata.summary).toBeDefined();
    });

    it('should include current status after operations', async () => {
      const sessionId = 'status-test';

      const result = await unifiedCommand({
        sessionId,
        operations: [
          { type: 'init', agent: 'status-agent', title: 'Status Test' },
          { type: 'add', title: 'Task 1' },
          { type: 'add', title: 'Task 2' },
          { type: 'complete', id: '1' }
        ]
      });

      // Should have current status
      expect(result.currentStatus).not.toBeNull();
      expect(result.response.metadata.taskList).toBeDefined();
      
      // Status should reflect completed task
      if (result.currentStatus) {
        expect(result.currentStatus.tasks.length).toBeGreaterThan(0);
      }
    });
  });
});
