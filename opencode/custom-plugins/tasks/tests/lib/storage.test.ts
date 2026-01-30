import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

// Storage will be imported dynamically to handle env var changes
describe('Storage', () => {
  let Storage: typeof import('../../src/lib/storage').Storage;
  let storage: InstanceType<typeof Storage>;
  let tempDir: string;
  let originalHome: string | undefined;
  let originalXdgDataHome: string | undefined;

  beforeEach(async () => {
    // Create temporary directory for testing
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-test-'));
    
    // Mock environment variables BEFORE importing Storage
    originalHome = process.env.HOME;
    originalXdgDataHome = process.env.XDG_DATA_HOME;
    delete process.env.XDG_DATA_HOME;  // Force fallback to HOME/.local/share
    process.env.HOME = tempDir;
    
    // Clear module cache and re-import to pick up new env vars
    jest.resetModules();
    const storageModule = await import('../../src/lib/storage');
    Storage = storageModule.Storage;
    
    // Create storage instance
    storage = new Storage();
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
    
    // Clean up temp directory
    try {
      await fs.rm(tempDir, { recursive: true, force: true });
    } catch (error) {
      // Ignore cleanup errors
    }
  });

  describe('ensureSessionDir', () => {
    it('should create session directory if it does not exist', async () => {
      const sessionDir = await storage.ensureSessionDir('test-session');
      
      const expectedDir = path.join(tempDir, '.local', 'share', 'opencode', 'tasks', 'test-session');
      expect(sessionDir).toBe(expectedDir);
      
      // Verify directory was created
      const stats = await fs.stat(sessionDir);
      expect(stats.isDirectory()).toBe(true);
    });

    it('should return existing directory without error', async () => {
      // Create directory first
      const firstDir = await storage.ensureSessionDir('test-session');
      
      // Call again - should not throw
      const secondDir = await storage.ensureSessionDir('test-session');
      
      expect(secondDir).toBe(firstDir);
    });
  });

  describe('saveTaskList and readTaskList', () => {
    it('should save and read task list content', async () => {
      const sessionId = 'test-session';
      const title = 'Test Task List';
      const content = '# Task List: Test\n\n**세션**: test-session';
      
      await storage.saveTaskList(sessionId, title, content);
      
      const readContent = await storage.readTaskList(sessionId, title);
      expect(readContent).toBe(content);
    });

    it('should sanitize file names with special characters', async () => {
      const sessionId = 'test-session';
      const title = 'Test @#$%^&*() Task !!!';
      const content = 'Test content';
      
      await storage.saveTaskList(sessionId, title, content);
      
      // Should be readable with original title
      const readContent = await storage.readTaskList(sessionId, title);
      expect(readContent).toBe(content);
    });

    it('should handle Korean characters in file names', async () => {
      const sessionId = 'test-session';
      const title = '한국어 테스트';
      const content = 'Test content';
      
      await storage.saveTaskList(sessionId, title, content);
      
      const readContent = await storage.readTaskList(sessionId, title);
      expect(readContent).toBe(content);
    });

    it('should return null for non-existent file', async () => {
      const content = await storage.readTaskList('non-existent', 'non-existent');
      expect(content).toBeNull();
    });

    it('should truncate long file names', async () => {
      const sessionId = 'test-session';
      const title = 'a'.repeat(100);
      const content = 'Test content';
      
      await storage.saveTaskList(sessionId, title, content);
      
      const readContent = await storage.readTaskList(sessionId, title);
      expect(readContent).toBe(content);
    });

    it('should throw error if read fails with non-ENOENT error', async () => {
      // Create a directory with the same name as the file to cause EISDIR error
      const sessionId = 'test-session-read-error';
      const title = 'test-title';
      
      await storage.ensureSessionDir(sessionId);
      
      // Create a directory instead of a file
      const sessionDir = path.join(tempDir, '.local', 'share', 'opencode', 'tasks', sessionId);
      const fileName = title.toLowerCase().replace(/[^a-z0-9가-힣\s-]/g, '').replace(/\s+/g, '-').substring(0, 50) + '.md';
      await fs.mkdir(path.join(sessionDir, fileName));
      
      await expect(storage.readTaskList(sessionId, title)).rejects.toThrow();
    });
  });

  describe('listTaskFiles', () => {
    it('should return empty array for non-existent session', async () => {
      const files = await storage.listTaskFiles('non-existent-session');
      expect(files).toEqual([]);
    });

    it('should list only markdown files', async () => {
      const sessionId = 'test-session-md-files';
      
      // Create task lists
      await storage.saveTaskList(sessionId, 'Task 1', 'Content 1');
      await storage.saveTaskList(sessionId, 'Task 2', 'Content 2');
      
      // Create a non-markdown file directly
      const sessionDir = path.join(tempDir, '.local', 'share', 'opencode', 'tasks', sessionId);
      await fs.mkdir(sessionDir, { recursive: true });
      await fs.writeFile(path.join(sessionDir, 'other.txt'), 'not a task');
      
      const files = await storage.listTaskFiles(sessionId);
      expect(files).toHaveLength(2);
      expect(files.every((f: string) => f.endsWith('.md'))).toBe(true);
    });

    it('should return sorted list of files', async () => {
      const agent = 'test-agent-sorted';
      
      await storage.saveTaskList(agent, 'Zebra', 'Content');
      await storage.saveTaskList(agent, 'Apple', 'Content');
      await storage.saveTaskList(agent, 'Mango', 'Content');
      
      const files = await storage.listTaskFiles(agent);
      expect(files).toEqual(['apple.md', 'mango.md', 'zebra.md']);
    });

    it('should throw error if readdir fails with non-ENOENT error', async () => {
      // Create a file instead of a directory to cause ENOTDIR error
      const sessionId = 'test-session-notdir';
      const baseDir = path.join(tempDir, '.local', 'share', 'opencode', 'tasks');
      await fs.mkdir(baseDir, { recursive: true });
      await fs.writeFile(path.join(baseDir, sessionId), 'not a directory');
      
      await expect(storage.listTaskFiles(sessionId)).rejects.toThrow();
    });
  });

  describe('deleteTaskList', () => {
    it('should delete existing task list', async () => {
      const agent = 'test-agent-delete';
      const title = 'To Delete';
      
      await storage.saveTaskList(agent, title, 'Content');
      
      // Verify it exists
      let content = await storage.readTaskList(agent, title);
      expect(content).not.toBeNull();
      
      // Delete it
      await storage.deleteTaskList(agent, title);
      
      // Verify it's gone
      content = await storage.readTaskList(agent, title);
      expect(content).toBeNull();
    });

    it('should not throw when deleting non-existent file', async () => {
      await expect(
        storage.deleteTaskList('non-existent', 'non-existent')
      ).resolves.not.toThrow();
    });
  });
});
