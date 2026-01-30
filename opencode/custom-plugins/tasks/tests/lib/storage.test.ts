import { Storage } from '../../src/lib/storage';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

describe('Storage', () => {
  let storage: Storage;
  let tempDir: string;
  let originalHome: string | undefined;

  beforeEach(async () => {
    // Create temporary directory for testing
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-test-'));
    
    // Mock HOME environment variable
    originalHome = process.env.HOME;
    process.env.HOME = tempDir;
    
    // Create storage instance - this now reads HOME at construction time
    storage = new Storage();
  });

  afterEach(async () => {
    // Restore original HOME
    if (originalHome !== undefined) {
      process.env.HOME = originalHome;
    }
    
    // Clean up temp directory
    try {
      await fs.rm(tempDir, { recursive: true, force: true });
    } catch (error) {
      // Ignore cleanup errors
    }
  });

  describe('ensureAgentDir', () => {
    it('should create agent directory if it does not exist', async () => {
      const agentDir = await storage.ensureAgentDir('test-agent');
      
      const expectedDir = path.join(tempDir, '.config', 'opencode', 'tasks', 'test-agent');
      expect(agentDir).toBe(expectedDir);
      
      // Verify directory was created
      const stats = await fs.stat(agentDir);
      expect(stats.isDirectory()).toBe(true);
    });

    it('should return existing directory without error', async () => {
      // Create directory first
      const firstDir = await storage.ensureAgentDir('test-agent');
      
      // Call again - should not throw
      const secondDir = await storage.ensureAgentDir('test-agent');
      
      expect(secondDir).toBe(firstDir);
    });
  });

  describe('saveTaskList and readTaskList', () => {
    it('should save and read task list content', async () => {
      const agent = 'test-agent';
      const title = 'Test Task List';
      const content = '# Task List: Test\n\n**에이전트**: test-agent';
      
      await storage.saveTaskList(agent, title, content);
      
      const readContent = await storage.readTaskList(agent, title);
      expect(readContent).toBe(content);
    });

    it('should sanitize file names with special characters', async () => {
      const agent = 'test-agent';
      const title = 'Test @#$%^&*() Task !!!';
      const content = 'Test content';
      
      await storage.saveTaskList(agent, title, content);
      
      // Should be readable with original title
      const readContent = await storage.readTaskList(agent, title);
      expect(readContent).toBe(content);
    });

    it('should handle Korean characters in file names', async () => {
      const agent = 'test-agent';
      const title = '한국어 테스트';
      const content = 'Test content';
      
      await storage.saveTaskList(agent, title, content);
      
      const readContent = await storage.readTaskList(agent, title);
      expect(readContent).toBe(content);
    });

    it('should return null for non-existent file', async () => {
      const content = await storage.readTaskList('non-existent', 'non-existent');
      expect(content).toBeNull();
    });

    it('should truncate long file names', async () => {
      const agent = 'test-agent';
      const title = 'a'.repeat(100);
      const content = 'Test content';
      
      await storage.saveTaskList(agent, title, content);
      
      const readContent = await storage.readTaskList(agent, title);
      expect(readContent).toBe(content);
    });

    it('should throw error if read fails with non-ENOENT error', async () => {
      // Create a directory with the same name as the file to cause EISDIR error
      const agent = 'test-agent-read-error';
      const title = 'test-title';
      
      await storage.ensureAgentDir(agent);
      
      // Create a directory instead of a file
      const agentDir = path.join(tempDir, '.config', 'opencode', 'tasks', agent);
      const fileName = title.toLowerCase().replace(/[^a-z0-9가-힣\s-]/g, '').replace(/\s+/g, '-').substring(0, 50) + '.md';
      await fs.mkdir(path.join(agentDir, fileName));
      
      await expect(storage.readTaskList(agent, title)).rejects.toThrow();
    });
  });

  describe('listTaskFiles', () => {
    it('should return empty array for non-existent agent', async () => {
      const files = await storage.listTaskFiles('non-existent-agent');
      expect(files).toEqual([]);
    });

    it('should list only markdown files', async () => {
      const agent = 'test-agent-md-files';
      
      // Create task lists
      await storage.saveTaskList(agent, 'Task 1', 'Content 1');
      await storage.saveTaskList(agent, 'Task 2', 'Content 2');
      
      // Create a non-markdown file directly
      const agentDir = path.join(tempDir, '.config', 'opencode', 'tasks', agent);
      await fs.mkdir(agentDir, { recursive: true });
      await fs.writeFile(path.join(agentDir, 'other.txt'), 'not a task');
      
      const files = await storage.listTaskFiles(agent);
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
      const agent = 'test-agent-notdir';
      const baseDir = path.join(tempDir, '.config', 'opencode', 'tasks');
      await fs.mkdir(baseDir, { recursive: true });
      await fs.writeFile(path.join(baseDir, agent), 'not a directory');
      
      await expect(storage.listTaskFiles(agent)).rejects.toThrow();
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
