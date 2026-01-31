import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';
import { execSync } from 'child_process';

describe('Tasks Plugin Installation and Execution Integration', () => {
  let tempDir: string;
  let opencodeDir: string;
  let installScriptPath: string;
  const sourceDir = path.join(__dirname, '../..');

  beforeAll(async () => {
    // Get the install.sh script path
    installScriptPath = path.join(sourceDir, 'install.sh');
    
    // Verify install.sh exists
    try {
      await fs.access(installScriptPath);
    } catch {
      throw new Error(`install.sh not found at ${installScriptPath}`);
    }
  });

  beforeEach(async () => {
    // Create temporary directory for isolated testing
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-integration-test-'));
    opencodeDir = path.join(tempDir, '.opencode');
    
    console.log(`Created test directory: ${tempDir}`);
  });

  afterEach(async () => {
    // Clean up temp directory
    try {
      await fs.rm(tempDir, { recursive: true, force: true });
      console.log(`Cleaned up test directory: ${tempDir}`);
    } catch (error) {
      console.warn(`Failed to clean up ${tempDir}:`, error);
    }
  });

  describe('Step 1: Isolated Environment Setup', () => {
    it('should create clean test directory', async () => {
      const stats = await fs.stat(tempDir);
      expect(stats.isDirectory()).toBe(true);
      
      // Directory should be empty initially
      const files = await fs.readdir(tempDir);
      expect(files).toHaveLength(0);
    });
  });

  describe('Step 2: Plugin Installation', () => {
    it('should install plugin to target directory using install.sh', async () => {
      // Execute install.sh with --target option
      const result = execSync(
        `bash "${installScriptPath}" --target "${tempDir}"`,
        {
          encoding: 'utf-8',
          timeout: 30000,
          cwd: sourceDir
        }
      );

      // Verify installation output
      expect(result).toContain('Installing tasks plugin to custom location');
      expect(result).toContain('Isolated test mode');
      expect(result).toContain('Installation complete');
    });

    it('should create correct directory structure', async () => {
      // Run installation
      execSync(
        `bash "${installScriptPath}" --target "${tempDir}"`,
        {
          encoding: 'utf-8',
          timeout: 30000,
          cwd: sourceDir
        }
      );

      // Verify .opencode directory exists
      const opencodeStats = await fs.stat(opencodeDir);
      expect(opencodeStats.isDirectory()).toBe(true);

      // Verify required subdirectories
      const requiredDirs = [
        'plugins',
        'plugins/tasks',
        'plugins/tasks/commands',
        'plugins/tasks/lib',
        'plugins/tasks/types',
        'shared/tasks/docs'
      ];

      for (const dir of requiredDirs) {
        const dirPath = path.join(opencodeDir, dir);
        const stats = await fs.stat(dirPath);
        expect(stats.isDirectory()).toBe(true);
      }
    });

    it('should copy plugin files correctly', async () => {
      // Run installation
      execSync(
        `bash "${installScriptPath}" --target "${tempDir}"`,
        {
          encoding: 'utf-8',
          timeout: 30000,
          cwd: sourceDir
        }
      );

      // Verify main plugin file exists
      const pluginFile = path.join(opencodeDir, 'plugins/tasks/index.ts');
      const pluginStats = await fs.stat(pluginFile);
      expect(pluginStats.isFile()).toBe(true);

      // Verify plugin content
      const content = await fs.readFile(pluginFile, 'utf-8');
      expect(content).toContain('export const TasksPlugin: Plugin');
      expect(content).toContain('import { tool } from "@opencode-ai/plugin"');
      
      // Verify import paths are corrected (commands are imported from ./commands/)
      expect(content).toContain("from './commands/");
    });

    it('should copy all source files to tasks subdirectory', async () => {
      // Run installation
      execSync(
        `bash "${installScriptPath}" --target "${tempDir}"`,
        {
          encoding: 'utf-8',
          timeout: 30000,
          cwd: sourceDir
        }
      );

      // Verify source directories
      const tasksDir = path.join(opencodeDir, 'plugins/tasks');
      
      const commandsDir = path.join(tasksDir, 'commands');
      const libDir = path.join(tasksDir, 'lib');
      const typesDir = path.join(tasksDir, 'types');

      const [commandsStats, libStats, typesStats] = await Promise.all([
        fs.stat(commandsDir),
        fs.stat(libDir),
        fs.stat(typesDir)
      ]);

      expect(commandsStats.isDirectory()).toBe(true);
      expect(libStats.isDirectory()).toBe(true);
      expect(typesStats.isDirectory()).toBe(true);

      // Verify some key files exist
      const keyFiles = [
        path.join(commandsDir, 'init.ts'),
        path.join(libDir, 'storage.ts'),
        path.join(libDir, 'parser.ts'),
        path.join(typesDir, 'index.ts')
      ];

      for (const file of keyFiles) {
        const stats = await fs.stat(file);
        expect(stats.isFile()).toBe(true);
      }
    });
  });

  describe('Step 3: Plugin Structure Validation', () => {
    beforeEach(async () => {
      // Run installation before each test
      execSync(
        `bash "${installScriptPath}" --target "${tempDir}"`,
        {
          encoding: 'utf-8',
          timeout: 30000,
          cwd: sourceDir
        }
      );
    });

    it('should have valid TypeScript syntax in plugin file', async () => {
      const pluginFile = path.join(opencodeDir, 'plugins/tasks/index.ts');
      const content = await fs.readFile(pluginFile, 'utf-8');

      // Check for valid plugin structure
      expect(content).toMatch(/export\s+const\s+TasksPlugin\s*:\s*Plugin/);
      expect(content).toMatch(/export\s+default\s+TasksPlugin/);
      expect(content).toMatch(/return\s*\{/);
      expect(content).toMatch(/tool\s*:/);
      
      // Check all tools are defined
      const expectedTools = [
        'tasks_init',
        'tasks_list',
        'tasks_update',
        'tasks_complete',
        'tasks_add',
        'tasks_remove',
        'tasks_status'
      ];

      for (const toolName of expectedTools) {
        expect(content).toContain(toolName);
      }
    });

    it('should not have manifest.json (which caused infinite loop)', async () => {
      const manifestFile = path.join(opencodeDir, 'plugins/tasks-manifest.json');
      
      // Manifest should not exist
      await expect(fs.access(manifestFile)).rejects.toThrow();
    });

    it('should have correct import paths in plugin file', async () => {
      const pluginFile = path.join(opencodeDir, 'plugins/tasks/index.ts');
      const content = await fs.readFile(pluginFile, 'utf-8');

      // All imports should use direct subdirectory paths (./commands/, ./lib/, ./types/)
      const importMatches = content.match(/from\s+['"]\.\/[^'"]+['"]/g) || [];
      
      for (const importPath of importMatches) {
        expect(importPath).toMatch(/from\s+['"]\.\/(commands|lib|types)\//);
      }
    });
  });

  describe('Step 4: Documentation and Templates', () => {
    beforeEach(async () => {
      execSync(
        `bash "${installScriptPath}" --target "${tempDir}"`,
        {
          encoding: 'utf-8',
          timeout: 30000,
          cwd: sourceDir
        }
      );
    });

    it('should copy documentation files', async () => {
      const docsDir = path.join(opencodeDir, 'shared/tasks/docs');
      const guideFile = path.join(docsDir, 'tasks-tools-guide.md');
      
      const stats = await fs.stat(guideFile);
      expect(stats.isFile()).toBe(true);

      const content = await fs.readFile(guideFile, 'utf-8');
      expect(content).toContain('Tasks Plugin Tools Guide');
      expect(content).toContain('tasks_*');
    });
  });

  describe('Full Integration Flow', () => {
    it('should complete full installation flow without errors', async () => {
      // Step 1: Create isolated environment
      const testStartTime = Date.now();
      
      // Step 2: Run installation
      let installOutput: string;
      try {
        installOutput = execSync(
          `bash "${installScriptPath}" --target "${tempDir}"`,
          {
            encoding: 'utf-8',
            timeout: 30000,
            cwd: sourceDir
          }
        );
      } catch (error: any) {
        console.error('Installation failed:', error.stderr || error.message);
        throw error;
      }

      const installDuration = Date.now() - testStartTime;
      console.log(`Installation completed in ${installDuration}ms`);

      // Step 3: Verify all components
      const checks = [
        // Main plugin file
        fs.access(path.join(opencodeDir, 'plugins/tasks/index.ts')),
        // Source directories
        fs.access(path.join(opencodeDir, 'plugins/tasks/commands')),
        fs.access(path.join(opencodeDir, 'plugins/tasks/lib')),
        fs.access(path.join(opencodeDir, 'plugins/tasks/types')),
        // Documentation
        fs.access(path.join(opencodeDir, 'shared/tasks/docs/tasks-tools-guide.md'))
      ];

      await Promise.all(checks);

      // Step 4: Verify plugin content
      const pluginContent = await fs.readFile(
        path.join(opencodeDir, 'plugins/tasks/index.ts'),
        'utf-8'
      );

      // Should be valid plugin structure
      expect(pluginContent).toContain('Plugin');
      expect(pluginContent).toContain('tool:');
      expect(installOutput).toContain('✅ Installation complete');

      console.log('✅ Full integration test passed');
    });

    it('should handle concurrent installations to different directories', async () => {
      const tempDir2 = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-integration-test-2-'));
      
      try {
        // Run two installations concurrently
        const install1 = execSync(
          `bash "${installScriptPath}" --target "${tempDir}"`,
          {
            encoding: 'utf-8',
            timeout: 30000,
            cwd: sourceDir
          }
        );

        const install2 = execSync(
          `bash "${installScriptPath}" --target "${tempDir2}"`,
          {
            encoding: 'utf-8',
            timeout: 30000,
            cwd: sourceDir
          }
        );

        // Both should succeed
        expect(install1).toContain('Installation complete');
        expect(install2).toContain('Installation complete');

        // Both directories should have the plugin
        const [plugin1, plugin2] = await Promise.all([
          fs.readFile(path.join(tempDir, '.opencode/plugins/tasks/index.ts'), 'utf-8'),
          fs.readFile(path.join(tempDir2, '.opencode/plugins/tasks/index.ts'), 'utf-8')
        ]);

        expect(plugin1).toContain('TasksPlugin');
        expect(plugin2).toContain('TasksPlugin');

      } finally {
        await fs.rm(tempDir2, { recursive: true, force: true });
      }
    });
  });

  describe('Error Handling', () => {
    it('should skip AGENTS.md update in isolated mode', async () => {
      // Run installation
      const output = execSync(
        `bash "${installScriptPath}" --target "${tempDir}"`,
        {
          encoding: 'utf-8',
          timeout: 30000,
          cwd: sourceDir
        }
      );

      // Should indicate isolated mode
      expect(output).toContain('Isolated test mode');
      expect(output).toContain('Skipping AGENTS.md update');
      
      // AGENTS.md should not be created in isolated mode
      const agentsMdPath = path.join(opencodeDir, 'AGENTS.md');
      await expect(fs.access(agentsMdPath)).rejects.toThrow();
    });

    it('should skip package.json update in isolated mode', async () => {
      // Run installation
      const output = execSync(
        `bash "${installScriptPath}" --target "${tempDir}"`,
        {
          encoding: 'utf-8',
          timeout: 30000,
          cwd: sourceDir
        }
      );

      // Should indicate skipping package.json
      expect(output).toContain('Skipping package.json update');
    });
  });
});
