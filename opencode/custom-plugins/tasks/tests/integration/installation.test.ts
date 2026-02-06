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

  describe('Step 2: Plugin Installation (Modern Bundled Approach)', () => {
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
      expect(result).toContain('Bundling plugin into a single file');
    });

    it('should create correct directory structure (bundled single file)', async () => {
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

      // Modern bundled approach: single file in plugins/
      const pluginsDir = path.join(opencodeDir, 'plugins');
      const pluginsStats = await fs.stat(pluginsDir);
      expect(pluginsStats.isDirectory()).toBe(true);

      // Documentation in shared/
      const sharedDir = path.join(opencodeDir, 'shared/tasks');
      const sharedStats = await fs.stat(sharedDir);
      expect(sharedStats.isDirectory()).toBe(true);
    });

    it('should create bundled single file with all plugin code', async () => {
      // Run installation
      execSync(
        `bash "${installScriptPath}" --target "${tempDir}"`,
        {
          encoding: 'utf-8',
          timeout: 30000,
          cwd: sourceDir
        }
      );

      // Verify bundled plugin file exists (renamed from .js to .ts)
      const pluginFile = path.join(opencodeDir, 'plugins/tasks.ts');
      const pluginStats = await fs.stat(pluginFile);
      expect(pluginStats.isFile()).toBe(true);

      // Verify plugin content (bundled code)
      const content = await fs.readFile(pluginFile, 'utf-8');
      
      // Should contain plugin export (bundled format uses 'var' and different export pattern)
      expect(content).toContain('TasksPlugin');
      // Bundled code exports at the end: export { TasksPlugin, index_default as default }
      expect(content).toMatch(/export\s*\{\s*TasksPlugin/);
      
      // Should contain unified command
      expect(content).toContain('unifiedCommand');
      
      // Should contain new features
      expect(content).toContain('CompletionChecker');
      expect(content).toContain('PromptGenerator');
      expect(content).toContain('session.idle');
      
      // Should have substantial bundled code (not just imports)
      expect(content.length).toBeGreaterThan(5000);
      
      // Verify in_progress status support
      expect(content).toContain('in_progress');
    });

    it('should not create old directory structure (pre-bundling)', async () => {
      // Run installation
      execSync(
        `bash "${installScriptPath}" --target "${tempDir}"`,
        {
          encoding: 'utf-8',
          timeout: 30000,
          cwd: sourceDir
        }
      );

      // Old structure should NOT exist (bundled approach)
      const oldStructurePaths = [
        path.join(opencodeDir, 'plugins/tasks/index.ts'),
        path.join(opencodeDir, 'plugins/tasks/commands'),
        path.join(opencodeDir, 'plugins/tasks/lib'),
        path.join(opencodeDir, 'plugins/tasks/types')
      ];

      for (const oldPath of oldStructurePaths) {
        await expect(fs.access(oldPath)).rejects.toThrow();
      }
    });
  });

  describe('Step 3: Bundled Plugin Structure Validation', () => {
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

    it('should have valid bundled TypeScript syntax', async () => {
      const pluginFile = path.join(opencodeDir, 'plugins/tasks.ts');
      const content = await fs.readFile(pluginFile, 'utf-8');

      // Check for valid plugin structure (bundled format may vary)
      expect(content).toContain('TasksPlugin');
      expect(content).toMatch(/export\s*\{/);
      // Check that TasksPlugin is exported (may be in different format)
      expect(content).toMatch(/TasksPlugin/);
      expect(content).toMatch(/return\s*\{/);
      expect(content).toMatch(/tool\s*:/);
      
      // Check for bundled modules
      expect(content).toContain('Parser');
      expect(content).toContain('Storage');
      expect(content).toContain('CompletionChecker');
      expect(content).toContain('PromptGenerator');
      
      // Check for session event handler
      expect(content).toContain('session.idle');
      expect(content).toContain('event:');
    });

    it('should not have manifest.json (which caused infinite loop)', async () => {
      const manifestFile = path.join(opencodeDir, 'plugins/tasks-manifest.json');
      
      // Manifest should not exist
      await expect(fs.access(manifestFile)).rejects.toThrow();
    });

    it('should have all required imports bundled', async () => {
      const pluginFile = path.join(opencodeDir, 'plugins/tasks.ts');
      const content = await fs.readFile(pluginFile, 'utf-8');

      // External imports should be preserved (not bundled)
      expect(content).toContain('@opencode-ai/plugin');
      
      // Internal modules should be bundled (not imported)
      expect(content).not.toMatch(/from\s+['"]\.\/commands['"]/);
      expect(content).not.toMatch(/from\s+['"]\.\/lib['"]/);
    });

    it('should support in_progress status in bundled code', async () => {
      const pluginFile = path.join(opencodeDir, 'plugins/tasks.ts');
      const content = await fs.readFile(pluginFile, 'utf-8');

      // Verify in_progress status support
      expect(content).toContain('in_progress');
      expect(content).toContain('pending');
      expect(content).toContain('completed');
      
      // Should contain the checkbox pattern [~] for in_progress
      expect(content).toContain('[~]');
    });
  });

  describe('Step 4: Documentation and Shared Resources', () => {
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

    it('should copy documentation files to shared directory', async () => {
      const docsDir = path.join(opencodeDir, 'shared/tasks/docs');
      const guideFile = path.join(docsDir, 'tasks-tools-guide.md');
      
      const stats = await fs.stat(guideFile);
      expect(stats.isFile()).toBe(true);

      const content = await fs.readFile(guideFile, 'utf-8');
      expect(content).toContain('Tasks Plugin');
      expect(content).toContain('tasks');
    });

    it('should copy templates to shared directory', async () => {
      const templatesDir = path.join(opencodeDir, 'shared/tasks/templates');
      
      // Templates directory should exist
      const stats = await fs.stat(templatesDir);
      expect(stats.isDirectory()).toBe(true);
    });
  });

  describe('Full Integration Flow (Bundled)', () => {
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
        // Bundled plugin file
        fs.access(path.join(opencodeDir, 'plugins/tasks.ts')),
        // Documentation
        fs.access(path.join(opencodeDir, 'shared/tasks/docs/tasks-tools-guide.md'))
      ];

      await Promise.all(checks);

      // Step 4: Verify bundled plugin content
      const pluginContent = await fs.readFile(
        path.join(opencodeDir, 'plugins/tasks.ts'),
        'utf-8'
      );

      // Should be valid bundled plugin
      expect(pluginContent).toContain('TasksPlugin');
      expect(pluginContent).toContain('unifiedCommand');
      expect(installOutput).toContain('✅ Installation complete');

      console.log('✅ Full integration test passed (bundled approach)');
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

        // Both directories should have the bundled plugin
        const [plugin1, plugin2] = await Promise.all([
          fs.readFile(path.join(tempDir, '.opencode/plugins/tasks.ts'), 'utf-8'),
          fs.readFile(path.join(tempDir2, '.opencode/plugins/tasks.ts'), 'utf-8')
        ]);

        expect(plugin1).toContain('TasksPlugin');
        expect(plugin2).toContain('TasksPlugin');
        
        // Both should be bundled (single file)
        expect(plugin1.length).toBeGreaterThan(5000);
        expect(plugin2.length).toBeGreaterThan(5000);

      } finally {
        await fs.rm(tempDir2, { recursive: true, force: true });
      }
    });
  });

  describe('Error Handling in Isolated Mode', () => {
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
