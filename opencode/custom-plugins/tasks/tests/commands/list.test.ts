import { listCommand, ListArgs } from '../../src/commands/list';
import { Storage } from '../../src/lib/storage';
import { Parser } from '../../src/lib/parser';
import { Formatter } from '../../src/lib/formatter';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

// Mocks
const mockConsoleLog = jest.spyOn(console, 'log').mockImplementation();
const mockConsoleError = jest.spyOn(console, 'error').mockImplementation();

jest.mock('../../src/lib/storage');
jest.mock('../../src/lib/parser');
jest.mock('../../src/lib/formatter');

describe('listCommand', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let mockStorage: jest.Mocked<Storage>;
  let mockParser: jest.Mocked<Parser>;
  let mockFormatter: jest.Mocked<Formatter>;

  beforeEach(async () => {
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-list-test-'));
    originalHome = process.env.HOME;
    process.env.HOME = tempDir;

    mockConsoleLog.mockClear();
    mockConsoleError.mockClear();
    jest.clearAllMocks();

    // Setup mocks
    mockStorage = new Storage() as jest.Mocked<Storage>;
    mockParser = new Parser() as jest.Mocked<Parser>;
    mockFormatter = new Formatter() as jest.Mocked<Formatter>;

    (Storage as jest.MockedClass<typeof Storage>).mockImplementation(() => mockStorage);
    (Parser as jest.MockedClass<typeof Parser>).mockImplementation(() => mockParser);
    (Formatter as jest.MockedClass<typeof Formatter>).mockImplementation(() => mockFormatter);
  });

  afterEach(async () => {
    if (originalHome !== undefined) {
      process.env.HOME = originalHome;
    }
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

  it('should list tasks in markdown format by default', async () => {
    const args: ListArgs = { agent: 'test-agent' };
    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      createdAt: '2026-01-30',
      sessionId: 'abc',
      tasks: []
    };
    const markdownContent = '# Task List';

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue(markdownContent);
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockFormatter.formatAsMarkdown.mockReturnValue('Formatted Markdown');

    await listCommand(args);

    expect(mockStorage.listTaskFiles).toHaveBeenCalledWith('test-agent');
    expect(mockParser.parseTaskList).toHaveBeenCalledWith(markdownContent);
    expect(mockFormatter.formatAsMarkdown).toHaveBeenCalledWith(taskList);
    expect(mockConsoleLog).toHaveBeenCalledWith('Formatted Markdown');
  });

  it('should list tasks in JSON format', async () => {
    const args: ListArgs = { agent: 'test-agent', format: 'json' };
    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      createdAt: '2026-01-30',
      sessionId: 'abc',
      tasks: []
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockFormatter.formatAsJSON.mockReturnValue('{"title":"Test"}');

    await listCommand(args);

    expect(mockFormatter.formatAsJSON).toHaveBeenCalledWith(taskList);
    expect(mockConsoleLog).toHaveBeenCalledWith('{"title":"Test"}');
  });

  it('should list tasks in table format', async () => {
    const args: ListArgs = { agent: 'test-agent', format: 'table' };
    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      createdAt: '2026-01-30',
      sessionId: 'abc',
      tasks: []
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockFormatter.formatAsTable.mockReturnValue('Table Output');

    await listCommand(args);

    expect(mockFormatter.formatAsTable).toHaveBeenCalledWith(taskList);
    expect(mockConsoleLog).toHaveBeenCalledWith('Table Output');
  });

  it('should handle no task lists found', async () => {
    const args: ListArgs = { agent: 'non-existent' };

    mockStorage.listTaskFiles.mockResolvedValue([]);

    await listCommand(args);

    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('No task lists found')
    );
  });

  it('should handle multiple task lists', async () => {
    const args: ListArgs = { agent: 'test-agent' };

    mockStorage.listTaskFiles.mockResolvedValue(['project1.md', 'project2.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue({
      title: 'Test',
      agent: 'test-agent',
      createdAt: '2026-01-30',
      sessionId: 'abc',
      tasks: []
    } as any);
    mockFormatter.formatAsMarkdown.mockReturnValue('Formatted');

    await listCommand(args);

    expect(mockStorage.readTaskList).toHaveBeenCalledTimes(2);
    expect(mockConsoleLog).toHaveBeenCalledTimes(4); // 2 formatted outputs + 2 separators
  });

  it('should skip files that cannot be read', async () => {
    const args: ListArgs = { agent: 'test-agent' };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue(null);

    await listCommand(args);

    expect(mockParser.parseTaskList).not.toHaveBeenCalled();
  });
});
