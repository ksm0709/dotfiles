import { listCommand, ListArgs } from '../../src/commands/list';
import { Storage } from '../../src/lib/storage';
import { Parser } from '../../src/lib/parser';
import { Formatter } from '../../src/lib/formatter';

// Mocks
jest.mock('../../src/lib/storage');
jest.mock('../../src/lib/parser');
jest.mock('../../src/lib/formatter');

describe('listCommand', () => {
  let mockStorage: jest.Mocked<Storage>;
  let mockParser: jest.Mocked<Parser>;
  let mockFormatter: jest.Mocked<Formatter>;

  beforeEach(() => {
    jest.clearAllMocks();

    // Setup mocks
    mockStorage = new Storage() as jest.Mocked<Storage>;
    mockParser = new Parser() as jest.Mocked<Parser>;
    mockFormatter = new Formatter() as jest.Mocked<Formatter>;

    (Storage as jest.MockedClass<typeof Storage>).mockImplementation(() => mockStorage);
    (Parser as jest.MockedClass<typeof Parser>).mockImplementation(() => mockParser);
    (Formatter as jest.MockedClass<typeof Formatter>).mockImplementation(() => mockFormatter);
  });

  it('should list tasks in markdown format by default', async () => {
    const args: ListArgs = { sessionId: 'test-session' };
    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: []
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockFormatter.formatAsMarkdown.mockReturnValue('Formatted Markdown');

    const result = await listCommand(args);

    expect(mockStorage.listTaskFiles).toHaveBeenCalledWith('test-session');
    expect(mockParser.parseTaskList).toHaveBeenCalledWith('content');
    expect(mockFormatter.formatAsMarkdown).toHaveBeenCalledWith(taskList);
    expect(result.success).toBe(true);
    expect(result.formattedOutput).toContain('Formatted Markdown');
  });

  it('should list tasks in json format', async () => {
    const args: ListArgs = { sessionId: 'test-session', format: 'json' };
    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: []
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockFormatter.formatAsJSON.mockReturnValue('{"tasks":[]}');

    const result = await listCommand(args);

    expect(mockFormatter.formatAsJSON).toHaveBeenCalledWith(taskList);
    expect(result.formattedOutput).toContain('{"tasks":[]}');
  });

  it('should list tasks in table format', async () => {
    const args: ListArgs = { sessionId: 'test-session', format: 'table' };
    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: []
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockFormatter.formatAsTable.mockReturnValue('Table Output');

    const result = await listCommand(args);

    expect(mockFormatter.formatAsTable).toHaveBeenCalledWith(taskList);
    expect(result.formattedOutput).toContain('Table Output');
  });

  it('should handle no task lists found', async () => {
    const args: ListArgs = { sessionId: 'non-existent-session' };

    mockStorage.listTaskFiles.mockResolvedValue([]);

    const result = await listCommand(args);

    expect(result.success).toBe(true);
    expect(result.message).toContain('No task lists found');
  });

  it('should handle read error gracefully', async () => {
    const args: ListArgs = { sessionId: 'test-session' };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue(null);

    const result = await listCommand(args);

    // Should continue without throwing
    expect(result.success).toBe(true);
    expect(result.taskLists).toEqual([]);
  });

  it('should list multiple task lists', async () => {
    const args: ListArgs = { sessionId: 'test-session' };
    const taskList1 = {
      title: 'Project 1',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: []
    };
    const taskList2 = {
      title: 'Project 2',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: []
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project1.md', 'project2.md']);
    mockStorage.readTaskList
      .mockResolvedValueOnce('content1')
      .mockResolvedValueOnce('content2');
    mockParser.parseTaskList
      .mockReturnValueOnce(taskList1 as any)
      .mockReturnValueOnce(taskList2 as any);
    mockFormatter.formatAsMarkdown
      .mockReturnValueOnce('Project 1 Output')
      .mockReturnValueOnce('Project 2 Output');

    const result = await listCommand(args);

    expect(mockStorage.readTaskList).toHaveBeenCalledTimes(2);
    expect(result.success).toBe(true);
    expect(result.taskLists).toHaveLength(2);
  });

  it('should return ToolResponse with title, output, and metadata', async () => {
    const args: ListArgs = { sessionId: 'test-session' };
    const taskList = {
      title: 'Test Project',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: [
        { id: '1', title: 'Task One', status: 'completed' },
        { id: '2', title: 'Task Two', status: 'pending' }
      ]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList.mockReturnValue(taskList as any);
    mockFormatter.formatAsMarkdown.mockReturnValue('# Task List');

    const result = await listCommand(args);

    // Verify ToolResponse structure
    expect(result.response).toBeDefined();
    expect(result.response.title).toBeDefined();
    expect(result.response.output).toBeDefined();
    expect(result.response.metadata).toBeDefined();
    expect(result.response.metadata.operation).toBe('list');
    expect(result.response.metadata.taskList).toBeDefined();
    expect(result.response.metadata.taskList!.title).toBe('Test Project');
    expect(result.response.metadata.tasks).toBeDefined();
    expect(result.response.metadata.tasks).toHaveLength(2);
    expect(result.response.metadata.summary).toBeDefined();
    // Multiple task lists are in result.taskLists, not in response.metadata
    expect(result.taskLists).toHaveLength(1);
  });
});
