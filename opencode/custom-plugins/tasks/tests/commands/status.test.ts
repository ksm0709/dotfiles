import { statusCommand, StatusArgs } from '../../src/commands/status';
import { Storage } from '../../src/lib/storage';
import { Parser } from '../../src/lib/parser';
import { Formatter } from '../../src/lib/formatter';

// Mocks
jest.mock('../../src/lib/storage');
jest.mock('../../src/lib/parser');
jest.mock('../../src/lib/formatter');

describe('statusCommand', () => {
  let mockStorage: jest.Mocked<Storage>;
  let mockParser: jest.Mocked<Parser>;
  let mockFormatter: jest.Mocked<Formatter>;

  beforeEach(() => {
    jest.clearAllMocks();

    mockStorage = new Storage() as jest.Mocked<Storage>;
    mockParser = new Parser() as jest.Mocked<Parser>;
    mockFormatter = new Formatter() as jest.Mocked<Formatter>;

    (Storage as jest.MockedClass<typeof Storage>).mockImplementation(() => mockStorage);
    (Parser as jest.MockedClass<typeof Parser>).mockImplementation(() => mockParser);
    (Formatter as jest.MockedClass<typeof Formatter>).mockImplementation(() => mockFormatter);
  });

  it('should show status summary for single task list', async () => {
    const args: StatusArgs = { sessionId: 'test-session' };

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
    mockFormatter.formatStatusSummary.mockReturnValue('Status Output');

    const result = await statusCommand(args);

    expect(mockStorage.listTaskFiles).toHaveBeenCalledWith('test-session');
    expect(mockFormatter.formatStatusSummary).toHaveBeenCalled();
    expect(result.success).toBe(true);
    expect(result.formattedOutput).toContain('Status Output');
  });

  it('should show status for multiple task lists', async () => {
    const args: StatusArgs = { sessionId: 'test-session' };

    const taskList1 = {
      title: 'Project 1',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: [{ id: '1', title: 'Task', status: 'completed' }]
    };

    const taskList2 = {
      title: 'Project 2',
      agent: 'test-agent',
      sessionId: 'test-session',
      createdAt: '2026-01-30',
      tasks: [{ id: '2', title: 'Task', status: 'pending' }]
    };

    mockStorage.listTaskFiles.mockResolvedValue(['project1.md', 'project2.md']);
    mockStorage.readTaskList.mockResolvedValue('content');
    mockParser.parseTaskList
      .mockReturnValueOnce(taskList1 as any)
      .mockReturnValueOnce(taskList2 as any);
    mockFormatter.formatStatusSummary
      .mockReturnValueOnce('Status 1')
      .mockReturnValueOnce('Status 2');

    const result = await statusCommand(args);

    expect(result.success).toBe(true);
    expect(result.summaries).toHaveLength(2);
  });

  it('should handle no task lists found', async () => {
    const args: StatusArgs = { sessionId: 'non-existent-session' };

    mockStorage.listTaskFiles.mockResolvedValue([]);

    const result = await statusCommand(args);

    expect(result.success).toBe(true);
    expect(result.message).toContain('No task lists found');
  });
});
