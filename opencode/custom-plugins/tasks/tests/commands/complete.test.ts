import { completeCommand, CompleteArgs } from '../../src/commands/complete';
import { updateCommand } from '../../src/commands/update';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

// Mocks
const mockConsoleLog = jest.spyOn(console, 'log').mockImplementation();
const mockConsoleError = jest.spyOn(console, 'error').mockImplementation();

jest.mock('../../src/commands/update');

describe('completeCommand', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let mockUpdateCommand: jest.MockedFunction<typeof updateCommand>;

  beforeEach(async () => {
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-complete-test-'));
    originalHome = process.env.HOME;
    process.env.HOME = tempDir;

    mockConsoleLog.mockClear();
    mockConsoleError.mockClear();
    jest.clearAllMocks();

    mockUpdateCommand = updateCommand as jest.MockedFunction<typeof updateCommand>;
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

  it('should mark task as completed', async () => {
    const args: CompleteArgs = {
      agent: 'test-agent',
      id: '1'
    };

    mockUpdateCommand.mockResolvedValue(undefined);

    await completeCommand(args);

    expect(mockUpdateCommand).toHaveBeenCalledWith({
      agent: 'test-agent',
      id: '1',
      status: 'completed'
    });
    expect(mockConsoleLog).toHaveBeenCalledWith(
      expect.stringContaining('completed')
    );
  });

  it('should handle error from updateCommand', async () => {
    const args: CompleteArgs = {
      agent: 'test-agent',
      id: '1'
    };

    mockUpdateCommand.mockRejectedValue(new Error('Update failed'));

    await expect(completeCommand(args)).rejects.toThrow('Update failed');
  });

  it('should handle non-existent task', async () => {
    const args: CompleteArgs = {
      agent: 'test-agent',
      id: '999'
    };

    // Simulate updateCommand failing
    mockUpdateCommand.mockImplementation(() => {
      console.error('Task not found');
      return Promise.resolve();
    });

    await completeCommand(args);

    expect(mockUpdateCommand).toHaveBeenCalledWith({
      agent: 'test-agent',
      id: '999',
      status: 'completed'
    });
  });
});
