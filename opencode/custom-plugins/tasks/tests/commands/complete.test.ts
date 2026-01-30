import { completeCommand, CompleteArgs } from '../../src/commands/complete';
import { updateCommand } from '../../src/commands/update';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

// Mocks
jest.mock('../../src/commands/update');

describe('completeCommand', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let mockUpdateCommand: jest.MockedFunction<typeof updateCommand>;

  beforeEach(async () => {
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-complete-test-'));
    originalHome = process.env.HOME;
    process.env.HOME = tempDir;

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
  });

  it('should mark task as completed', async () => {
    const args: CompleteArgs = {
      sessionId: 'test-session',
      id: '1'
    };

    mockUpdateCommand.mockResolvedValue({
      success: true,
      taskId: '1',
      status: 'completed',
      message: 'Task 1 status updated to: completed'
    });

    const result = await completeCommand(args);

    expect(mockUpdateCommand).toHaveBeenCalledWith({
      sessionId: 'test-session',
      id: '1',
      status: 'completed'
    });
    expect(result.success).toBe(true);
    expect(result.message).toContain('completed');
  });

  it('should handle error from updateCommand', async () => {
    const args: CompleteArgs = {
      sessionId: 'test-session',
      id: '1'
    };

    mockUpdateCommand.mockRejectedValue(new Error('Update failed'));

    await expect(completeCommand(args)).rejects.toThrow('Update failed');
  });

  it('should handle non-existent task', async () => {
    const args: CompleteArgs = {
      sessionId: 'test-session',
      id: '999'
    };

    mockUpdateCommand.mockResolvedValue({
      success: false,
      taskId: '999',
      status: 'completed',
      message: 'Task 999 not found'
    });

    await expect(completeCommand(args)).rejects.toThrow('Task 999 not found');
  });
});
