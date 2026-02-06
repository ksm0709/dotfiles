import { PromptGenerator } from '../../src/lib/prompt-generator';
import { TaskDetail, TaskStatus } from '../../src/types';

describe('PromptGenerator - Incomplete Task Prompt Generation (ADDED-002)', () => {
  let promptGenerator: PromptGenerator;

  beforeEach(() => {
    promptGenerator = new PromptGenerator();
  });

  describe('generateIncompleteTaskPrompt', () => {
    it('should generate prompt with incomplete task list (ADDED-002)', () => {
      const incompleteTasks: TaskDetail[] = [
        {
          id: '1',
          title: 'Task in progress',
          status: 'in_progress' as TaskStatus,
          details: ['Detail 1'],
          createdAt: '2026-01-30T10:00:00.000Z',
          updatedAt: '2026-01-30T11:00:00.000Z'
        },
        {
          id: '2',
          title: 'Pending task first',
          status: 'pending' as TaskStatus,
          details: [],
          createdAt: '2026-01-30T10:00:00.000Z',
          updatedAt: '2026-01-30T10:00:00.000Z'
        },
        {
          id: '3',
          title: 'Pending task second',
          status: 'pending' as TaskStatus,
          details: [],
          createdAt: '2026-01-30T10:00:00.000Z',
          updatedAt: '2026-01-30T10:00:00.000Z'
        }
      ];

      const summary = {
        pending: 2,
        inProgress: 1,
        completed: 1
      };

      const prompt = promptGenerator.generateIncompleteTaskPrompt(incompleteTasks, summary);

      // 헤더 확인
      expect(prompt).toContain('⚠️ **작업 완료 알림**');
      expect(prompt).toContain('현재 세션에 완료되지 않은 작업이 있습니다.');

      // 요약 정보 확인
      expect(prompt).toContain('🔄 진행 중: 1개');
      expect(prompt).toContain('⏳ 대기 중: 2개');
      expect(prompt).toContain('✅ 완료됨: 1개');

      // 작업 목록 확인
      expect(prompt).toContain('**남은 작업 목록:**');
      expect(prompt).toContain('🔄 **1**. Task in progress (진행 중)');
      expect(prompt).toContain('⏳ **2**. Pending task first (대기 중)');
      expect(prompt).toContain('⏳ **3**. Pending task second (대기 중)');

      // 완료 요청 메시지 확인
      expect(prompt).toContain('📌 **이 작업들을 완료해주세요.**');
      expect(prompt).toContain('완료 후 `tasks` 도구를 사용하여 상태를 업데이트하세요.');
    });

    it('should generate prompt with only pending tasks', () => {
      const incompleteTasks: TaskDetail[] = [
        {
          id: '1',
          title: 'Only pending 1',
          status: 'pending' as TaskStatus,
          details: [],
          createdAt: '2026-01-30T10:00:00.000Z',
          updatedAt: '2026-01-30T10:00:00.000Z'
        },
        {
          id: '2',
          title: 'Only pending 2',
          status: 'pending' as TaskStatus,
          details: [],
          createdAt: '2026-01-30T10:00:00.000Z',
          updatedAt: '2026-01-30T10:00:00.000Z'
        }
      ];

      const summary = {
        pending: 2,
        inProgress: 0,
        completed: 0
      };

      const prompt = promptGenerator.generateIncompleteTaskPrompt(incompleteTasks, summary);

      expect(prompt).toContain('⏳ 대기 중: 2개');
      expect(prompt).toContain('🔄 진행 중: 0개');
      expect(prompt).toContain('⏳ **1**. Only pending 1 (대기 중)');
      expect(prompt).toContain('⏳ **2**. Only pending 2 (대기 중)');
    });

    it('should generate prompt with only in_progress tasks', () => {
      const incompleteTasks: TaskDetail[] = [
        {
          id: '1',
          title: 'Only in progress 1',
          status: 'in_progress' as TaskStatus,
          details: ['Working on this'],
          createdAt: '2026-01-30T10:00:00.000Z',
          updatedAt: '2026-01-30T12:00:00.000Z'
        }
      ];

      const summary = {
        pending: 0,
        inProgress: 1,
        completed: 3
      };

      const prompt = promptGenerator.generateIncompleteTaskPrompt(incompleteTasks, summary);

      expect(prompt).toContain('🔄 진행 중: 1개');
      expect(prompt).toContain('⏳ 대기 중: 0개');
      expect(prompt).toContain('✅ 완료됨: 3개');
      expect(prompt).toContain('🔄 **1**. Only in progress 1 (진행 중)');
    });

    it('should handle empty incomplete tasks list', () => {
      const incompleteTasks: TaskDetail[] = [];
      const summary = {
        pending: 0,
        inProgress: 0,
        completed: 5
      };

      const prompt = promptGenerator.generateIncompleteTaskPrompt(incompleteTasks, summary);

      // 빈 목록이어도 기본 구조는 유지되어야 함
      expect(prompt).toContain('⚠️ **작업 완료 알림**');
      expect(prompt).toContain('**남은 작업 목록:**');
      expect(prompt).toContain('✅ 완료됨: 5개');
    });

    it('should handle hierarchical task IDs (subtasks)', () => {
      const incompleteTasks: TaskDetail[] = [
        {
          id: '1.1',
          title: 'Subtask in progress',
          status: 'in_progress' as TaskStatus,
          details: [],
          createdAt: '2026-01-30T10:00:00.000Z',
          updatedAt: '2026-01-30T11:00:00.000Z'
        },
        {
          id: '2.1',
          title: 'Subtask pending',
          status: 'pending' as TaskStatus,
          details: [],
          createdAt: '2026-01-30T10:00:00.000Z',
          updatedAt: '2026-01-30T10:00:00.000Z'
        }
      ];

      const summary = {
        pending: 1,
        inProgress: 1,
        completed: 0
      };

      const prompt = promptGenerator.generateIncompleteTaskPrompt(incompleteTasks, summary);

      expect(prompt).toContain('🔄 **1.1**. Subtask in progress (진행 중)');
      expect(prompt).toContain('⏳ **2.1**. Subtask pending (대기 중)');
    });

    it('should format prompt in Korean', () => {
      const incompleteTasks: TaskDetail[] = [
        {
          id: '1',
          title: 'Korean task',
          status: 'pending' as TaskStatus,
          details: [],
          createdAt: '2026-01-30T10:00:00.000Z',
          updatedAt: '2026-01-30T10:00:00.000Z'
        }
      ];

      const summary = {
        pending: 1,
        inProgress: 0,
        completed: 0
      };

      const prompt = promptGenerator.generateIncompleteTaskPrompt(incompleteTasks, summary);

      // 한국어 메시지 확인
      expect(prompt).toContain('현재 세션에 완료되지 않은 작업이 있습니다.');
      expect(prompt).toContain('완료됨');
      expect(prompt).toContain('진행 중');
      expect(prompt).toContain('대기 중');
      expect(prompt).toContain('이 작업들을 완료해주세요.');
    });

    it('should include markdown formatting', () => {
      const incompleteTasks: TaskDetail[] = [
        {
          id: '1',
          title: 'Test task',
          status: 'in_progress' as TaskStatus,
          details: [],
          createdAt: '2026-01-30T10:00:00.000Z',
          updatedAt: '2026-01-30T10:00:00.000Z'
        }
      ];

      const summary = {
        pending: 0,
        inProgress: 1,
        completed: 0
      };

      const prompt = promptGenerator.generateIncompleteTaskPrompt(incompleteTasks, summary);

      // 마크다운 포맷팅 확인
      expect(prompt).toContain('**'); // 볼드 텍스트
      expect(prompt).toContain('\n'); // 줄바꿈
    });
  });

  describe('edge cases', () => {
    it('should handle task with very long title', () => {
      const incompleteTasks: TaskDetail[] = [
        {
          id: '1',
          title: 'A'.repeat(200),
          status: 'pending' as TaskStatus,
          details: [],
          createdAt: '2026-01-30T10:00:00.000Z',
          updatedAt: '2026-01-30T10:00:00.000Z'
        }
      ];

      const summary = {
        pending: 1,
        inProgress: 0,
        completed: 0
      };

      const prompt = promptGenerator.generateIncompleteTaskPrompt(incompleteTasks, summary);

      expect(prompt).toContain('**1**.');
      expect(prompt).toContain('A'.repeat(200));
    });

    it('should handle task with special characters in title', () => {
      const incompleteTasks: TaskDetail[] = [
        {
          id: '1',
          title: 'Task with special chars: @#$%^&*()<>[]{}',
          status: 'in_progress' as TaskStatus,
          details: [],
          createdAt: '2026-01-30T10:00:00.000Z',
          updatedAt: '2026-01-30T10:00:00.000Z'
        }
      ];

      const summary = {
        pending: 0,
        inProgress: 1,
        completed: 0
      };

      const prompt = promptGenerator.generateIncompleteTaskPrompt(incompleteTasks, summary);

      expect(prompt).toContain('Task with special chars: @#$%^&*()<>[]{}');
    });

    it('should handle task with Korean characters in title', () => {
      const incompleteTasks: TaskDetail[] = [
        {
          id: '1',
          title: '한국어 작업 제목',
          status: 'pending' as TaskStatus,
          details: [],
          createdAt: '2026-01-30T10:00:00.000Z',
          updatedAt: '2026-01-30T10:00:00.000Z'
        }
      ];

      const summary = {
        pending: 1,
        inProgress: 0,
        completed: 0
      };

      const prompt = promptGenerator.generateIncompleteTaskPrompt(incompleteTasks, summary);

      expect(prompt).toContain('한국어 작업 제목');
      expect(prompt).toContain('⏳ **1**. 한국어 작업 제목 (대기 중)');
    });
  });
});
