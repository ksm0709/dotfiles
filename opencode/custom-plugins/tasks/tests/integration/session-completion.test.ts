import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';

describe('Session Completion Integration Test (ADDED-002)', () => {
  let tempDir: string;
  let originalHome: string | undefined;
  let originalXdgDataHome: string | undefined;

  // 플러그인 및 의존성 모듈
  let Storage: typeof import('../../src/lib/storage').Storage;
  let Parser: typeof import('../../src/lib/parser').Parser;
  let CompletionChecker: typeof import('../../src/lib/completion-checker').CompletionChecker;
  let PromptGenerator: typeof import('../../src/lib/prompt-generator').PromptGenerator;

  beforeEach(async () => {
    // 임시 디렉토리 생성
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-integration-test-'));

    // 환경 변수 모의 설정
    originalHome = process.env.HOME;
    originalXdgDataHome = process.env.XDG_DATA_HOME;
    delete process.env.XDG_DATA_HOME;
    process.env.HOME = tempDir;

    // 모듈 재로드
    jest.resetModules();
    const storageModule = await import('../../src/lib/storage');
    const parserModule = await import('../../src/lib/parser');
    const completionCheckerModule = await import('../../src/lib/completion-checker');
    const promptGeneratorModule = await import('../../src/lib/prompt-generator');

    Storage = storageModule.Storage;
    Parser = parserModule.Parser;
    CompletionChecker = completionCheckerModule.CompletionChecker;
    PromptGenerator = promptGeneratorModule.PromptGenerator;
  });

  afterEach(async () => {
    // 환경 변수 복원
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

    // 임시 디렉토리 정리
    try {
      await fs.rm(tempDir, { recursive: true, force: true });
    } catch (error) {
      // 정리 오류 무시
    }
  });

  describe('Scenario: 미완료 task가 있는 세션 종료', () => {
    it('should check incomplete tasks when session.idle event occurs', async () => {
      const sessionId = 'test-session-idle-incomplete';
      const storage = new Storage();
      const parser = new Parser();
      const completionChecker = new CompletionChecker(storage, parser);

      // 미완료 task가 있는 task list 생성
      const content = `# Task List: Integration Test

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [ ] 1. Pending task
- [~] 2. In Progress task
- [x] 3. Completed task

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Implementation  
**상태**: in_progress  
**완료율**: 33% (1/3)
`;

      await storage.saveTaskList(sessionId, 'Integration Test', content);

      // 세션 종료 체크 수행
      const result = await completionChecker.checkIncompleteTasks(sessionId);

      // 미완료 task가 있는지 확인
      expect(result.hasIncomplete).toBe(true);
      expect(result.incompleteTasks).toHaveLength(2);
      expect(result.summary.pending).toBe(1);
      expect(result.summary.inProgress).toBe(1);
      expect(result.summary.completed).toBe(1);
    });

    it('should generate prompt for incomplete tasks', async () => {
      const sessionId = 'test-session-prompt';
      const storage = new Storage();
      const parser = new Parser();
      const completionChecker = new CompletionChecker(storage, parser);
      const promptGenerator = new PromptGenerator();

      const content = `# Task List: Prompt Test

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [~] 1. Task to complete
- [ ] 2. Another pending task

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Testing  
**상태**: in_progress  
**완료율**: 0% (0/2)
`;

      await storage.saveTaskList(sessionId, 'Prompt Test', content);

      const result = await completionChecker.checkIncompleteTasks(sessionId);
      expect(result.hasIncomplete).toBe(true);

      // 프롬프트 생성
      const prompt = promptGenerator.generateIncompleteTaskPrompt(
        result.incompleteTasks,
        result.summary
      );

      // 프롬프트 내용 검증
      expect(prompt).toContain('⚠️ **작업 완료 알림**');
      expect(prompt).toContain('🔄 진행 중: 1개');
      expect(prompt).toContain('⏳ 대기 중: 1개');
    });
  });

  describe('Scenario: 모든 task가 완료된 세션 종료', () => {
    it('should not generate prompt when all tasks are completed', async () => {
      const sessionId = 'test-session-all-complete';
      const storage = new Storage();
      const parser = new Parser();
      const completionChecker = new CompletionChecker(storage, parser);

      const content = `# Task List: All Complete Test

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [x] 1. First completed task
- [x] 2. Second completed task
- [x] 3. Third completed task

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Done  
**상태**: completed  
**완료율**: 100% (3/3)
`;

      await storage.saveTaskList(sessionId, 'All Complete Test', content);

      const result = await completionChecker.checkIncompleteTasks(sessionId);

      // 모든 task가 완료되었으므로 미완료 task가 없어야 함
      expect(result.hasIncomplete).toBe(false);
      expect(result.incompleteTasks).toHaveLength(0);
      expect(result.summary.pending).toBe(0);
      expect(result.summary.inProgress).toBe(0);
      expect(result.summary.completed).toBe(3);
    });

    it('should return null or empty prompt for completed session', async () => {
      const sessionId = 'test-session-no-prompt';
      const storage = new Storage();
      const parser = new Parser();
      const completionChecker = new CompletionChecker(storage, parser);
      const promptGenerator = new PromptGenerator();

      const content = `# Task List: No Prompt Test

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [x] 1. Only completed task

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Done  
**상태**: completed  
**완료율**: 100% (1/1)
`;

      await storage.saveTaskList(sessionId, 'No Prompt Test', content);

      const result = await completionChecker.checkIncompleteTasks(sessionId);

      // 모든 task가 완료된 경우 프롬프트를 생성하지 않아야 함
      expect(result.hasIncomplete).toBe(false);
    });
  });

  describe('Scenario: 빈 세션 종료', () => {
    it('should handle empty session without error', async () => {
      const sessionId = 'test-session-empty';
      const storage = new Storage();
      const parser = new Parser();
      const completionChecker = new CompletionChecker(storage, parser);

      // task list가 없는 세션
      const result = await completionChecker.checkIncompleteTasks(sessionId);

      expect(result.hasIncomplete).toBe(false);
      expect(result.incompleteTasks).toHaveLength(0);
      expect(result.summary.pending).toBe(0);
      expect(result.summary.inProgress).toBe(0);
      expect(result.summary.completed).toBe(0);
    });

    it('should handle session with empty task list', async () => {
      const sessionId = 'test-session-empty-list';
      const storage = new Storage();
      const parser = new Parser();
      const completionChecker = new CompletionChecker(storage, parser);

      const content = `# Task List: Empty List Test

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: 미정  
**상태**: pending  
**완료율**: 0% (0/0)
`;

      await storage.saveTaskList(sessionId, 'Empty List Test', content);

      const result = await completionChecker.checkIncompleteTasks(sessionId);

      expect(result.hasIncomplete).toBe(false);
      expect(result.incompleteTasks).toHaveLength(0);
    });
  });

  describe('End-to-End: session.idle event simulation', () => {
    it('should simulate complete session end workflow with incomplete tasks', async () => {
      const sessionId = 'test-session-e2e-incomplete';
      const storage = new Storage();
      const parser = new Parser();
      const completionChecker = new CompletionChecker(storage, parser);
      const promptGenerator = new PromptGenerator();

      // 1. Task List 저장
      const content = `# Task List: E2E Test

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [~] 1. First task (in progress)
- [ ] 2. Second task (pending)
- [~] 3. Third task (in progress)
- [x] 4. Fourth task (completed)

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Implementation  
**상태**: in_progress  
**완료율**: 25% (1/4)
`;

      await storage.saveTaskList(sessionId, 'E2E Test', content);

      // 2. 세션 종료 체크
      const checkResult = await completionChecker.checkIncompleteTasks(sessionId);
      expect(checkResult.hasIncomplete).toBe(true);
      expect(checkResult.incompleteTasks).toHaveLength(3);

      // 3. 프롬프트 생성
      const prompt = promptGenerator.generateIncompleteTaskPrompt(
        checkResult.incompleteTasks,
        checkResult.summary
      );

      // 4. 프롬프트 내용 검증
      expect(prompt).toContain('⚠️ **작업 완료 알림**');
      expect(prompt).toContain('🔄 진행 중: 2개');
      expect(prompt).toContain('⏳ 대기 중: 1개');
      expect(prompt).toContain('✅ 완료됨: 1개');

      // 5. 미완료 task 목록 확인
      const task1 = checkResult.incompleteTasks.find(t => t.id === '1');
      const task2 = checkResult.incompleteTasks.find(t => t.id === '2');
      const task3 = checkResult.incompleteTasks.find(t => t.id === '3');

      expect(task1).toBeDefined();
      expect(task1?.status).toBe('in_progress');
      expect(task2).toBeDefined();
      expect(task2?.status).toBe('pending');
      expect(task3).toBeDefined();
      expect(task3?.status).toBe('in_progress');
    });

    it('should simulate complete session end workflow with all completed tasks', async () => {
      const sessionId = 'test-session-e2e-complete';
      const storage = new Storage();
      const parser = new Parser();
      const completionChecker = new CompletionChecker(storage, parser);

      // 1. 완료된 Task List 저장
      const content = `# Task List: E2E Complete Test

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [x] 1. All done 1
- [x] 2. All done 2

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Done  
**상태**: completed  
**완료율**: 100% (2/2)
`;

      await storage.saveTaskList(sessionId, 'E2E Complete Test', content);

      // 2. 세션 종료 체크
      const checkResult = await completionChecker.checkIncompleteTasks(sessionId);

      // 3. 모든 task가 완료되었으므로 미완료 task가 없어야 함
      expect(checkResult.hasIncomplete).toBe(false);
      expect(checkResult.incompleteTasks).toHaveLength(0);
      expect(checkResult.summary.pending).toBe(0);
      expect(checkResult.summary.inProgress).toBe(0);
      expect(checkResult.summary.completed).toBe(2);
    });
  });

  describe('Multi-task-list session handling', () => {
    it('should aggregate incomplete tasks from multiple task lists', async () => {
      const sessionId = 'test-session-multi-lists';
      const storage = new Storage();
      const parser = new Parser();
      const completionChecker = new CompletionChecker(storage, parser);

      // 첫 번째 task list
      const content1 = `# Task List: First List

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [~] 1. First list in progress
- [x] 2. First list completed

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Development  
**상태**: in_progress  
**완료율**: 50% (1/2)
`;

      // 두 번째 task list
      const content2 = `# Task List: Second List

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [ ] 1. Second list pending
- [~] 2. Second list in progress
- [ ] 3. Another pending

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Testing  
**상태**: in_progress  
**완료율**: 0% (0/3)
`;

      await storage.saveTaskList(sessionId, 'First List', content1);
      await storage.saveTaskList(sessionId, 'Second List', content2);

      const result = await completionChecker.checkIncompleteTasks(sessionId);

      // 두 task list에서 미완료 task 합산
      expect(result.hasIncomplete).toBe(true);
      expect(result.incompleteTasks).toHaveLength(4); // 1 (in_progress) + 1 (pending) + 2 (pending, in_progress)
      expect(result.summary.pending).toBe(2);
      expect(result.summary.inProgress).toBe(2);
      expect(result.summary.completed).toBe(1);
    });
  });
});
