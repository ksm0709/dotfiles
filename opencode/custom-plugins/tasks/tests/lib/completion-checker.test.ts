import * as fs from 'fs/promises';
import * as path from 'path';
import * as os from 'os';
import type { CompletionChecker as CompletionCheckerType } from '../../src/lib/completion-checker';
import type { Storage as StorageType } from '../../src/lib/storage';
import type { Parser as ParserType } from '../../src/lib/parser';

describe('CompletionChecker - Session Completion Check (ADDED-002)', () => {
  let completionChecker: CompletionCheckerType;
  let storage: StorageType;
  let parser: ParserType;
  let tempDir: string;
  let originalHome: string | undefined;
  let originalXdgDataHome: string | undefined;

  beforeEach(async () => {
    // 임시 디렉토리 생성
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), 'tasks-completion-test-'));

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

    const Storage = storageModule.Storage;
    const Parser = parserModule.Parser;
    const CompletionChecker = completionCheckerModule.CompletionChecker;

    storage = new Storage();
    parser = new Parser();
    completionChecker = new CompletionChecker(storage, parser);
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

  describe('checkIncompleteTasks', () => {
    it('should detect incomplete tasks (pending + in_progress) when session ends (ADDED-002)', async () => {
      const sessionId = 'test-session-incomplete';
      const content = `# Task List: Incomplete Test

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [ ] 1. Pending task first
- [~] 2. In Progress task
- [ ] 3. Pending task second
- [x] 4. Completed task

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Implementation  
**상태**: in_progress  
**완료율**: 25% (1/4)
`;

      await storage.saveTaskList(sessionId, 'Incomplete Test', content);

      const result = await completionChecker.checkIncompleteTasks(sessionId);

      expect(result.hasIncomplete).toBe(true);
      expect(result.incompleteTasks).toHaveLength(3);
      expect(result.summary.pending).toBe(2);
      expect(result.summary.inProgress).toBe(1);
      expect(result.summary.completed).toBe(1);

      // 미완료 task 확인
      const incompleteIds = result.incompleteTasks.map(t => t.id);
      expect(incompleteIds).toContain('1');
      expect(incompleteIds).toContain('2');
      expect(incompleteIds).toContain('3');
      expect(incompleteIds).not.toContain('4');
    });

    it('should return hasIncomplete=false when all tasks are completed (ADDED-002)', async () => {
      const sessionId = 'test-session-complete';
      const content = `# Task List: Complete Test

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

      await storage.saveTaskList(sessionId, 'Complete Test', content);

      const result = await completionChecker.checkIncompleteTasks(sessionId);

      expect(result.hasIncomplete).toBe(false);
      expect(result.incompleteTasks).toHaveLength(0);
      expect(result.summary.pending).toBe(0);
      expect(result.summary.inProgress).toBe(0);
      expect(result.summary.completed).toBe(3);
    });

    it('should return hasIncomplete=false for empty session (ADDED-002)', async () => {
      const sessionId = 'test-session-empty';

      const result = await completionChecker.checkIncompleteTasks(sessionId);

      expect(result.hasIncomplete).toBe(false);
      expect(result.incompleteTasks).toHaveLength(0);
      expect(result.summary.pending).toBe(0);
      expect(result.summary.inProgress).toBe(0);
      expect(result.summary.completed).toBe(0);
    });

    it('should handle multiple task list files in same session', async () => {
      const sessionId = 'test-session-multi';

      const content1 = `# Task List: First List

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [~] 1. In progress from first
- [x] 2. Completed from first

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Implementation  
**상태**: in_progress  
**완료율**: 50% (1/2)
`;

      const content2 = `# Task List: Second List

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [ ] 1. Pending from second
- [~] 2. In progress from second

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Development  
**상태**: in_progress  
**완료율**: 0% (0/2)
`;

      await storage.saveTaskList(sessionId, 'First List', content1);
      await storage.saveTaskList(sessionId, 'Second List', content2);

      const result = await completionChecker.checkIncompleteTasks(sessionId);

      expect(result.hasIncomplete).toBe(true);
      expect(result.incompleteTasks).toHaveLength(3);
      expect(result.summary.pending).toBe(1);
      expect(result.summary.inProgress).toBe(2);
      expect(result.summary.completed).toBe(1);
    });

    it('should correctly identify only pending tasks', async () => {
      const sessionId = 'test-session-pending';
      const content = `# Task List: Only Pending

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [ ] 1. Only pending 1
- [ ] 2. Only pending 2

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Planning  
**상태**: pending  
**완료율**: 0% (0/2)
`;

      await storage.saveTaskList(sessionId, 'Only Pending', content);

      const result = await completionChecker.checkIncompleteTasks(sessionId);

      expect(result.hasIncomplete).toBe(true);
      expect(result.incompleteTasks).toHaveLength(2);
      expect(result.summary.pending).toBe(2);
      expect(result.summary.inProgress).toBe(0);
      expect(result.summary.completed).toBe(0);
    });

    it('should correctly identify only in_progress tasks', async () => {
      const sessionId = 'test-session-inprogress-only';
      const content = `# Task List: Only In Progress

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [~] 1. Only in progress 1
- [~] 2. Only in progress 2

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Implementation  
**상태**: in_progress  
**완료율**: 0% (0/2)
`;

      await storage.saveTaskList(sessionId, 'Only In Progress', content);

      const result = await completionChecker.checkIncompleteTasks(sessionId);

      expect(result.hasIncomplete).toBe(true);
      expect(result.incompleteTasks).toHaveLength(2);
      expect(result.summary.pending).toBe(0);
      expect(result.summary.inProgress).toBe(2);
      expect(result.summary.completed).toBe(0);
    });

    it('should include task details in incompleteTasks', async () => {
      const sessionId = 'test-session-details';
      const content = `# Task List: With Details

**에이전트**: test-agent  
**생성일**: 2026-01-30 10:00:00  
**세션 ID**: ${sessionId}

---

## 작업 목록 (Task List)

- [~] 1. Task with details
  - Detail line 1
  - Detail line 2

---

## 진행 상황 요약 (Progress Summary)

**현재 단계**: Implementation  
**상태**: in_progress  
**완료율**: 0% (0/1)
`;

      await storage.saveTaskList(sessionId, 'With Details', content);

      const result = await completionChecker.checkIncompleteTasks(sessionId);

      expect(result.hasIncomplete).toBe(true);
      expect(result.incompleteTasks).toHaveLength(1);
      expect(result.incompleteTasks[0].id).toBe('1');
      expect(result.incompleteTasks[0].title).toBe('Task with details');
      expect(result.incompleteTasks[0].status).toBe('in_progress');
    });
  });

  describe('edge cases', () => {
    it('should handle task list file with empty task list', async () => {
      const sessionId = 'test-session-empty-list';
      const content = `# Task List: Empty List

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

      await storage.saveTaskList(sessionId, 'Empty List', content);

      const result = await completionChecker.checkIncompleteTasks(sessionId);

      expect(result.hasIncomplete).toBe(false);
      expect(result.incompleteTasks).toHaveLength(0);
    });

    it('should handle session directory that does not exist', async () => {
      const sessionId = 'non-existent-session-id';

      const result = await completionChecker.checkIncompleteTasks(sessionId);

      expect(result.hasIncomplete).toBe(false);
      expect(result.incompleteTasks).toHaveLength(0);
      expect(result.summary.pending).toBe(0);
      expect(result.summary.inProgress).toBe(0);
      expect(result.summary.completed).toBe(0);
    });
  });
});
