// src/lib/completion-checker.ts

import { Storage } from './storage';
import { Parser } from './parser';
import { TaskDetail, CompletionCheckResult } from '../types';

export class CompletionChecker {
  constructor(private storage: Storage, private parser: Parser) {}

  /**
   * 세션의 모든 작업 목록을 확인하여 미완료 작업 수집
   */
  async checkIncompleteTasks(sessionId: string): Promise<CompletionCheckResult> {
    try {
      // 세션의 모든 작업 파일 목록 조회
      const taskFiles = await this.storage.listTaskFiles(sessionId);
      
      const incompleteTasks: TaskDetail[] = [];
      let pendingCount = 0;
      let inProgressCount = 0;
      let completedCount = 0;

      // 각 작업 파일을 읽어서 미완료 작업 확인
      for (const fileName of taskFiles) {
        const title = fileName.replace('.md', '');
        const content = await this.storage.readTaskList(sessionId, title);
        
        if (content) {
          const taskList = this.parser.parseTaskList(content);
          
          for (const task of taskList.tasks) {
            if (task.status === 'pending') {
              pendingCount++;
              incompleteTasks.push(task);
            } else if (task.status === 'in_progress') {
              inProgressCount++;
              incompleteTasks.push(task);
            } else if (task.status === 'completed') {
              completedCount++;
            }
          }
        }
      }

      return {
        hasIncomplete: incompleteTasks.length > 0,
        incompleteTasks,
        summary: {
          pending: pendingCount,
          inProgress: inProgressCount,
          completed: completedCount
        }
      };
    } catch (error) {
      // 에러 발생 시 기본값 반환 (미완료 작업 없음으로 간주)
      console.error('Error checking incomplete tasks:', error);
      return {
        hasIncomplete: false,
        incompleteTasks: [],
        summary: {
          pending: 0,
          inProgress: 0,
          completed: 0
        }
      };
    }
  }
}
