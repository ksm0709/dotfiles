// src/lib/formatter.ts

import {
  TaskList,
  TaskDetail,
  TaskStatusSummary,
  StatusSummary,
  BatchResult,
  BatchOperation
} from '../types';

export class Formatter {
  formatAsMarkdown(taskList: TaskList): string {
    const lines: string[] = [];

    lines.push(`# Task List: ${taskList.title}`);
    lines.push('');
    lines.push(`**에이전트**: ${taskList.agent}`);
    lines.push(`**생성일**: ${taskList.createdAt}`);
    lines.push(`**세션 ID**: ${taskList.sessionId}`);
    lines.push('');
    lines.push('---');
    lines.push('');
    lines.push('## 작업 목록');
    lines.push('');

    for (const task of taskList.tasks) {
      lines.push(...this.formatTaskMarkdown(task, 0));
    }

    lines.push('');
    lines.push('---');
    lines.push('');
    lines.push('## 진행 상황 요약');
    lines.push('');
    
    const stats = this.calculateStats(taskList.tasks);
    lines.push(`**현재 단계**: ${taskList.currentPhase || '미정'}`);
    lines.push(`**상태**: ${stats.status}`);
    lines.push(`**완료율**: ${stats.completionRate}% (${stats.completedCount}/${stats.totalCount})`);
    
    if (taskList.memo) {
      lines.push(`**메모**: ${taskList.memo}`);
    }

    return lines.join('\n');
  }

  private formatTaskMarkdown(task: TaskDetail, indent: number): string[] {
    const lines: string[] = [];
    const prefix = '  '.repeat(indent);
    const checkbox = task.status === 'completed' ? '[x]' : '[ ]';
    const statusEmoji = task.status === 'completed' ? '✅' : 
                       task.status === 'in_progress' ? '🔄' : '⏳';
    
    lines.push(`${prefix}- ${checkbox} ${statusEmoji} **${task.id}**. ${task.title}`);

    for (const detail of task.details) {
      lines.push(`${prefix}  - ${detail}`);
    }

    if (task.subtasks) {
      for (const subtask of task.subtasks) {
        lines.push(...this.formatTaskMarkdown(subtask, indent + 1));
      }
    }

    return lines;
  }

  formatAsJSON(taskList: TaskList): string {
    return JSON.stringify(taskList, null, 2);
  }

  formatAsTable(taskList: TaskList): string {
    const lines: string[] = [];
    
    lines.push(`# ${taskList.title}`);
    lines.push('');
    lines.push('| ID | 상태 | 제목 | 세부사항 |');
    lines.push('|------|--------|------|----------|');

    const formatTaskRow = (task: TaskDetail, indent: string): void => {
      const status = task.status === 'completed' ? '✅ 완료' : 
                    task.status === 'in_progress' ? '🔄 진행중' : '⏳ 대기';
      const details = task.details.join(', ').substring(0, 30);
      const title = indent + task.title;
      
      lines.push(`| ${task.id} | ${status} | ${title} | ${details}${details.length > 30 ? '...' : ''} |`);

      if (task.subtasks) {
        for (const subtask of task.subtasks) {
          formatTaskRow(subtask, indent + '  ');
        }
      }
    };

    for (const task of taskList.tasks) {
      formatTaskRow(task, '');
    }

    return lines.join('\n');
  }

  formatStatusSummary(summary: TaskStatusSummary): string {
    const lines: string[] = [];
    
    lines.push(`📋 Task Status Summary`);
    lines.push('');
    lines.push(`에이전트: ${summary.agent}`);
    lines.push(`작업: ${summary.title}`);
    lines.push(`상태: ${this.getStatusEmoji(summary.status)} ${summary.status}`);
    lines.push(`완료율: ${summary.completionRate}% (${summary.completedCount}/${summary.totalCount})`);
    
    if (summary.currentPhase) {
      lines.push(`현재 단계: ${summary.currentPhase}`);
    }

    // Progress bar
    const barLength = 20;
    const filledLength = Math.round((summary.completionRate / 100) * barLength);
    const bar = '█'.repeat(filledLength) + '░'.repeat(barLength - filledLength);
    lines.push(`진행도: [${bar}] ${summary.completionRate}%`);

    return lines.join('\n');
  }

  private getStatusEmoji(status: string): string {
    switch (status) {
      case 'completed': return '✅';
      case 'in_progress': return '🔄';
      default: return '⏳';
    }
  }

  private calculateStats(tasks: TaskDetail[]): { status: string; completionRate: number; completedCount: number; totalCount: number } {
    let total = 0;
    let completed = 0;
    let inProgress = 0;

    const countTasks = (taskList: TaskDetail[]) => {
      for (const task of taskList) {
        total++;
        if (task.status === 'completed') {
          completed++;
        } else if (task.status === 'in_progress') {
          inProgress++;
        }
        if (task.subtasks) {
          countTasks(task.subtasks);
        }
      }
    };

    countTasks(tasks);

    const completionRate = total > 0 ? Math.round((completed / total) * 100) : 0;
    let status = 'pending';
    if (completed === total && total > 0) {
      status = 'completed';
    } else if (inProgress > 0 || completed > 0) {
      status = 'in_progress';
    }

    return { status, completionRate, completedCount: completed, totalCount: total };
  }

  // ===========================================
  // New Methods for Batch Operations & UI Enhancement
  // ===========================================

  /**
   * 마크다운 형식의 전체 현황 출력
   */
  formatTaskListWithStatus(taskList: TaskList, summary: StatusSummary): string {
    const lines: string[] = [];

    lines.push(`# 📋 ${taskList.title}`);
    lines.push('');
    lines.push(`**에이전트**: ${taskList.agent}`);
    lines.push('');
    lines.push('## 📊 진행 상황');
    lines.push('');
    lines.push(`| 상태 | 개수 | 비율 |`);
    lines.push(`|------|------|------|`);
    lines.push(`| ✅ 완료 | ${summary.completed} | ${Math.round((summary.completed / summary.total) * 100) || 0}% |`);
    lines.push(`| 🔄 진행중 | ${summary.inProgress} | ${Math.round((summary.inProgress / summary.total) * 100) || 0}% |`);
    lines.push(`| ⏳ 대기 | ${summary.pending} | ${Math.round((summary.pending / summary.total) * 100) || 0}% |`);
    lines.push(`| **합계** | **${summary.total}** | **${summary.completionRate}%** |`);
    lines.push('');
    lines.push('### 진행률');
    lines.push('');
    lines.push(this.formatProgressBar(summary.completionRate));
    lines.push('');
    lines.push('---');
    lines.push('');
    lines.push('## 📋 작업 목록');
    lines.push('');

    for (const task of taskList.tasks) {
      lines.push(...this.formatTaskWithStatus(task, 0));
    }

    return lines.join('\n');
  }

  private formatTaskWithStatus(task: TaskDetail, indent: number): string[] {
    const lines: string[] = [];
    const prefix = '  '.repeat(indent);
    const checkbox = task.status === 'completed' ? '[x]' : '[ ]';
    const statusEmoji = task.status === 'completed' ? '✅' : 
                       task.status === 'in_progress' ? '🔄' : '⏳';
    
    lines.push(`${prefix}- ${checkbox} ${statusEmoji} **${task.id}**. ${this.escapeMarkdown(task.title)}`);

    for (const detail of task.details) {
      lines.push(`${prefix}  - ${this.escapeMarkdown(detail)}`);
    }

    if (task.subtasks && task.subtasks.length > 0) {
      for (const subtask of task.subtasks) {
        lines.push(...this.formatTaskWithStatus(subtask, indent + 1));
      }
    }

    return lines;
  }

  /**
   * 배치 작업 결과 포맷팅
   */
  formatBatchResult(
    results: BatchResult[],
    summary: { total: number; succeeded: number; failed: number },
    taskList: TaskList,
    statusSummary: StatusSummary
  ): string {
    const lines: string[] = [];

    lines.push('# 📦 배치 작업 결과');
    lines.push('');
    lines.push('## 📊 요약');
    lines.push('');
    lines.push(`- **총 작업**: ${summary.total}`);
    lines.push(`- **✅ 성공**: ${summary.succeeded}`);
    lines.push(`- **❌ 실패**: ${summary.failed}`);
    lines.push('');
    lines.push(this.formatProgressBar(Math.round((summary.succeeded / summary.total) * 100)));
    lines.push('');

    if (summary.failed > 0) {
      lines.push('## ❌ 실패한 작업');
      lines.push('');
      for (const result of results.filter(r => !r.success)) {
        lines.push(`### ${this.formatOperationType(result.operation.type)}: ${result.operation.title || result.operation.id}`);
        lines.push(`- **오류**: ${result.message}`);
        if (result.error) {
          lines.push(`- **상세**: ${result.error}`);
        }
        lines.push('');
      }
    }

    lines.push('## ✅ 작업 상세');
    lines.push('');
    for (const result of results) {
      const emoji = result.success ? '✅' : '❌';
      lines.push(`- ${emoji} **${result.operation.type.toUpperCase()}**: ${result.message}`);
    }
    lines.push('');
    lines.push('---');
    lines.push('');
    lines.push('## 📋 현재 작업 현황');
    lines.push('');
    lines.push(this.formatTaskListWithStatus(taskList, statusSummary));

    return lines.join('\n');
  }

  private formatOperationType(type: string): string {
    const typeMap: Record<string, string> = {
      'add': '➕ 추가',
      'update': '🔄 업데이트',
      'complete': '✅ 완료',
      'remove': '🗑️ 삭제'
    };
    return typeMap[type] || type;
  }

  /**
   * add 작업 결과 포맷팅
   */
  formatAddResult(task: TaskDetail, taskList: TaskList, summary: StatusSummary): string {
    const lines: string[] = [];

    lines.push(`✅ 작업 추가 완료`);
    lines.push('');
    lines.push(`**제목**: ${task.title}`);
    lines.push(`**ID**: ${task.id}`);
    lines.push(`**상태**: ⏳ 대기`);
    lines.push('');
    lines.push('---');
    lines.push('');
    lines.push(this.formatTaskListWithStatus(taskList, summary));

    return lines.join('\n');
  }

  /**
   * update 작업 결과 포맷팅
   */
  formatUpdateResult(
    taskId: string,
    newStatus: string,
    taskList: TaskList,
    summary: StatusSummary
  ): string {
    const lines: string[] = [];

    const statusEmoji = newStatus === 'completed' ? '✅' : 
                       newStatus === 'in_progress' ? '🔄' : '⏳';

    lines.push(`✅ 작업 상태 업데이트 완료`);
    lines.push('');
    lines.push(`**ID**: ${taskId}`);
    lines.push(`**새 상태**: ${statusEmoji} ${newStatus}`);
    lines.push('');
    lines.push('---');
    lines.push('');
    lines.push(this.formatTaskListWithStatus(taskList, summary));

    return lines.join('\n');
  }

  /**
   * 텍스트 기반 진행률 바 생성
   * 예: [████░░░░░░] 40%
   */
  formatProgressBar(percentage: number, barLength: number = 20): string {
    const clampedPercentage = Math.max(0, Math.min(100, percentage));
    const filledLength = Math.round((clampedPercentage / 100) * barLength);
    const emptyLength = barLength - filledLength;
    
    const bar = '█'.repeat(filledLength) + '░'.repeat(emptyLength);
    return `[${bar}] ${clampedPercentage}%`;
  }

  /**
   * TaskList에서 상태 통계 계산
   */
  calculateStatusSummary(taskList: TaskList): StatusSummary {
    let total = 0;
    let completed = 0;
    let inProgress = 0;
    let pending = 0;

    const countTasks = (tasks: TaskDetail[]) => {
      for (const task of tasks) {
        total++;
        if (task.status === 'completed') {
          completed++;
        } else if (task.status === 'in_progress') {
          inProgress++;
        } else {
          pending++;
        }
        if (task.subtasks) {
          countTasks(task.subtasks);
        }
      }
    };

    countTasks(taskList.tasks);

    const completionRate = total > 0 ? Math.round((completed / total) * 100) : 0;

    return {
      agent: taskList.agent,
      title: taskList.title,
      total,
      completed,
      inProgress,
      pending,
      completionRate
    };
  }

  /**
   * 마크다운 특수 문자 이스케이프
   */
  private escapeMarkdown(text: string): string {
    if (!text) return '';
    return text
      .replace(/\\/g, '\\\\')
      .replace(/\*/g, '\\*')
      .replace(/_/g, '\\_')
      .replace(/\[/g, '\\[')
      .replace(/\]/g, '\\]')
      .replace(/\(/g, '\\(')
      .replace(/\)/g, '\\)')
      .replace(/`/g, '\\`')
      .replace(/#/g, '\\#');
  }
}
