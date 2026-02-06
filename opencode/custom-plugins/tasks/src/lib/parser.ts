// src/lib/parser.ts

import { TaskList, TaskDetail, TaskStatus } from '../types';

export class Parser {
  parseTaskList(content: string): TaskList {
    const lines = content.split('\n');
    const taskList: TaskList = {
      title: '',
      agent: '',
      createdAt: '',
      sessionId: '',
      tasks: []
    };

    let currentSection: 'header' | 'tasks' | 'summary' | 'changelog' = 'header';
    let currentTask: TaskDetail | null = null;

    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];

      // Parse header
      if (line.startsWith('# Task List:')) {
        taskList.title = line.replace('# Task List:', '').trim();
      } else if (line.startsWith('**에이전트**:')) {
        taskList.agent = line.replace('**에이전트**:', '').trim();
      } else if (line.startsWith('**생성일**:')) {
        taskList.createdAt = line.replace('**생성일**:', '').trim();
      } else if (line.startsWith('**세션 ID**:')) {
        taskList.sessionId = line.replace('**세션 ID**:', '').trim();
      } else if (line.startsWith('## 작업 목록')) {
        currentSection = 'tasks';
      } else if (line.startsWith('## 진행 상황 요약')) {
        currentSection = 'summary';
      } else if (line.startsWith('## 변경 이력')) {
        currentSection = 'changelog';
      }

      // Parse tasks - flat structure only
      if (currentSection === 'tasks') {
        const taskMatch = line.match(/^- \[([ x~])\] ((?:\d+\.)+\d*\.?\s*.+)$/);
        const detailMatch = line.match(/^\s+- (.+)$/);

        if (taskMatch) {
          const checkbox = taskMatch[1];
          const title = taskMatch[2].trim();
          const id = title.match(/^((?:\d+\.)+\d*)/)?.[1]?.replace(/\.$/, '') || '';

          // Determine status based on checkbox character
          let status: TaskStatus;
          if (checkbox === 'x') {
            status = 'completed';
          } else if (checkbox === '~') {
            status = 'in_progress';
          } else {
            status = 'pending';
          }

          const task: TaskDetail = {
            id,
            title: title.replace(/^(?:\d+\.)+\s*/, ''),
            status,
            details: [],
            createdAt: new Date().toISOString(),
            updatedAt: new Date().toISOString()
          };

          taskList.tasks.push(task);
          currentTask = task;
        } else if (detailMatch && currentTask) {
          const detail = detailMatch[1].trim();
          // 상세 정보로 인식 (ID 패턴이 아닌 경우)
          if (!detail.match(/^(?:\d+\.)+\d+/)) {
            currentTask.details.push(detail);
          }
        }
      }

      // Parse summary
      if (currentSection === 'summary') {
        if (line.startsWith('**현재 단계**:')) {
          taskList.currentPhase = line.replace('**현재 단계**:', '').trim();
        } else if (line.startsWith('**메모**:')) {
          taskList.memo = line.replace('**메모**:', '').trim();
        }
      }
    }

    return taskList;
  }

  generateTaskList(taskList: TaskList): string {
    const lines: string[] = [];

    // Header
    lines.push(`# Task List: ${taskList.title}`);
    lines.push('');
    lines.push(`**에이전트**: ${taskList.agent}  `);
    lines.push(`**생성일**: ${taskList.createdAt}  `);
    lines.push(`**세션 ID**: ${taskList.sessionId}`);
    lines.push('');
    lines.push('---');
    lines.push('');

    // Tasks - flat structure
    lines.push('## 작업 목록 (Task List)');
    lines.push('');

    for (const task of taskList.tasks) {
      lines.push(...this.formatTask(task));
    }

    lines.push('');
    lines.push('---');
    lines.push('');

    // Summary
    lines.push('## 진행 상황 요약 (Progress Summary)');
    lines.push('');
    lines.push(`**현재 단계**: ${taskList.currentPhase || '미정'}  `);
    
    const stats = this.calculateStats(taskList.tasks);
    lines.push(`**상태**: ${stats.status}  `);
    lines.push(`**완료율**: ${stats.completionRate}% (${stats.completedCount}/${stats.totalCount})`);
    if (taskList.memo) {
      lines.push(`**메모**: ${taskList.memo}`);
    }

    lines.push('');
    lines.push('---');
    lines.push('');

    // Change log placeholder
    lines.push('## 변경 이력 (Change Log)');
    lines.push('');
    lines.push('| 시간 | 작업 | 상태 |');
    lines.push('|------|------|------|');
    lines.push(`| ${new Date().toISOString().slice(0, 16).replace('T', ' ')} | 작업 목록 생성 | created |`);
    lines.push('');

    return lines.join('\n');
  }

  private formatTask(task: TaskDetail): string[] {
    const lines: string[] = [];
    // Generate checkbox based on status: completed -> [x], in_progress -> [~], pending -> [ ]
    const checkbox = task.status === 'completed' ? '[x]' : 
                    task.status === 'in_progress' ? '[~]' : '[ ]';
    
    lines.push(`- ${checkbox} ${task.id}. ${task.title}`);

    // Add details
    for (const detail of task.details) {
      lines.push(`  - ${detail}`);
    }

    return lines;
  }

  private calculateStats(tasks: TaskDetail[]): { status: string; completionRate: number; completedCount: number; totalCount: number } {
    let total = tasks.length;
    let completed = 0;
    let inProgress = 0;

    for (const task of tasks) {
      if (task.status === 'completed') {
        completed++;
      } else if (task.status === 'in_progress') {
        inProgress++;
      }
    }

    const completionRate = total > 0 ? Math.round((completed / total) * 100) : 0;
    let status = 'pending';
    if (completed === total && total > 0) {
      status = 'completed';
    } else if (inProgress > 0 || completed > 0) {
      status = 'in_progress';
    }

    return { status, completionRate, completedCount: completed, totalCount: total };
  }

  updateTaskStatus(taskList: TaskList, taskId: string, status: TaskStatus): boolean {
    for (const task of taskList.tasks) {
      if (task.id === taskId) {
        task.status = status;
        task.updatedAt = new Date().toISOString();
        return true;
      }
    }
    return false;
  }

  addTask(taskList: TaskList, title: string, details: string[]): boolean {
    const newTask: TaskDetail = {
      id: '',
      title,
      status: 'pending',
      details,
      createdAt: new Date().toISOString(),
      updatedAt: new Date().toISOString()
    };

    // Flat structure: generate ID based on existing tasks
    const existingIds = taskList.tasks.map(t => t.id);
    const maxId = existingIds.length > 0 
      ? Math.max(...existingIds.map(id => {
          const parts = id.split('.');
          return parts.length > 0 ? parseInt(parts[0]) : 0;
        }))
      : 0;
    
    newTask.id = `${maxId + 1}`;
    taskList.tasks.push(newTask);
    return true;
  }

  removeTask(taskList: TaskList, taskId: string): boolean {
    const index = taskList.tasks.findIndex(t => t.id === taskId);
    if (index !== -1) {
      taskList.tasks.splice(index, 1);
      return true;
    }
    return false;
  }
}
