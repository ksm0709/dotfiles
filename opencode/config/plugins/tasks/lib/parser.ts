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
    let currentParent: TaskDetail | null = null;
    let taskStack: TaskDetail[] = [];

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

      // Parse tasks
      if (currentSection === 'tasks') {
        const taskMatch = line.match(/^(\s*)- \[([ x])\] (\d+\.\s*.+)$/);
        const detailMatch = line.match(/^(\s{2,})- (.+)$/);

        if (taskMatch) {
          const indent = taskMatch[1].length;
          const isChecked = taskMatch[2] === 'x';
          const title = taskMatch[3].trim();
          const id = title.match(/^(\d+(?:\.\d+)*)/)?.[1] || '';

          const task: TaskDetail = {
            id,
            title: title.replace(/^\d+\.\s*/, ''),
            status: isChecked ? 'completed' : 'pending',
            details: [],
            subtasks: [],
            createdAt: new Date().toISOString(),
            updatedAt: new Date().toISOString()
          };

          if (indent === 0) {
            taskList.tasks.push(task);
            currentParent = task;
            taskStack = [task];
          } else if (currentParent && indent >= 2) {
            currentParent.subtasks = currentParent.subtasks || [];
            currentParent.subtasks.push(task);
          }

          currentTask = task;
        } else if (detailMatch && currentTask) {
          const detail = detailMatch[2].trim();
          if (!detail.match(/^\d+\.\d+/)) {
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

    // Tasks
    lines.push('## 작업 목록 (Task List)');
    lines.push('');

    for (const task of taskList.tasks) {
      lines.push(...this.formatTask(task, 0));
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

  private formatTask(task: TaskDetail, indent: number): string[] {
    const lines: string[] = [];
    const prefix = '  '.repeat(indent);
    const checkbox = task.status === 'completed' ? '[x]' : '[ ]';
    
    lines.push(`${prefix}- ${checkbox} ${task.id}. ${task.title}`);

    // Add details
    for (const detail of task.details) {
      lines.push(`${prefix}  - ${detail}`);
    }

    // Add subtasks
    if (task.subtasks) {
      for (const subtask of task.subtasks) {
        lines.push(...this.formatTask(subtask, indent + 1));
      }
    }

    return lines;
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

  updateTaskStatus(taskList: TaskList, taskId: string, status: TaskStatus): boolean {
    const findAndUpdate = (tasks: TaskDetail[]): boolean => {
      for (const task of tasks) {
        if (task.id === taskId) {
          task.status = status;
          task.updatedAt = new Date().toISOString();
          return true;
        }
        if (task.subtasks && findAndUpdate(task.subtasks)) {
          return true;
        }
      }
      return false;
    };

    return findAndUpdate(taskList.tasks);
  }

  addTask(taskList: TaskList, parentId: string | undefined, title: string, details: string[]): boolean {
    const newTask: TaskDetail = {
      id: '',
      title,
      status: 'pending',
      details,
      createdAt: new Date().toISOString(),
      updatedAt: new Date().toISOString()
    };

    if (!parentId) {
      // Add as top-level task
      const maxId = Math.max(...taskList.tasks.map(t => {
        const match = t.id.match(/^(\d+)/);
        return match ? parseInt(match[1]) : 0;
      }), 0);
      newTask.id = `${maxId + 1}`;
      taskList.tasks.push(newTask);
      return true;
    }

    // Add as subtask
    const findParent = (tasks: TaskDetail[]): TaskDetail | null => {
      for (const task of tasks) {
        if (task.id === parentId) {
          return task;
        }
        if (task.subtasks) {
          const found = findParent(task.subtasks);
          if (found) return found;
        }
      }
      return null;
    };

    const parent = findParent(taskList.tasks);
    if (parent) {
      parent.subtasks = parent.subtasks || [];
      const maxSubId = Math.max(...parent.subtasks.map(t => {
        const match = t.id.match(/\.(\d+)$/);
        return match ? parseInt(match[1]) : 0;
      }), 0);
      newTask.id = `${parentId}.${maxSubId + 1}`;
      parent.subtasks.push(newTask);
      return true;
    }

    return false;
  }

  removeTask(taskList: TaskList, taskId: string): boolean {
    const findAndRemove = (tasks: TaskDetail[]): boolean => {
      const index = tasks.findIndex(t => t.id === taskId);
      if (index !== -1) {
        tasks.splice(index, 1);
        return true;
      }
      for (const task of tasks) {
        if (task.subtasks && findAndRemove(task.subtasks)) {
          return true;
        }
      }
      return false;
    };

    return findAndRemove(taskList.tasks);
  }
}
