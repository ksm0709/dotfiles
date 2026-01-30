// src/commands/init.ts

import * as fs from 'fs/promises';
import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { TaskList, TaskDetail } from '../types';

export interface InitArgs {
  agent: string;
  title: string;
  file: string;
}

export async function initCommand(args: InitArgs): Promise<void> {
  const storage = new Storage();
  const parser = new Parser();

  try {
    // Read source tasks.md file
    const content = await fs.readFile(args.file, 'utf-8');
    
    // Parse the tasks.md to extract task structure
    const taskList = parseTasksFile(content, args.agent, args.title);
    
    // Generate markdown content
    const markdownContent = parser.generateTaskList(taskList);
    
    // Save to storage
    await storage.saveTaskList(args.agent, args.title, markdownContent);
    
    console.log(`✅ Task list initialized: ${args.title}`);
    console.log(`📁 Location: ~/.config/opencode/tasks/${args.agent}/${args.title}.md`);
    console.log(`📊 Total tasks: ${countTasks(taskList.tasks)}`);
  } catch (error) {
    console.error('❌ Failed to initialize task list:', error);
    process.exit(1);
  }
}

function parseTasksFile(content: string, agent: string, title: string): TaskList {
  const lines = content.split('\n');
  const tasks: TaskDetail[] = [];
  let currentTask: TaskDetail | null = null;
  let currentParent: TaskDetail | null = null;

  for (const line of lines) {
    // Match task lines (e.g., "- [ ] 1. 작업 이름" or "- [x] 1.1. 하위 작업")
    const taskMatch = line.match(/^(\s*)- \[([ x])\] ((\d+(?:\.\d+)*)\.\s*(.+))$/);
    
    if (taskMatch) {
      const indent = taskMatch[1].length;
      const isChecked = taskMatch[2] === 'x';
      const id = taskMatch[4];
      const taskTitle = taskMatch[5].trim();

      const task: TaskDetail = {
        id,
        title: taskTitle,
        status: isChecked ? 'completed' : 'pending',
        details: [],
        subtasks: [],
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString()
      };

      if (indent === 0) {
        // Top-level task
        tasks.push(task);
        currentParent = task;
      } else if (currentParent && id.includes('.')) {
        // Subtask
        currentParent.subtasks = currentParent.subtasks || [];
        currentParent.subtasks.push(task);
      }

      currentTask = task;
    } else if (currentTask && line.match(/^\s{2,}- (.+)$/)) {
      // Detail line (indented bullet point that's not a task)
      const detailMatch = line.match(/^\s{2,}- (.+)$/);
      if (detailMatch && !detailMatch[1].match(/^\d+\.\d+/)) {
        currentTask.details.push(detailMatch[1].trim());
      }
    }
  }

  return {
    title,
    agent,
    createdAt: new Date().toISOString(),
    sessionId: generateSessionId(),
    tasks,
    currentPhase: tasks.length > 0 ? tasks[0].title : undefined
  };
}

function countTasks(tasks: TaskDetail[]): number {
  let count = 0;
  for (const task of tasks) {
    count++;
    if (task.subtasks) {
      count += countTasks(task.subtasks);
    }
  }
  return count;
}

function generateSessionId(): string {
  return Math.random().toString(36).substring(2, 15) + 
         Math.random().toString(36).substring(2, 15);
}
