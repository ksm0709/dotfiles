// src/commands/remove.ts

import * as readline from 'readline';
import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';

export interface RemoveArgs {
  agent: string;
  id: string;
  force?: boolean;
}

export async function removeCommand(args: RemoveArgs): Promise<void> {
  const storage = new Storage();
  const parser = new Parser();

  try {
    const files = await storage.listTaskFiles(args.agent);
    
    if (files.length === 0) {
      console.log(`ℹ️ No task lists found for agent: ${args.agent}`);
      return;
    }

    // Try to find the task in any of the task lists
    for (const file of files) {
      const title = file.replace('.md', '');
      const content = await storage.readTaskList(args.agent, title);
      
      if (!content) continue;

      const taskList = parser.parseTaskList(content);
      
      // Find the task to get its title for confirmation
      const findTask = (tasks: any[]): any | null => {
        for (const task of tasks) {
          if (task.id === args.id) return task;
          if (task.subtasks) {
            const found = findTask(task.subtasks);
            if (found) return found;
          }
        }
        return null;
      };

      const taskToRemove = findTask(taskList.tasks);

      if (taskToRemove) {
        // Confirm if not forced
        if (!args.force) {
          const confirmed = await confirmRemoval(taskToRemove.title);
          if (!confirmed) {
            console.log('❌ Removal cancelled');
            return;
          }
        }

        const removed = parser.removeTask(taskList, args.id);

        if (removed) {
          // Save updated content
          const updatedContent = parser.generateTaskList(taskList);
          await storage.saveTaskList(args.agent, title, updatedContent);
          
          console.log(`✅ Task ${args.id} "${taskToRemove.title}" removed`);
          return;
        }
      }
    }

    console.log(`❌ Task ${args.id} not found`);
    process.exit(1);
  } catch (error) {
    console.error('❌ Failed to remove task:', error);
    process.exit(1);
  }
}

function confirmRemoval(taskTitle: string): Promise<boolean> {
  return new Promise((resolve) => {
    const rl = readline.createInterface({
      input: process.stdin,
      output: process.stdout
    });

    rl.question(`⚠️ Are you sure you want to remove task "${taskTitle}"? (y/N): `, (answer) => {
      rl.close();
      resolve(answer.toLowerCase() === 'y' || answer.toLowerCase() === 'yes');
    });
  });
}
