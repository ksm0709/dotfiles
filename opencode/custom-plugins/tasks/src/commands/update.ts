// src/commands/update.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { TaskStatus } from '../types';

export interface UpdateArgs {
  agent: string;
  id: string;
  status: TaskStatus;
}

export async function updateCommand(args: UpdateArgs): Promise<void> {
  const storage = new Storage();
  const parser = new Parser();

  try {
    const files = await storage.listTaskFiles(args.agent);
    
    if (files.length === 0) {
      console.log(`ℹ️ No task lists found for agent: ${args.agent}`);
      return;
    }

    // Try to find and update the task in any of the task lists
    for (const file of files) {
      const title = file.replace('.md', '');
      const content = await storage.readTaskList(args.agent, title);
      
      if (!content) continue;

      const taskList = parser.parseTaskList(content);
      const updated = parser.updateTaskStatus(taskList, args.id, args.status);

      if (updated) {
        // Save updated content
        const updatedContent = parser.generateTaskList(taskList);
        await storage.saveTaskList(args.agent, title, updatedContent);
        
        console.log(`✅ Task ${args.id} status updated to: ${args.status}`);
        return;
      }
    }

    console.log(`❌ Task ${args.id} not found`);
    process.exit(1);
  } catch (error) {
    console.error('❌ Failed to update task:', error);
    process.exit(1);
  }
}
