// src/commands/add-task.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';

export interface AddTaskArgs {
  agent: string;
  parent?: string;
  title: string;
  details?: string;
}

export async function addTaskCommand(args: AddTaskArgs): Promise<void> {
  const storage = new Storage();
  const parser = new Parser();

  try {
    const files = await storage.listTaskFiles(args.agent);
    
    if (files.length === 0) {
      console.log(`ℹ️ No task lists found for agent: ${args.agent}`);
      console.log('💡 Create a task list first with: tasks init');
      return;
    }

    // For simplicity, add to the first task list found
    // In a more complex implementation, you might want to specify which task list
    const file = files[0];
    const listTitle = file.replace('.md', '');
    const content = await storage.readTaskList(args.agent, listTitle);
    
    if (!content) {
      console.log('❌ Failed to read task list');
      process.exit(1);
    }

    const taskList = parser.parseTaskList(content);
    
    // Parse details from comma-separated string
    const details = args.details ? args.details.split(',').map(d => d.trim()).filter(d => d) : [];

    const added = parser.addTask(taskList, args.parent, args.title, details);

    if (added) {
      // Save updated content
      const updatedContent = parser.generateTaskList(taskList);
      await storage.saveTaskList(args.agent, listTitle, updatedContent);
      
      console.log(`✅ Task added: ${args.title}`);
      if (details.length > 0) {
        console.log(`📝 Details: ${details.join(', ')}`);
      }
    } else {
      console.log(`❌ Failed to add task. Parent task ${args.parent} not found.`);
      process.exit(1);
    }
  } catch (error) {
    console.error('❌ Failed to add task:', error);
    process.exit(1);
  }
}
