// src/commands/list.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { Formatter } from '../lib/formatter';

export interface ListArgs {
  agent: string;
  format?: 'markdown' | 'json' | 'table';
}

export async function listCommand(args: ListArgs): Promise<void> {
  const storage = new Storage();
  const parser = new Parser();
  const formatter = new Formatter();

  try {
    const files = await storage.listTaskFiles(args.agent);
    
    if (files.length === 0) {
      console.log(`ℹ️ No task lists found for agent: ${args.agent}`);
      return;
    }

    const format = args.format || 'markdown';

    for (const file of files) {
      const title = file.replace('.md', '');
      const content = await storage.readTaskList(args.agent, title);
      
      if (!content) continue;

      const taskList = parser.parseTaskList(content);

      switch (format) {
        case 'json':
          console.log(formatter.formatAsJSON(taskList));
          break;
        case 'table':
          console.log(formatter.formatAsTable(taskList));
          break;
        case 'markdown':
        default:
          console.log(formatter.formatAsMarkdown(taskList));
          break;
      }
      
      console.log('\n---\n');
    }
  } catch (error) {
    console.error('❌ Failed to list tasks:', error);
    process.exit(1);
  }
}
