// src/commands/complete.ts

import { updateCommand } from './update';

export interface CompleteArgs {
  agent: string;
  id: string;
}

export async function completeCommand(args: CompleteArgs): Promise<void> {
  // Complete is just a special case of update with status 'completed'
  await updateCommand({
    agent: args.agent,
    id: args.id,
    status: 'completed'
  });
  
  console.log(`🎉 Task ${args.id} marked as completed!`);
}
