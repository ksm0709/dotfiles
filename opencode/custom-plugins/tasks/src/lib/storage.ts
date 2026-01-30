// src/lib/storage.ts

import * as fs from 'fs/promises';
import * as path from 'path';
import { TaskList } from '../types';

export class Storage {
  private tasksDir: string;

  constructor() {
    this.tasksDir = path.join(process.env.HOME || '~', '.config', 'opencode', 'tasks');
  }

  async ensureAgentDir(agent: string): Promise<string> {
    const agentDir = path.join(this.tasksDir, agent);
    try {
      await fs.mkdir(agentDir, { recursive: true });
    } catch (error) {
      console.error(`Failed to create directory: ${agentDir}`, error);
      throw error;
    }
    return agentDir;
  }

  async saveTaskList(agent: string, title: string, content: string): Promise<void> {
    const agentDir = await this.ensureAgentDir(agent);
    const fileName = this.sanitizeFileName(title) + '.md';
    const filePath = path.join(agentDir, fileName);
    
    try {
      await fs.writeFile(filePath, content, 'utf-8');
    } catch (error) {
      console.error(`Failed to write file: ${filePath}`, error);
      throw error;
    }
  }

  async readTaskList(agent: string, title: string): Promise<string | null> {
    const agentDir = path.join(this.tasksDir, agent);
    const fileName = this.sanitizeFileName(title) + '.md';
    const filePath = path.join(agentDir, fileName);
    
    try {
      return await fs.readFile(filePath, 'utf-8');
    } catch (error) {
      if ((error as NodeJS.ErrnoException).code === 'ENOENT') {
        return null;
      }
      throw error;
    }
  }

  async listTaskFiles(agent: string): Promise<string[]> {
    const agentDir = path.join(this.tasksDir, agent);
    
    try {
      const files = await fs.readdir(agentDir);
      return files.filter(f => f.endsWith('.md'));
    } catch (error) {
      if ((error as NodeJS.ErrnoException).code === 'ENOENT') {
        return [];
      }
      throw error;
    }
  }

  async deleteTaskList(agent: string, title: string): Promise<void> {
    const agentDir = path.join(this.tasksDir, agent);
    const fileName = this.sanitizeFileName(title) + '.md';
    const filePath = path.join(agentDir, fileName);
    
    try {
      await fs.unlink(filePath);
    } catch (error) {
      if ((error as NodeJS.ErrnoException).code !== 'ENOENT') {
        throw error;
      }
    }
  }

  private sanitizeFileName(title: string): string {
    return title
      .toLowerCase()
      .replace(/[^a-z0-9가-힣\s-]/g, '')
      .replace(/\s+/g, '-')
      .substring(0, 50);
  }
}
