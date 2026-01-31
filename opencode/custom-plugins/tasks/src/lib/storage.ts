// src/lib/storage.ts

import * as fs from 'fs/promises';
import * as path from 'path';
import { TaskList } from '../types';

export class Storage {
  private tasksDir: string;

  constructor() {
    // XDG Base Directory Specification 준수
    // 데이터 파일은 ~/.local/share/opencode/tasks/에 저장
    // constructor에서 계산하여 테스트 시 환경변수 변경 가능
    this.tasksDir = process.env.XDG_DATA_HOME
      ? path.join(process.env.XDG_DATA_HOME, 'opencode', 'tasks')
      : path.join(process.env.HOME || '~', '.local', 'share', 'opencode', 'tasks');
  }

  /**
   * 공통 에러 핸들러
   */
  private handleError(operation: string, path: string, error: unknown): never {
    console.error(`❌ Storage error [${operation}]: ${path}`, error);
    throw error;
  }

  /**
   * 타입 가드: NodeJS 에러인지 확인
   * instanceof Error를 사용하지 않음 - Jest 환경에서 VM 컨텍스트 문제 방지
   */
  private isNodeError(error: unknown): error is NodeJS.ErrnoException {
    return (
      typeof error === 'object' &&
      error !== null &&
      'code' in error &&
      typeof (error as NodeJS.ErrnoException).code === 'string'
    );
  }

  /**
   * 세션별 디렉토리 생성
   * ~/.local/share/opencode/tasks/{sessionId}/
   */
  async ensureSessionDir(sessionId: string): Promise<string> {
    const sessionDir = path.join(this.tasksDir, sessionId);
    try {
      await fs.mkdir(sessionDir, { recursive: true });
    } catch (error) {
      this.handleError('mkdir', sessionDir, error);
    }
    return sessionDir;
  }

  /**
   * 작업 목록 저장
   * ~/.local/share/opencode/tasks/{sessionId}/{title}.md
   */
  async saveTaskList(sessionId: string, title: string, content: string): Promise<void> {
    const sessionDir = await this.ensureSessionDir(sessionId);
    const fileName = this.sanitizeFileName(title) + '.md';
    const filePath = path.join(sessionDir, fileName);

    try {
      await fs.writeFile(filePath, content, 'utf-8');
    } catch (error) {
      this.handleError('write', filePath, error);
    }
  }

  /**
   * 작업 목록 읽기
   */
  async readTaskList(sessionId: string, title: string): Promise<string | null> {
    const sessionDir = path.join(this.tasksDir, sessionId);
    const fileName = this.sanitizeFileName(title) + '.md';
    const filePath = path.join(sessionDir, fileName);

    try {
      return await fs.readFile(filePath, 'utf-8');
    } catch (error) {
      if (this.isNodeError(error) && error.code === 'ENOENT') {
        return null;
      }
      throw error;
    }
  }

  /**
   * 세션의 모든 작업 목록 파일 조회
   */
  async listTaskFiles(sessionId: string): Promise<string[]> {
    const sessionDir = path.join(this.tasksDir, sessionId);

    try {
      const files = await fs.readdir(sessionDir);
      return files.filter(f => f.endsWith('.md'));
    } catch (error) {
      if (this.isNodeError(error) && error.code === 'ENOENT') {
        return [];
      }
      throw error;
    }
  }

  /**
   * 작업 목록 삭제
   */
  async deleteTaskList(sessionId: string, title: string): Promise<void> {
    const sessionDir = path.join(this.tasksDir, sessionId);
    const fileName = this.sanitizeFileName(title) + '.md';
    const filePath = path.join(sessionDir, fileName);

    try {
      await fs.unlink(filePath);
    } catch (error) {
      if (this.isNodeError(error) && error.code !== 'ENOENT') {
        throw error;
      }
    }
  }

  /**
   * 파일명에서 특수문자 제거 (안전한 파일명 생성)
   */
  private sanitizeFileName(title: string): string {
    return title
      .toLowerCase()
      .replace(/[^a-z0-9가-힣\s-]/g, '')
      .replace(/\s+/g, '-')
      .substring(0, 50);
  }

  /**
   * 전체 저장 경로 반환 (디버깅용)
   */
  getTasksDir(): string {
    return this.tasksDir;
  }
}
