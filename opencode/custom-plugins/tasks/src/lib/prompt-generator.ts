// src/lib/prompt-generator.ts

import { TaskDetail } from '../types';

export class PromptGenerator {
  /**
   * 미완료 작업 알림을 위한 한국어 프롬프트 생성
   */
  generateIncompleteTaskPrompt(
    incompleteTasks: TaskDetail[],
    summary: { pending: number; inProgress: number; completed: number }
  ): string {
    let prompt = `⚠️ **작업 완료 알림**\n\n`;
    prompt += `현재 세션에 완료되지 않은 작업이 있습니다.\n\n`;
    
    // 상태 요약 (이모지 형식)
    prompt += `🔄 진행 중: ${summary.inProgress}개\n`;
    prompt += `⏳ 대기 중: ${summary.pending}개\n`;
    prompt += `✅ 완료됨: ${summary.completed}개\n\n`;
    
    // 미완료 작업 목록
    prompt += `**남은 작업 목록:**\n`;
    
    if (incompleteTasks.length > 0) {
      for (const task of incompleteTasks) {
        const statusEmoji = task.status === 'in_progress' ? '🔄' : '⏳';
        const statusText = task.status === 'in_progress' ? '진행 중' : '대기 중';
        prompt += `${statusEmoji} **${task.id}**. ${task.title} (${statusText})\n`;
      }
    }
    
    prompt += `\n`;
    
    // 완료 요청 메시지
    prompt += `📌 **이 작업들을 완료해주세요.**\n`;
    prompt += '완료 후 `tasks` 도구를 사용하여 상태를 업데이트하세요.';
    
    return prompt;
  }
}
