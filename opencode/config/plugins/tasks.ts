#!/usr/bin/env node

// src/index.ts
import { tool } from "@opencode-ai/plugin";

// src/lib/storage.ts
import * as fs from "fs/promises";
import * as path from "path";
var Storage = class {
  constructor() {
    this.tasksDir = process.env.XDG_DATA_HOME ? path.join(process.env.XDG_DATA_HOME, "opencode", "tasks") : path.join(process.env.HOME || "~", ".local", "share", "opencode", "tasks");
  }
  /**
   * 공통 에러 핸들러
   */
  handleError(operation, path2, error) {
    console.error(`\u274C Storage error [${operation}]: ${path2}`, error);
    throw error;
  }
  /**
   * 타입 가드: NodeJS 에러인지 확인
   * instanceof Error를 사용하지 않음 - Jest 환경에서 VM 컨텍스트 문제 방지
   */
  isNodeError(error) {
    return typeof error === "object" && error !== null && "code" in error && typeof error.code === "string";
  }
  /**
   * 세션별 디렉토리 생성
   * ~/.local/share/opencode/tasks/{sessionId}/
   */
  async ensureSessionDir(sessionId) {
    const sessionDir = path.join(this.tasksDir, sessionId);
    try {
      await fs.mkdir(sessionDir, { recursive: true });
    } catch (error) {
      this.handleError("mkdir", sessionDir, error);
    }
    return sessionDir;
  }
  /**
   * 작업 목록 저장
   * ~/.local/share/opencode/tasks/{sessionId}/{title}.md
   */
  async saveTaskList(sessionId, title, content) {
    const sessionDir = await this.ensureSessionDir(sessionId);
    const fileName = this.sanitizeFileName(title) + ".md";
    const filePath = path.join(sessionDir, fileName);
    try {
      await fs.writeFile(filePath, content, "utf-8");
    } catch (error) {
      this.handleError("write", filePath, error);
    }
  }
  /**
   * 작업 목록 읽기
   */
  async readTaskList(sessionId, title) {
    const sessionDir = path.join(this.tasksDir, sessionId);
    const fileName = this.sanitizeFileName(title) + ".md";
    const filePath = path.join(sessionDir, fileName);
    try {
      return await fs.readFile(filePath, "utf-8");
    } catch (error) {
      if (this.isNodeError(error) && error.code === "ENOENT") {
        return null;
      }
      throw error;
    }
  }
  /**
   * 세션의 모든 작업 목록 파일 조회
   */
  async listTaskFiles(sessionId) {
    const sessionDir = path.join(this.tasksDir, sessionId);
    try {
      const files = await fs.readdir(sessionDir);
      return files.filter((f) => f.endsWith(".md"));
    } catch (error) {
      if (this.isNodeError(error) && error.code === "ENOENT") {
        return [];
      }
      throw error;
    }
  }
  /**
   * 작업 목록 삭제
   */
  async deleteTaskList(sessionId, title) {
    const sessionDir = path.join(this.tasksDir, sessionId);
    const fileName = this.sanitizeFileName(title) + ".md";
    const filePath = path.join(sessionDir, fileName);
    try {
      await fs.unlink(filePath);
    } catch (error) {
      if (this.isNodeError(error) && error.code !== "ENOENT") {
        throw error;
      }
    }
  }
  /**
   * 파일명에서 특수문자 제거 (안전한 파일명 생성)
   */
  sanitizeFileName(title) {
    return title.toLowerCase().replace(/[^a-z0-9가-힣\s-]/g, "").replace(/\s+/g, "-").substring(0, 50);
  }
  /**
   * 전체 저장 경로 반환 (디버깅용)
   */
  getTasksDir() {
    return this.tasksDir;
  }
};

// src/lib/parser.ts
var Parser = class {
  parseTaskList(content) {
    const lines = content.split("\n");
    const taskList = {
      title: "",
      agent: "",
      createdAt: "",
      sessionId: "",
      tasks: []
    };
    let currentSection = "header";
    let currentTask = null;
    let currentParent = null;
    let taskStack = [];
    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];
      if (line.startsWith("# Task List:")) {
        taskList.title = line.replace("# Task List:", "").trim();
      } else if (line.startsWith("**\uC5D0\uC774\uC804\uD2B8**:")) {
        taskList.agent = line.replace("**\uC5D0\uC774\uC804\uD2B8**:", "").trim();
      } else if (line.startsWith("**\uC0DD\uC131\uC77C**:")) {
        taskList.createdAt = line.replace("**\uC0DD\uC131\uC77C**:", "").trim();
      } else if (line.startsWith("**\uC138\uC158 ID**:")) {
        taskList.sessionId = line.replace("**\uC138\uC158 ID**:", "").trim();
      } else if (line.startsWith("## \uC791\uC5C5 \uBAA9\uB85D")) {
        currentSection = "tasks";
      } else if (line.startsWith("## \uC9C4\uD589 \uC0C1\uD669 \uC694\uC57D")) {
        currentSection = "summary";
      } else if (line.startsWith("## \uBCC0\uACBD \uC774\uB825")) {
        currentSection = "changelog";
      }
      if (currentSection === "tasks") {
        const taskMatch = line.match(/^(\s*)- \[([ x])\] (\d+\.\s*.+)$/);
        const detailMatch = line.match(/^(\s{2,})- (.+)$/);
        if (taskMatch) {
          const indent = taskMatch[1].length;
          const isChecked = taskMatch[2] === "x";
          const title = taskMatch[3].trim();
          const id = title.match(/^(\d+(?:\.\d+)*)/)?.[1] || "";
          const task = {
            id,
            title: title.replace(/^\d+\.\s*/, ""),
            status: isChecked ? "completed" : "pending",
            details: [],
            subtasks: [],
            createdAt: (/* @__PURE__ */ new Date()).toISOString(),
            updatedAt: (/* @__PURE__ */ new Date()).toISOString()
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
      if (currentSection === "summary") {
        if (line.startsWith("**\uD604\uC7AC \uB2E8\uACC4**:")) {
          taskList.currentPhase = line.replace("**\uD604\uC7AC \uB2E8\uACC4**:", "").trim();
        } else if (line.startsWith("**\uBA54\uBAA8**:")) {
          taskList.memo = line.replace("**\uBA54\uBAA8**:", "").trim();
        }
      }
    }
    return taskList;
  }
  generateTaskList(taskList) {
    const lines = [];
    lines.push(`# Task List: ${taskList.title}`);
    lines.push("");
    lines.push(`**\uC5D0\uC774\uC804\uD2B8**: ${taskList.agent}  `);
    lines.push(`**\uC0DD\uC131\uC77C**: ${taskList.createdAt}  `);
    lines.push(`**\uC138\uC158 ID**: ${taskList.sessionId}`);
    lines.push("");
    lines.push("---");
    lines.push("");
    lines.push("## \uC791\uC5C5 \uBAA9\uB85D (Task List)");
    lines.push("");
    for (const task of taskList.tasks) {
      lines.push(...this.formatTask(task, 0));
    }
    lines.push("");
    lines.push("---");
    lines.push("");
    lines.push("## \uC9C4\uD589 \uC0C1\uD669 \uC694\uC57D (Progress Summary)");
    lines.push("");
    lines.push(`**\uD604\uC7AC \uB2E8\uACC4**: ${taskList.currentPhase || "\uBBF8\uC815"}  `);
    const stats = this.calculateStats(taskList.tasks);
    lines.push(`**\uC0C1\uD0DC**: ${stats.status}  `);
    lines.push(`**\uC644\uB8CC\uC728**: ${stats.completionRate}% (${stats.completedCount}/${stats.totalCount})`);
    if (taskList.memo) {
      lines.push(`**\uBA54\uBAA8**: ${taskList.memo}`);
    }
    lines.push("");
    lines.push("---");
    lines.push("");
    lines.push("## \uBCC0\uACBD \uC774\uB825 (Change Log)");
    lines.push("");
    lines.push("| \uC2DC\uAC04 | \uC791\uC5C5 | \uC0C1\uD0DC |");
    lines.push("|------|------|------|");
    lines.push(`| ${(/* @__PURE__ */ new Date()).toISOString().slice(0, 16).replace("T", " ")} | \uC791\uC5C5 \uBAA9\uB85D \uC0DD\uC131 | created |`);
    lines.push("");
    return lines.join("\n");
  }
  formatTask(task, indent) {
    const lines = [];
    const prefix = "  ".repeat(indent);
    const checkbox = task.status === "completed" ? "[x]" : "[ ]";
    lines.push(`${prefix}- ${checkbox} ${task.id}. ${task.title}`);
    for (const detail of task.details) {
      lines.push(`${prefix}  - ${detail}`);
    }
    if (task.subtasks) {
      for (const subtask of task.subtasks) {
        lines.push(...this.formatTask(subtask, indent + 1));
      }
    }
    return lines;
  }
  calculateStats(tasks) {
    let total = 0;
    let completed = 0;
    let inProgress = 0;
    const countTasks2 = (taskList) => {
      for (const task of taskList) {
        total++;
        if (task.status === "completed") {
          completed++;
        } else if (task.status === "in_progress") {
          inProgress++;
        }
        if (task.subtasks) {
          countTasks2(task.subtasks);
        }
      }
    };
    countTasks2(tasks);
    const completionRate = total > 0 ? Math.round(completed / total * 100) : 0;
    let status = "pending";
    if (completed === total && total > 0) {
      status = "completed";
    } else if (inProgress > 0 || completed > 0) {
      status = "in_progress";
    }
    return { status, completionRate, completedCount: completed, totalCount: total };
  }
  updateTaskStatus(taskList, taskId, status) {
    const findAndUpdate = (tasks) => {
      for (const task of tasks) {
        if (task.id === taskId) {
          task.status = status;
          task.updatedAt = (/* @__PURE__ */ new Date()).toISOString();
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
  addTask(taskList, parentId, title, details) {
    const newTask = {
      id: "",
      title,
      status: "pending",
      details,
      createdAt: (/* @__PURE__ */ new Date()).toISOString(),
      updatedAt: (/* @__PURE__ */ new Date()).toISOString()
    };
    if (!parentId) {
      const maxId = Math.max(...taskList.tasks.map((t) => {
        const match = t.id.match(/^(\d+)/);
        return match ? parseInt(match[1]) : 0;
      }), 0);
      newTask.id = `${maxId + 1}`;
      taskList.tasks.push(newTask);
      return true;
    }
    const findParent = (tasks) => {
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
      const maxSubId = Math.max(...parent.subtasks.map((t) => {
        const match = t.id.match(/\.(\d+)$/);
        return match ? parseInt(match[1]) : 0;
      }), 0);
      newTask.id = `${parentId}.${maxSubId + 1}`;
      parent.subtasks.push(newTask);
      return true;
    }
    return false;
  }
  removeTask(taskList, taskId) {
    const findAndRemove = (tasks) => {
      const index = tasks.findIndex((t) => t.id === taskId);
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
};

// src/lib/formatter.ts
var Formatter = class {
  formatAsMarkdown(taskList) {
    const lines = [];
    lines.push(`# Task List: ${taskList.title}`);
    lines.push("");
    lines.push(`**\uC5D0\uC774\uC804\uD2B8**: ${taskList.agent}`);
    lines.push(`**\uC0DD\uC131\uC77C**: ${taskList.createdAt}`);
    lines.push(`**\uC138\uC158 ID**: ${taskList.sessionId}`);
    lines.push("");
    lines.push("---");
    lines.push("");
    lines.push("## \uC791\uC5C5 \uBAA9\uB85D");
    lines.push("");
    for (const task of taskList.tasks) {
      lines.push(...this.formatTaskMarkdown(task, 0));
    }
    lines.push("");
    lines.push("---");
    lines.push("");
    lines.push("## \uC9C4\uD589 \uC0C1\uD669 \uC694\uC57D");
    lines.push("");
    const stats = this.calculateStats(taskList.tasks);
    lines.push(`**\uD604\uC7AC \uB2E8\uACC4**: ${taskList.currentPhase || "\uBBF8\uC815"}`);
    lines.push(`**\uC0C1\uD0DC**: ${stats.status}`);
    lines.push(`**\uC644\uB8CC\uC728**: ${stats.completionRate}% (${stats.completedCount}/${stats.totalCount})`);
    if (taskList.memo) {
      lines.push(`**\uBA54\uBAA8**: ${taskList.memo}`);
    }
    return lines.join("\n");
  }
  formatTaskMarkdown(task, indent) {
    const lines = [];
    const prefix = "  ".repeat(indent);
    const checkbox = task.status === "completed" ? "[x]" : "[ ]";
    const statusEmoji = task.status === "completed" ? "\u2705" : task.status === "in_progress" ? "\u{1F504}" : "\u23F3";
    lines.push(`${prefix}- ${checkbox} ${statusEmoji} **${task.id}**. ${task.title}`);
    for (const detail of task.details) {
      lines.push(`${prefix}  - ${detail}`);
    }
    if (task.subtasks) {
      for (const subtask of task.subtasks) {
        lines.push(...this.formatTaskMarkdown(subtask, indent + 1));
      }
    }
    return lines;
  }
  formatAsJSON(taskList) {
    return JSON.stringify(taskList, null, 2);
  }
  formatAsTable(taskList) {
    const lines = [];
    lines.push(`# ${taskList.title}`);
    lines.push("");
    lines.push("| ID | \uC0C1\uD0DC | \uC81C\uBAA9 | \uC138\uBD80\uC0AC\uD56D |");
    lines.push("|------|--------|------|----------|");
    const formatTaskRow = (task, indent) => {
      const status = task.status === "completed" ? "\u2705 \uC644\uB8CC" : task.status === "in_progress" ? "\u{1F504} \uC9C4\uD589\uC911" : "\u23F3 \uB300\uAE30";
      const details = task.details.join(", ").substring(0, 30);
      const title = indent + task.title;
      lines.push(`| ${task.id} | ${status} | ${title} | ${details}${details.length > 30 ? "..." : ""} |`);
      if (task.subtasks) {
        for (const subtask of task.subtasks) {
          formatTaskRow(subtask, indent + "  ");
        }
      }
    };
    for (const task of taskList.tasks) {
      formatTaskRow(task, "");
    }
    return lines.join("\n");
  }
  formatStatusSummary(summary) {
    const lines = [];
    lines.push(`\u{1F4CB} Task Status Summary`);
    lines.push("");
    lines.push(`\uC5D0\uC774\uC804\uD2B8: ${summary.agent}`);
    lines.push(`\uC791\uC5C5: ${summary.title}`);
    lines.push(`\uC0C1\uD0DC: ${this.getStatusEmoji(summary.status)} ${summary.status}`);
    lines.push(`\uC644\uB8CC\uC728: ${summary.completionRate}% (${summary.completedCount}/${summary.totalCount})`);
    if (summary.currentPhase) {
      lines.push(`\uD604\uC7AC \uB2E8\uACC4: ${summary.currentPhase}`);
    }
    const barLength = 20;
    const filledLength = Math.round(summary.completionRate / 100 * barLength);
    const bar = "\u2588".repeat(filledLength) + "\u2591".repeat(barLength - filledLength);
    lines.push(`\uC9C4\uD589\uB3C4: [${bar}] ${summary.completionRate}%`);
    return lines.join("\n");
  }
  getStatusEmoji(status) {
    switch (status) {
      case "completed":
        return "\u2705";
      case "in_progress":
        return "\u{1F504}";
      default:
        return "\u23F3";
    }
  }
  calculateStats(tasks) {
    let total = 0;
    let completed = 0;
    let inProgress = 0;
    const countTasks2 = (taskList) => {
      for (const task of taskList) {
        total++;
        if (task.status === "completed") {
          completed++;
        } else if (task.status === "in_progress") {
          inProgress++;
        }
        if (task.subtasks) {
          countTasks2(task.subtasks);
        }
      }
    };
    countTasks2(tasks);
    const completionRate = total > 0 ? Math.round(completed / total * 100) : 0;
    let status = "pending";
    if (completed === total && total > 0) {
      status = "completed";
    } else if (inProgress > 0 || completed > 0) {
      status = "in_progress";
    }
    return { status, completionRate, completedCount: completed, totalCount: total };
  }
  // ===========================================
  // New Methods for Batch Operations & UI Enhancement
  // ===========================================
  /**
   * 마크다운 형식의 전체 현황 출력
   */
  formatTaskListWithStatus(taskList, summary) {
    const lines = [];
    lines.push(`# \u{1F4CB} ${taskList.title}`);
    lines.push("");
    lines.push(`**\uC5D0\uC774\uC804\uD2B8**: ${taskList.agent}`);
    lines.push("");
    lines.push("## \u{1F4CA} \uC9C4\uD589 \uC0C1\uD669");
    lines.push("");
    lines.push(`| \uC0C1\uD0DC | \uAC1C\uC218 | \uBE44\uC728 |`);
    lines.push(`|------|------|------|`);
    lines.push(`| \u2705 \uC644\uB8CC | ${summary.completed} | ${Math.round(summary.completed / summary.total * 100) || 0}% |`);
    lines.push(`| \u{1F504} \uC9C4\uD589\uC911 | ${summary.inProgress} | ${Math.round(summary.inProgress / summary.total * 100) || 0}% |`);
    lines.push(`| \u23F3 \uB300\uAE30 | ${summary.pending} | ${Math.round(summary.pending / summary.total * 100) || 0}% |`);
    lines.push(`| **\uD569\uACC4** | **${summary.total}** | **${summary.completionRate}%** |`);
    lines.push("");
    lines.push("### \uC9C4\uD589\uB960");
    lines.push("");
    lines.push(this.formatProgressBar(summary.completionRate));
    lines.push("");
    lines.push("---");
    lines.push("");
    lines.push("## \u{1F4CB} \uC791\uC5C5 \uBAA9\uB85D");
    lines.push("");
    for (const task of taskList.tasks) {
      lines.push(...this.formatTaskWithStatus(task, 0));
    }
    return lines.join("\n");
  }
  formatTaskWithStatus(task, indent) {
    const lines = [];
    const prefix = "  ".repeat(indent);
    const checkbox = task.status === "completed" ? "[x]" : "[ ]";
    const statusEmoji = task.status === "completed" ? "\u2705" : task.status === "in_progress" ? "\u{1F504}" : "\u23F3";
    lines.push(`${prefix}- ${checkbox} ${statusEmoji} **${task.id}**. ${this.escapeMarkdown(task.title)}`);
    for (const detail of task.details) {
      lines.push(`${prefix}  - ${this.escapeMarkdown(detail)}`);
    }
    if (task.subtasks && task.subtasks.length > 0) {
      for (const subtask of task.subtasks) {
        lines.push(...this.formatTaskWithStatus(subtask, indent + 1));
      }
    }
    return lines;
  }
  /**
   * 배치 작업 결과 포맷팅
   */
  formatBatchResult(results, summary, taskList, statusSummary) {
    const lines = [];
    lines.push("# \u{1F4E6} \uBC30\uCE58 \uC791\uC5C5 \uACB0\uACFC");
    lines.push("");
    lines.push("## \u{1F4CA} \uC694\uC57D");
    lines.push("");
    lines.push(`- **\uCD1D \uC791\uC5C5**: ${summary.total}`);
    lines.push(`- **\u2705 \uC131\uACF5**: ${summary.succeeded}`);
    lines.push(`- **\u274C \uC2E4\uD328**: ${summary.failed}`);
    lines.push("");
    lines.push(this.formatProgressBar(Math.round(summary.succeeded / summary.total * 100)));
    lines.push("");
    if (summary.failed > 0) {
      lines.push("## \u274C \uC2E4\uD328\uD55C \uC791\uC5C5");
      lines.push("");
      for (const result of results.filter((r) => !r.success)) {
        lines.push(`### ${this.formatOperationType(result.operation.type)}: ${result.operation.title || result.operation.id}`);
        lines.push(`- **\uC624\uB958**: ${result.message}`);
        if (result.error) {
          lines.push(`- **\uC0C1\uC138**: ${result.error}`);
        }
        lines.push("");
      }
    }
    lines.push("## \u2705 \uC791\uC5C5 \uC0C1\uC138");
    lines.push("");
    for (const result of results) {
      const emoji = result.success ? "\u2705" : "\u274C";
      lines.push(`- ${emoji} **${result.operation.type.toUpperCase()}**: ${result.message}`);
    }
    lines.push("");
    lines.push("---");
    lines.push("");
    lines.push("## \u{1F4CB} \uD604\uC7AC \uC791\uC5C5 \uD604\uD669");
    lines.push("");
    lines.push(this.formatTaskListWithStatus(taskList, statusSummary));
    return lines.join("\n");
  }
  formatOperationType(type) {
    const typeMap = {
      "add": "\u2795 \uCD94\uAC00",
      "update": "\u{1F504} \uC5C5\uB370\uC774\uD2B8",
      "complete": "\u2705 \uC644\uB8CC",
      "remove": "\u{1F5D1}\uFE0F \uC0AD\uC81C"
    };
    return typeMap[type] || type;
  }
  /**
   * add 작업 결과 포맷팅
   */
  formatAddResult(task, taskList, summary) {
    const lines = [];
    lines.push(`\u2705 \uC791\uC5C5 \uCD94\uAC00 \uC644\uB8CC`);
    lines.push("");
    lines.push(`**\uC81C\uBAA9**: ${task.title}`);
    lines.push(`**ID**: ${task.id}`);
    lines.push(`**\uC0C1\uD0DC**: \u23F3 \uB300\uAE30`);
    lines.push("");
    lines.push("---");
    lines.push("");
    lines.push(this.formatTaskListWithStatus(taskList, summary));
    return lines.join("\n");
  }
  /**
   * update 작업 결과 포맷팅
   */
  formatUpdateResult(taskId, newStatus, taskList, summary) {
    const lines = [];
    const statusEmoji = newStatus === "completed" ? "\u2705" : newStatus === "in_progress" ? "\u{1F504}" : "\u23F3";
    lines.push(`\u2705 \uC791\uC5C5 \uC0C1\uD0DC \uC5C5\uB370\uC774\uD2B8 \uC644\uB8CC`);
    lines.push("");
    lines.push(`**ID**: ${taskId}`);
    lines.push(`**\uC0C8 \uC0C1\uD0DC**: ${statusEmoji} ${newStatus}`);
    lines.push("");
    lines.push("---");
    lines.push("");
    lines.push(this.formatTaskListWithStatus(taskList, summary));
    return lines.join("\n");
  }
  /**
   * 텍스트 기반 진행률 바 생성
   * 예: [████░░░░░░] 40%
   */
  formatProgressBar(percentage, barLength = 20) {
    const clampedPercentage = Math.max(0, Math.min(100, percentage));
    const filledLength = Math.round(clampedPercentage / 100 * barLength);
    const emptyLength = barLength - filledLength;
    const bar = "\u2588".repeat(filledLength) + "\u2591".repeat(emptyLength);
    return `[${bar}] ${clampedPercentage}%`;
  }
  /**
   * TaskList에서 상태 통계 계산
   */
  calculateStatusSummary(taskList) {
    let total = 0;
    let completed = 0;
    let inProgress = 0;
    let pending = 0;
    const countTasks2 = (tasks) => {
      for (const task of tasks) {
        total++;
        if (task.status === "completed") {
          completed++;
        } else if (task.status === "in_progress") {
          inProgress++;
        } else {
          pending++;
        }
        if (task.subtasks) {
          countTasks2(task.subtasks);
        }
      }
    };
    countTasks2(taskList.tasks);
    const completionRate = total > 0 ? Math.round(completed / total * 100) : 0;
    return {
      agent: taskList.agent,
      title: taskList.title,
      total,
      completed,
      inProgress,
      pending,
      completionRate
    };
  }
  /**
   * 마크다운 특수 문자 이스케이프
   */
  escapeMarkdown(text) {
    if (!text) return "";
    return text.replace(/\\/g, "\\\\").replace(/\*/g, "\\*").replace(/_/g, "\\_").replace(/\[/g, "\\[").replace(/\]/g, "\\]").replace(/\(/g, "\\(").replace(/\)/g, "\\)").replace(/`/g, "\\`").replace(/#/g, "\\#");
  }
};

// src/commands/init.ts
async function initCommand(args) {
  const storage = new Storage();
  const parser = new Parser();
  const formatter = new Formatter();
  try {
    const taskList = createTaskList(args);
    const markdownContent = parser.generateTaskList(taskList);
    const fileName = generateFileName(args.agent, args.title);
    await storage.saveTaskList(args.sessionId, fileName, markdownContent);
    const taskIds = extractTaskIds(taskList.tasks);
    const totalTasks = countTasks(taskList.tasks);
    const statusSummary = formatter.calculateStatusSummary(taskList);
    const formattedOutput = formatInitResult(args, taskList, statusSummary, fileName, taskIds, totalTasks, formatter);
    const response = {
      title: `Initialized: ${args.title} (${totalTasks} tasks)`,
      output: formattedOutput,
      metadata: {
        taskList,
        tasks: taskList.tasks,
        summary: statusSummary,
        operation: "init",
        message: `Task list "${args.title}" initialized successfully for agent "${args.agent}"`
      }
    };
    return {
      success: true,
      title: args.title,
      agent: args.agent,
      fileName,
      taskIds,
      totalTasks,
      message: `Task list "${args.title}" initialized successfully for agent "${args.agent}"`,
      currentStatus: taskList,
      statusSummary,
      formattedOutput,
      response
    };
  } catch (error) {
    console.error("\u274C Failed to initialize task list:", error);
    throw error;
  }
}
function formatInitResult(args, taskList, summary, fileName, taskIds, totalTasks, formatter) {
  const lines = [];
  lines.push(`# \u2705 Task List Initialized`);
  lines.push("");
  lines.push(`**\uC791\uC5C5 \uBAA9\uB85D**: ${args.title}`);
  lines.push(`**\uC5D0\uC774\uC804\uD2B8**: ${args.agent}`);
  lines.push(`**\uD30C\uC77C\uBA85**: ${fileName}.md`);
  lines.push(`**\uCD1D \uC791\uC5C5**: ${totalTasks}`);
  if (taskIds.length > 0) {
    lines.push(`**\uC791\uC5C5 ID**: ${taskIds.join(", ")}`);
  }
  lines.push("");
  lines.push("---");
  lines.push("");
  lines.push(formatter.formatTaskListWithStatus(taskList, summary));
  return lines.join("\n");
}
function extractTaskIds(tasks) {
  const ids = [];
  for (const task of tasks) {
    ids.push(task.id);
    if (task.subtasks && task.subtasks.length > 0) {
      ids.push(...extractTaskIds(task.subtasks));
    }
  }
  return ids;
}
function generateFileName(agent, title) {
  return `${agent}-${title}`.toLowerCase().replace(/[^a-z0-9가-힣\s-]/g, "").replace(/\s+/g, "-").substring(0, 50);
}
function convertTasks(inputs) {
  if (!inputs || inputs.length === 0) {
    return [];
  }
  return inputs.map((input) => ({
    id: input.id,
    title: input.title,
    status: input.status || "pending",
    details: input.details || [],
    subtasks: input.subtasks ? convertTasks(input.subtasks) : [],
    createdAt: (/* @__PURE__ */ new Date()).toISOString(),
    updatedAt: (/* @__PURE__ */ new Date()).toISOString()
  }));
}
function createTaskList(args) {
  const tasks = convertTasks(args.tasks);
  return {
    title: args.title,
    agent: args.agent,
    createdAt: (/* @__PURE__ */ new Date()).toISOString(),
    sessionId: args.sessionId,
    tasks,
    currentPhase: tasks.length > 0 ? tasks[0].title : void 0
  };
}
function countTasks(tasks) {
  let count = 0;
  for (const task of tasks) {
    count++;
    if (task.subtasks && task.subtasks.length > 0) {
      count += countTasks(task.subtasks);
    }
  }
  return count;
}

// src/commands/add-task.ts
async function addTaskCommand(args) {
  const storage = new Storage();
  const parser = new Parser();
  const formatter = new Formatter();
  try {
    const files = await storage.listTaskFiles(args.sessionId);
    if (files.length === 0) {
      const emptyStatus = {
        agent: "unknown",
        title: "Empty",
        total: 0,
        completed: 0,
        inProgress: 0,
        pending: 0,
        completionRate: 0
      };
      const emptyTaskList = {
        title: "Empty",
        agent: "unknown",
        createdAt: (/* @__PURE__ */ new Date()).toISOString(),
        sessionId: args.sessionId,
        tasks: []
      };
      const errorResponse = {
        title: `Failed to add: ${args.title}`,
        output: `\u274C No task lists found for session: ${args.sessionId}. Create a task list first with: tasks init`,
        metadata: {
          taskList: emptyTaskList,
          tasks: [],
          summary: emptyStatus,
          operation: "add",
          message: `No task lists found for session: ${args.sessionId}`
        }
      };
      return {
        success: false,
        title: args.title,
        parent: args.parent,
        details: [],
        message: `No task lists found for session: ${args.sessionId}. Create a task list first with: tasks init`,
        currentStatus: emptyTaskList,
        statusSummary: emptyStatus,
        formattedOutput: `\u274C No task lists found for session: ${args.sessionId}. Create a task list first with: tasks init`,
        response: errorResponse
      };
    }
    const file = files[0];
    const listTitle = file.replace(".md", "");
    const content = await storage.readTaskList(args.sessionId, listTitle);
    if (!content) {
      throw new Error("Failed to read task list");
    }
    const taskList = parser.parseTaskList(content);
    const details = args.details ? args.details.split(",").map((d) => d.trim()).filter((d) => d) : [];
    const parent = args.parent && args.parent.trim() !== "" && args.parent !== "root" ? args.parent : void 0;
    const added = parser.addTask(taskList, parent, args.title, details);
    if (added) {
      const updatedContent = parser.generateTaskList(taskList);
      await storage.saveTaskList(args.sessionId, listTitle, updatedContent);
      const statusSummary = formatter.calculateStatusSummary(taskList);
      const newTask = findNewlyAddedTask(taskList, args.title);
      const formattedOutput = formatter.formatAddResult(newTask, taskList, statusSummary);
      const response = {
        title: `Added: ${args.title} (ID: ${newTask.id})`,
        output: formattedOutput,
        metadata: {
          taskList,
          tasks: taskList.tasks,
          summary: statusSummary,
          operation: "add",
          taskId: newTask.id,
          parent,
          message: details.length > 0 ? `Task added: ${args.title} with ${details.length} detail(s)` : `Task added: ${args.title}`
        }
      };
      return {
        success: true,
        title: args.title,
        parent,
        details,
        taskId: newTask.id,
        message: details.length > 0 ? `Task added: ${args.title} with ${details.length} detail(s)` : `Task added: ${args.title}`,
        currentStatus: taskList,
        statusSummary,
        formattedOutput,
        response
      };
    } else {
      throw new Error(`Failed to add task. Parent task ${args.parent} not found.`);
    }
  } catch (error) {
    throw error;
  }
}
function findNewlyAddedTask(taskList, title) {
  const findTask = (tasks) => {
    for (const task of tasks) {
      if (task.title === title) {
        return task;
      }
      if (task.subtasks) {
        const found2 = findTask(task.subtasks);
        if (found2) return found2;
      }
    }
    return null;
  };
  const found = findTask(taskList.tasks);
  if (found) {
    return found;
  }
  return {
    id: "unknown",
    title,
    status: "pending",
    details: [],
    createdAt: (/* @__PURE__ */ new Date()).toISOString(),
    updatedAt: (/* @__PURE__ */ new Date()).toISOString()
  };
}

// src/commands/update.ts
async function updateCommand(args) {
  const storage = new Storage();
  const parser = new Parser();
  const formatter = new Formatter();
  try {
    const files = await storage.listTaskFiles(args.sessionId);
    if (files.length === 0) {
      const emptyStatus = {
        agent: "unknown",
        title: "Empty",
        total: 0,
        completed: 0,
        inProgress: 0,
        pending: 0,
        completionRate: 0
      };
      const emptyTaskList = {
        title: "Empty",
        agent: "unknown",
        createdAt: (/* @__PURE__ */ new Date()).toISOString(),
        sessionId: args.sessionId,
        tasks: []
      };
      const errorResponse = {
        title: `Failed to update: Task ${args.id}`,
        output: `\u274C No task lists found for session: ${args.sessionId}`,
        metadata: {
          taskList: emptyTaskList,
          tasks: [],
          summary: emptyStatus,
          operation: "update",
          taskId: args.id,
          status: args.status,
          message: `No task lists found for session: ${args.sessionId}`
        }
      };
      return {
        success: false,
        taskId: args.id,
        status: args.status,
        message: `No task lists found for session: ${args.sessionId}`,
        currentStatus: emptyTaskList,
        statusSummary: emptyStatus,
        formattedOutput: `\u274C No task lists found for session: ${args.sessionId}`,
        response: errorResponse
      };
    }
    for (const file of files) {
      const title = file.replace(".md", "");
      const content = await storage.readTaskList(args.sessionId, title);
      if (!content) continue;
      const taskList = parser.parseTaskList(content);
      const updated = parser.updateTaskStatus(taskList, args.id, args.status);
      if (updated) {
        const updatedContent = parser.generateTaskList(taskList);
        await storage.saveTaskList(args.sessionId, title, updatedContent);
        const statusSummary2 = formatter.calculateStatusSummary(taskList);
        const formattedOutput = formatter.formatUpdateResult(args.id, args.status, taskList, statusSummary2);
        const response = {
          title: `Updated: Task ${args.id} \u2192 ${args.status}`,
          output: formattedOutput,
          metadata: {
            taskList,
            tasks: taskList.tasks,
            summary: statusSummary2,
            operation: "update",
            taskId: args.id,
            status: args.status,
            message: `Task ${args.id} status updated to: ${args.status}`
          }
        };
        return {
          success: true,
          taskId: args.id,
          status: args.status,
          message: `Task ${args.id} status updated to: ${args.status}`,
          currentStatus: taskList,
          statusSummary: statusSummary2,
          formattedOutput,
          response
        };
      }
    }
    const lastTaskList = files.length > 0 ? await (async () => {
      const file = files[0];
      const title = file.replace(".md", "");
      const content = await storage.readTaskList(args.sessionId, title);
      if (content) {
        return parser.parseTaskList(content);
      }
      return null;
    })() : null;
    const statusSummary = lastTaskList ? formatter.calculateStatusSummary(lastTaskList) : {
      agent: "unknown",
      title: "Empty",
      total: 0,
      completed: 0,
      inProgress: 0,
      pending: 0,
      completionRate: 0
    };
    const notFoundTaskList = lastTaskList || {
      title: "Empty",
      agent: "unknown",
      createdAt: (/* @__PURE__ */ new Date()).toISOString(),
      sessionId: args.sessionId,
      tasks: []
    };
    const notFoundResponse = {
      title: `Failed to update: Task ${args.id} not found`,
      output: `\u274C Task ${args.id} not found`,
      metadata: {
        taskList: notFoundTaskList,
        tasks: notFoundTaskList.tasks,
        summary: statusSummary,
        operation: "update",
        taskId: args.id,
        status: args.status,
        message: `Task ${args.id} not found`
      }
    };
    return {
      success: false,
      taskId: args.id,
      status: args.status,
      message: `Task ${args.id} not found`,
      currentStatus: notFoundTaskList,
      statusSummary,
      formattedOutput: `\u274C Task ${args.id} not found`,
      response: notFoundResponse
    };
  } catch (error) {
    throw error;
  }
}

// src/commands/complete.ts
async function completeCommand(args) {
  const result = await updateCommand({
    sessionId: args.sessionId,
    id: args.id,
    status: "completed"
  });
  const completeResponse = {
    title: result.success ? `Completed: Task ${args.id}` : `Failed to complete: Task ${args.id}`,
    output: result.formattedOutput,
    metadata: {
      taskList: result.currentStatus,
      tasks: result.currentStatus.tasks,
      summary: result.statusSummary,
      operation: "complete",
      taskId: args.id,
      status: "completed",
      message: result.success ? `Task ${args.id} marked as completed` : result.message
    }
  };
  if (result.success) {
    return {
      success: true,
      taskId: args.id,
      message: `Task ${args.id} marked as completed`,
      currentStatus: result.currentStatus,
      statusSummary: result.statusSummary,
      formattedOutput: result.formattedOutput,
      response: completeResponse
    };
  } else {
    return {
      success: false,
      taskId: args.id,
      message: result.message,
      currentStatus: result.currentStatus,
      statusSummary: result.statusSummary,
      formattedOutput: result.formattedOutput,
      response: completeResponse
    };
  }
}

// src/commands/remove.ts
async function removeCommand(args) {
  const storage = new Storage();
  const parser = new Parser();
  const formatter = new Formatter();
  try {
    const files = await storage.listTaskFiles(args.sessionId);
    if (files.length === 0) {
      const emptyStatus = {
        agent: "unknown",
        title: "Empty",
        total: 0,
        completed: 0,
        inProgress: 0,
        pending: 0,
        completionRate: 0
      };
      const emptyTaskList = {
        title: "Empty",
        agent: "unknown",
        createdAt: (/* @__PURE__ */ new Date()).toISOString(),
        sessionId: args.sessionId,
        tasks: []
      };
      const errorResponse = {
        title: `Failed to remove: Task ${args.id}`,
        output: `\u274C No task lists found for session: ${args.sessionId}`,
        metadata: {
          taskList: emptyTaskList,
          tasks: [],
          summary: emptyStatus,
          operation: "remove",
          taskId: args.id,
          message: `No task lists found for session: ${args.sessionId}`
        }
      };
      return {
        success: false,
        taskId: args.id,
        message: `No task lists found for session: ${args.sessionId}`,
        currentStatus: emptyTaskList,
        statusSummary: emptyStatus,
        formattedOutput: `\u274C No task lists found for session: ${args.sessionId}`,
        response: errorResponse
      };
    }
    for (const file of files) {
      const title = file.replace(".md", "");
      const content = await storage.readTaskList(args.sessionId, title);
      if (!content) continue;
      const taskList = parser.parseTaskList(content);
      const findTask = (tasks) => {
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
        const removed = parser.removeTask(taskList, args.id);
        if (removed) {
          const updatedContent = parser.generateTaskList(taskList);
          await storage.saveTaskList(args.sessionId, title, updatedContent);
          const statusSummary2 = formatter.calculateStatusSummary(taskList);
          const formattedOutput = formatRemoveResult(args.id, taskToRemove.title, taskList, statusSummary2, formatter);
          const response = {
            title: `Removed: Task ${args.id}`,
            output: formattedOutput,
            metadata: {
              taskList,
              tasks: taskList.tasks,
              summary: statusSummary2,
              operation: "remove",
              taskId: args.id,
              message: `Task ${args.id} "${taskToRemove.title}" removed`
            }
          };
          return {
            success: true,
            taskId: args.id,
            taskTitle: taskToRemove.title,
            message: `Task ${args.id} "${taskToRemove.title}" removed`,
            currentStatus: taskList,
            statusSummary: statusSummary2,
            formattedOutput,
            response
          };
        }
      }
    }
    const lastTaskList = files.length > 0 ? await (async () => {
      const file = files[0];
      const title = file.replace(".md", "");
      const content = await storage.readTaskList(args.sessionId, title);
      if (content) {
        return parser.parseTaskList(content);
      }
      return null;
    })() : null;
    const statusSummary = lastTaskList ? formatter.calculateStatusSummary(lastTaskList) : {
      agent: "unknown",
      title: "Empty",
      total: 0,
      completed: 0,
      inProgress: 0,
      pending: 0,
      completionRate: 0
    };
    const notFoundTaskList = lastTaskList || {
      title: "Empty",
      agent: "unknown",
      createdAt: (/* @__PURE__ */ new Date()).toISOString(),
      sessionId: args.sessionId,
      tasks: []
    };
    const notFoundResponse = {
      title: `Failed to remove: Task ${args.id} not found`,
      output: `\u274C Task ${args.id} not found`,
      metadata: {
        taskList: notFoundTaskList,
        tasks: notFoundTaskList.tasks,
        summary: statusSummary,
        operation: "remove",
        taskId: args.id,
        message: `Task ${args.id} not found`
      }
    };
    return {
      success: false,
      taskId: args.id,
      message: `Task ${args.id} not found`,
      currentStatus: notFoundTaskList,
      statusSummary,
      formattedOutput: `\u274C Task ${args.id} not found`,
      response: notFoundResponse
    };
  } catch (error) {
    throw error;
  }
}
function formatRemoveResult(taskId, taskTitle, taskList, summary, formatter) {
  const lines = [];
  lines.push(`\u2705 \uC791\uC5C5 \uC0AD\uC81C \uC644\uB8CC`);
  lines.push("");
  lines.push(`**ID**: ${taskId}`);
  lines.push(`**\uC81C\uBAA9**: ${taskTitle}`);
  lines.push("");
  lines.push("---");
  lines.push("");
  lines.push(formatter.formatTaskListWithStatus(taskList, summary));
  return lines.join("\n");
}

// src/commands/unified.ts
async function unifiedCommand(params) {
  const { sessionId, operations } = params;
  if (!sessionId || typeof sessionId !== "string") {
    throw new Error("Invalid sessionId: must be a non-empty string");
  }
  if (!Array.isArray(operations)) {
    throw new Error("Invalid operations: must be an array");
  }
  if (operations.length === 0) {
    throw new Error("No operations provided: operations array cannot be empty");
  }
  if (operations.length > 50) {
    throw new Error(`Too many operations: maximum 50 allowed, got ${operations.length}`);
  }
  for (let i = 0; i < operations.length; i++) {
    const op = operations[i];
    if (!op || typeof op !== "object") {
      throw new Error(`Invalid operation at index ${i}: must be an object`);
    }
    if (!op.type || !["init", "add", "update", "complete", "remove"].includes(op.type)) {
      throw new Error(`Invalid operation type at index ${i}: must be one of init, add, update, complete, remove`);
    }
  }
  const storage = new Storage();
  const formatter = new Formatter();
  const results = [];
  let succeeded = 0;
  let failed = 0;
  for (const operation of operations) {
    const result = await executeOperationWithErrorHandling(sessionId, operation);
    results.push(result);
    if (result.success) {
      succeeded++;
    } else {
      failed++;
    }
  }
  const currentStatus = await getCurrentSessionStatus(sessionId, storage);
  const response = formatUnifiedResponse(
    results,
    { total: operations.length, succeeded, failed },
    currentStatus,
    formatter
  );
  return {
    success: failed === 0,
    results,
    summary: {
      total: operations.length,
      succeeded,
      failed
    },
    currentStatus,
    response
  };
}
async function executeOperationWithErrorHandling(sessionId, operation) {
  try {
    return await executeOperation(sessionId, operation);
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : String(error);
    console.error(`\u274C Operation ${operation.type} failed:`, errorMessage);
    return {
      success: false,
      operation,
      message: `Failed to execute ${operation.type}: ${errorMessage}`,
      error: errorMessage
    };
  }
}
async function executeOperation(sessionId, operation) {
  switch (operation.type) {
    case "init":
      if (!operation.agent || !operation.title) {
        return {
          success: false,
          operation,
          message: "Missing required fields: agent and title"
        };
      }
      const initResult = await initCommand({
        sessionId,
        agent: operation.agent,
        title: operation.title
      });
      return {
        success: initResult.success,
        operation,
        message: initResult.message,
        taskId: initResult.taskIds?.[0]
      };
    case "add":
      if (!operation.title) {
        return {
          success: false,
          operation,
          message: "Missing required field: title"
        };
      }
      const addResult = await addTaskCommand({
        sessionId,
        title: operation.title,
        parent: operation.parent
      });
      return {
        success: addResult.success,
        operation,
        message: addResult.message,
        taskId: addResult.taskId,
        taskTitle: addResult.title
      };
    case "update":
      if (!operation.id || !operation.status) {
        return {
          success: false,
          operation,
          message: "Missing required fields: id and status"
        };
      }
      const updateResult = await updateCommand({
        sessionId,
        id: operation.id,
        status: operation.status
      });
      return {
        success: updateResult.success,
        operation,
        message: updateResult.message,
        taskId: operation.id
      };
    case "complete":
      if (!operation.id) {
        return {
          success: false,
          operation,
          message: "Missing required field: id"
        };
      }
      const completeResult = await completeCommand({
        sessionId,
        id: operation.id
      });
      return {
        success: completeResult.success,
        operation,
        message: completeResult.message,
        taskId: operation.id
      };
    case "remove":
      if (!operation.id) {
        return {
          success: false,
          operation,
          message: "Missing required field: id"
        };
      }
      const removeResult = await removeCommand({
        sessionId,
        id: operation.id,
        force: true
      });
      return {
        success: removeResult.success,
        operation,
        message: removeResult.message,
        taskId: operation.id
      };
    default:
      const _exhaustiveCheck = operation;
      return {
        success: false,
        operation,
        message: `Unknown operation type: ${operation.type}`
      };
  }
}
async function getCurrentSessionStatus(sessionId, storage) {
  try {
    const files = await storage.listTaskFiles(sessionId);
    if (files.length === 0) {
      return null;
    }
    const parser = new Parser();
    const content = await storage.readTaskList(sessionId, files[0].replace(".md", ""));
    if (!content) {
      return null;
    }
    return parser.parseTaskList(content);
  } catch (error) {
    console.error("Failed to get current session status:", error);
    return null;
  }
}
function formatUnifiedResponse(results, summary, currentStatus, formatter) {
  const lines = [];
  if (currentStatus) {
    const statusSummary = formatter.calculateStatusSummary(currentStatus);
    const total = statusSummary.total;
    const completed = statusSummary.completed;
    const inProgress = statusSummary.inProgress;
    const pending = statusSummary.pending;
    lines.push(`\u{1F4CB} total ${total}, in-progress ${inProgress}, done ${completed}`);
    lines.push("");
  }
  const failedResults = [];
  const succeededResults = [];
  for (const result of results) {
    if (result.success) {
      succeededResults.push(result);
    } else {
      failedResults.push(result);
    }
  }
  if (failedResults.length > 0) {
    lines.push("## \u274C \uC2E4\uD328\uD55C \uC791\uC5C5");
    lines.push("");
    for (const result of failedResults) {
      lines.push(`- \u274C **${result.operation.type}**: ${result.message}`);
      if (result.error) {
        lines.push(`  - \uC0C1\uC138: ${result.error}`);
      }
    }
    lines.push("");
  }
  if (currentStatus) {
    lines.push("");
    for (const task of currentStatus.tasks) {
      lines.push(...formatTaskCheckbox(task, 0));
    }
    lines.push("");
  } else {
    lines.push("\u2139\uFE0F \uD604\uC7AC \uC138\uC158\uC5D0 \uC791\uC5C5 \uBAA9\uB85D\uC774 \uC5C6\uC2B5\uB2C8\uB2E4.");
  }
  const output = lines.join("\n");
  return {
    title: `\uC791\uC5C5 \uC644\uB8CC: ${summary.succeeded}/${summary.total} \uC131\uACF5`,
    output,
    metadata: {
      results,
      // Operation execution summary (not task list status)
      // completed: operations that succeeded
      // failed: operations that failed
      // total: total operations attempted
      summary: currentStatus ? {
        agent: currentStatus.agent,
        title: currentStatus.title,
        total: summary.total,
        completed: summary.succeeded,
        inProgress: 0,
        pending: summary.failed,
        completionRate: summary.total > 0 ? Math.round(summary.succeeded / summary.total * 100) : 0
      } : void 0,
      taskList: currentStatus || void 0,
      operation: "unified"
    }
  };
}
function formatTaskCheckbox(task, indent) {
  const lines = [];
  const prefix = "  ".repeat(indent);
  const checkbox = task.status === "completed" ? "[x]" : "[ ]";
  const statusEmoji = task.status === "completed" ? "\u2705" : task.status === "in_progress" ? "\u{1F504}" : "\u23F3";
  lines.push(`${prefix}- ${checkbox} ${statusEmoji} **${task.id}**. ${task.title}`);
  if (task.subtasks && task.subtasks.length > 0) {
    for (const subtask of task.subtasks) {
      lines.push(...formatTaskCheckbox(subtask, indent + 1));
    }
  }
  return lines;
}

// src/index.ts
var TasksPlugin = async ({ client }) => {
  return {
    // Define custom tools that agents can use
    tool: {
      // Tool: tasks - Unified task management tool
      // Replaces all previous individual tools (tasks_init, tasks_add, tasks_update, tasks_complete, tasks_remove, tasks_list, tasks_status)
      tasks: tool({
        description: "Unified task management tool. Supports init, add, update, complete, remove operations. Execute multiple operations in a single call (max 50). Always shows current session's task status after execution. Other sessions' tasks are completely isolated and not visible.",
        args: {
          operations: tool.schema.array(
            tool.schema.object({
              type: tool.schema.enum(
                ["init", "add", "update", "complete", "remove"],
                { description: "Operation type" }
              ),
              // For 'init' operation
              agent: tool.schema.optional(
                tool.schema.string({ description: "Agent name (required for init)" })
              ),
              // For 'init' and 'add' operations
              title: tool.schema.optional(
                tool.schema.string({ description: "Task or list title (required for init/add)" })
              ),
              // For 'update', 'complete', 'remove' operations
              id: tool.schema.optional(
                tool.schema.string({ description: "Task ID (required for update/complete/remove)" })
              ),
              // For 'add' operation (optional)
              parent: tool.schema.optional(
                tool.schema.string({ description: "Parent task ID for nested tasks (optional)" })
              ),
              // For 'update' operation
              status: tool.schema.optional(
                tool.schema.enum(
                  ["pending", "in_progress", "completed"],
                  { description: "New status (required for update)" }
                )
              )
            }),
            { description: "Array of operations to execute (max 50). Each operation is executed in order. Partial failures are allowed - successful operations are kept even if some fail." }
          )
        },
        async execute(args, ctx) {
          try {
            if (!ctx.sessionID) {
              throw new Error("Session ID is required from ToolContext but was not provided. Cannot initialize task list without a valid session ID.");
            }
            const sessionId = ctx.sessionID;
            const result = await unifiedCommand({
              sessionId,
              operations: args.operations
            });
            return result.response.output;
          } catch (error) {
            const errorMessage = error instanceof Error ? error.message : String(error);
            return `\u274C \uC791\uC5C5 \uC2E4\uD589 \uC2E4\uD328: ${errorMessage}`;
          }
        }
      })
    }
  };
};
var index_default = TasksPlugin;
export {
  TasksPlugin,
  index_default as default
};
