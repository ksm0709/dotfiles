/**
 * OpenCode Flows Plugin
 * 
 * 멀티 에이전트 워크플로우 관리 플러그인
 * JSON 기반 FSM을 정의하여 복잡한 멀티스텝 작업을 자동화
 */

import type { Plugin } from "@opencode-ai/plugin";
import { FlowManager } from "./manager/FlowManager";
import type { FlowInstanceStatus } from "./types/schemas";

export const FlowsPlugin: Plugin = async ({ directory, client }) => {
  // 매니저 초기화
  const manager = FlowManager.getInstance();
  manager.initialize(client, directory);

  /**
   * 상태 포맷팅
   */
  const formatStatus = (statuses: FlowInstanceStatus[]): string => {
    if (statuses.length === 0) {
      return "실행 중인 플로우가 없습니다.";
    }

    return statuses
      .map((s) => {
        const elapsed = s.elapsedMs ? `${Math.round(s.elapsedMs / 1000)}s` : "N/A";
        return `[${s.instanceId.substring(0, 8)}] ${s.flowName} - ${s.status} @ ${s.currentNode} (${elapsed})`;
      })
      .join("\n");
  };

  /**
   * 플로우 목록 포맷팅
   */
  const formatFlowList = (flows: Array<{ name: string; description?: string; path: string }>): string => {
    if (flows.length === 0) {
      return "사용 가능한 플로우가 없습니다.\n.opencode/flows/ 디렉토리에 JSON 파일을 추가하세요.";
    }

    return flows
      .map((f) => `• ${f.name}${f.description ? ` - ${f.description}` : ""}`)
      .join("\n");
  };

  return {
    commands: {
      flow: {
        description: "워크플로우 관리 (start|stop|status|list)",
        args: {
          action: { 
            type: "string", 
            description: "액션: start, stop, status, list" 
          },
          name: { 
            type: "string", 
            optional: true, 
            description: "플로우 이름 또는 인스턴스 ID" 
          },
          prompt: { 
            type: "string", 
            optional: true, 
            description: "플로우에 전달할 프롬프트" 
          },
        },
        async execute(args: { action: string; name?: string; prompt?: string }) {
          try {
            switch (args.action) {
              case "start": {
                if (!args.name) {
                  return "플로우 이름을 지정하세요: /flow start <flow-name> [prompt]";
                }
                const id = await manager.start(args.name, args.prompt);
                return `✅ 플로우 시작됨\n• ID: ${id}\n• 이름: ${args.name}`;
              }

              case "stop": {
                if (!args.name) {
                  // 모든 플로우 중단
                  const count = await manager.stopAll();
                  return `${count}개의 플로우가 중단되었습니다.`;
                }
                const stopped = await manager.stop(args.name);
                if (stopped) {
                  return `플로우 중단됨: ${args.name}`;
                }
                return `플로우를 찾을 수 없습니다: ${args.name}`;
              }

              case "status": {
                if (args.name) {
                  const status = manager.getInstanceStatus(args.name);
                  if (status) {
                    return formatStatus([status]);
                  }
                  return `인스턴스를 찾을 수 없습니다: ${args.name}`;
                }
                const statuses = manager.getStatus();
                return formatStatus(statuses);
              }

              case "list": {
                const flows = await manager.listAvailableFlows();
                return formatFlowList(flows);
              }

              default:
                return `알 수 없는 액션: ${args.action}\n사용법: /flow <start|stop|status|list> [name] [prompt]`;
            }
          } catch (error) {
            return `오류: ${error}`;
          }
        },
      },
    },

    // 이벤트 훅
    event: async ({ event }) => {
      // 세션 종료 시 관련 플로우 정리
      if (event.type === "session.deleted") {
        // 해당 세션을 사용하는 플로우가 있다면 중단
        // (현재는 별도 추적 없음 - 향후 구현)
      }
    },
  };
};

export default FlowsPlugin;

// 타입 export
export * from "./types/schemas";
export { FlowManager } from "./manager/FlowManager";
export { FlowInstance } from "./manager/FlowInstance";
export { Blackboard } from "./manager/Blackboard";
export { BaseNode } from "./nodes/BaseNode";
