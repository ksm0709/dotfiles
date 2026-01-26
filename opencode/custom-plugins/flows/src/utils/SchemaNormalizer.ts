import {
  FlowDefinition,
  NodeDefinition,
  ConciseFlowDefinition,
  ConciseNode,
  FlowDefinitionSchema,
} from "../types/schemas";

/**
 * 스키마 정규화 유틸리티
 * 단축 문법(Concise)을 정규 문법(Canonical)으로 변환합니다.
 */
export class SchemaNormalizer {
  /**
   * 플로우 정의를 정규화합니다.
   * 입력이 이미 정규 형식이면 유효성 검사 후 반환하고,
   * 단축 형식이면 변환 후 반환합니다.
   */
  static normalize(input: any): FlowDefinition {
    // 1. 이미 Canonical 형식인지 확인 (initial_state가 있으면 Canonical로 간주)
    if (input.initial_state && !input.start) {
      return FlowDefinitionSchema.parse(input);
    }

    // 2. Concise 형식 변환
    const concise = input as ConciseFlowDefinition;
    
    const canonical: FlowDefinition = {
      name: concise.name,
      version: concise.version,
      description: concise.description,
      config: concise.config,
      initial_state: concise.start, // start -> initial_state
      nodes: {},
    };

    // 노드 변환
    for (const [nodeName, nodeBody] of Object.entries(concise.nodes)) {
      canonical.nodes[nodeName] = this.normalizeNode(nodeBody);
    }

    // 최종 유효성 검사
    return FlowDefinitionSchema.parse(canonical);
  }

  /**
   * 단일 노드를 정규화합니다.
   */
  private static normalizeNode(node: ConciseNode): NodeDefinition {
    // 이미 Canonical 형식이면 그대로 반환 (type이 있고 config가 있는 경우)
    if ('type' in node && 'config' in node) {
      const anyNode = node as any;
      return {
        type: anyNode.type,
        config: anyNode.config,
        routes: anyNode.routes || anyNode.on, // routes가 없으면 on을 사용
      };
    }

    // 단축 문법 변환
    const routes = 'on' in node ? node.on : undefined;
    let type: any;
    let config: any = {};

    // 1. Agent
    if ('agent' in node) {
      type = 'agent';
      config = {
        agent_type: node.agent,
        prompt: node.prompt,
        results: node.results,
      };
    }
    // 2. Command
    else if ('run' in node) {
      type = 'command';
      config = {
        command: node.run,
        workdir: node.workdir,
        expect_exit_code: node.expect_exit_code,
      };
    }
    // 3. Tool
    else if ('tool' in node) {
      type = 'tool';
      config = {
        tool: node.tool,
        args: node.args,
      };
    }
    // 4. Delay
    else if ('wait' in node) {
      type = 'delay';
      config = {
        duration: node.wait,
      };
    }
    // 5. Loop
    else if ('loop' in node) {
      type = 'loop';
      config = {
        body_node: node.loop,
        max_iterations: node.max,
        while_condition: node.while,
      };
    }
    // 6. End
    else if ('end' in node || ('status' in node && !('type' in node))) {
      type = 'end';
      config = {
        status: (node as any).status || 'success',
        message: (node as any).message,
      };
    }
    // 7. Conditional (conditions가 있으면)
    else if ('conditions' in node) {
      type = 'conditional';
      config = {
        conditions: node.conditions,
        default: node.default,
      };
    }
    // Fallback: 알 수 없는 타입
    else {
      throw new Error(`Unknown node type or missing required fields: ${JSON.stringify(node)}`);
    }

    return {
      type,
      config,
      routes,
    };
  }
}
