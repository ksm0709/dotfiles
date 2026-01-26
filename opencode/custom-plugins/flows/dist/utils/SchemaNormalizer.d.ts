import { FlowDefinition } from "../types/schemas";
/**
 * 스키마 정규화 유틸리티
 * 단축 문법(Concise)을 정규 문법(Canonical)으로 변환합니다.
 */
export declare class SchemaNormalizer {
    /**
     * 플로우 정의를 정규화합니다.
     * 입력이 이미 정규 형식이면 유효성 검사 후 반환하고,
     * 단축 형식이면 변환 후 반환합니다.
     */
    static normalize(input: any): FlowDefinition;
    /**
     * 단일 노드를 정규화합니다.
     */
    private static normalizeNode;
}
//# sourceMappingURL=SchemaNormalizer.d.ts.map