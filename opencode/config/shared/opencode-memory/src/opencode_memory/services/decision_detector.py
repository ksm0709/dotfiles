"""
Decision Detector Service

Task 2.1 & 2.2: 의사결정 감지
- question 도구 결과에서 Decision 생성
- 에이전트 텍스트에서 의사결정 패턴 감지
- 기술 선택 패턴 인식 (라이브러리, 아키텍처, 구현 방식)
"""

import re
from typing import List, Optional

from opencode_memory.models.semantic import Decision


class DecisionDetector:
    """규칙 기반 의사결정 감지기

    Phase 5에서 LLM 기반 감지로 확장 예정.
    현재는 정규식 패턴 기반으로 동작.
    """

    # 의사결정 패턴 (한국어)
    DECISION_PATTERNS = [
        # 선택/사용/채택 패턴 - 더 정확한 매칭
        (
            r"([A-Za-z][A-Za-z0-9\.\-_]+)(?:를|을)\s*(?:선택|사용|채택)(?:할|했|하기로)",
            "library",
        ),
        (
            r"([A-Za-z][A-Za-z0-9\.\-_]+)\s+대신\s+([A-Za-z][A-Za-z0-9\.\-_]+)(?:를|을)?\s*(?:사용|선택)",
            "library",
        ),
        (
            r"([A-Za-z][A-Za-z0-9\.\-_]+)(?:보다|대신에?)\s+([A-Za-z][A-Za-z0-9\.\-_]+)(?:가|이)?\s*(?:더\s+)?(?:적합|좋|낫)",
            "library",
        ),
        (
            r"([A-Za-z][A-Za-z0-9\s\.\-_]+)\s*패턴(?:을|를)?\s*(?:채택|적용|사용)",
            "architecture",
        ),
        (
            r"([A-Za-z][A-Za-z0-9\.\-_]+)(?:을|를)\s*(?:추천|권장)(?:드립니다|합니다)",
            "recommendation",
        ),
        # 결정 패턴
        (r"(?:결정|결론).*?([A-Za-z][A-Za-z0-9\.\-_]+)", "approach"),
        (r"([A-Za-z][A-Za-z0-9\s\.\-_]+)\s*방식(?:을|를)?\s*(?:채택|선택)", "approach"),
        # 영어 패턴
        (r"(?:using|chose|selected|picked)\s+([A-Za-z][A-Za-z0-9\.\-_]+)", "library"),
        (r"(?:instead of|over)\s+([A-Za-z][A-Za-z0-9\.\-_]+)", "library"),
    ]

    # 기술 키워드 (라이브러리/프레임워크)
    TECH_KEYWORDS = {
        # Python 라이브러리
        "pytest",
        "flask",
        "django",
        "fastapi",
        "pyjwt",
        "jwt",
        "requests",
        "sqlalchemy",
        "asyncio",
        "aiohttp",
        "celery",
        "redis",
        "poetry",
        "pip",
        # JavaScript/TypeScript
        "react",
        "vue",
        "angular",
        "nextjs",
        "express",
        "nestjs",
        "typescript",
        "javascript",
        "npm",
        "yarn",
        "webpack",
        "vite",
        "eslint",
        "prettier",
        # 데이터베이스
        "postgresql",
        "postgres",
        "mysql",
        "sqlite",
        "mongodb",
        "redis",
        "dynamodb",
        "elasticsearch",
        # 클라우드/인프라
        "aws",
        "gcp",
        "azure",
        "docker",
        "kubernetes",
        "k8s",
        "terraform",
        # 아키텍처 패턴
        "mvc",
        "mvvm",
        "clean architecture",
        "hexagonal",
        "microservices",
        "monolith",
        "serverless",
        "event-driven",
    }

    def __init__(self):
        """초기화"""
        self._compiled_patterns = [
            (re.compile(pattern, re.IGNORECASE), decision_type)
            for pattern, decision_type in self.DECISION_PATTERNS
        ]

    def detect(self, text: str) -> Optional[Decision]:
        """텍스트에서 의사결정 패턴 감지

        Args:
            text: 분석할 텍스트

        Returns:
            Decision 객체 또는 None
        """
        if not text or len(text.strip()) < 10:
            return None

        text_lower = text.lower()

        # 기술 키워드 검색
        found_techs = []
        for tech in self.TECH_KEYWORDS:
            if tech.lower() in text_lower:
                found_techs.append(tech)

        if not found_techs:
            # 기술 키워드가 없으면 일반 텍스트로 간주
            return None

        # 패턴 매칭
        for pattern, decision_type in self._compiled_patterns:
            match = pattern.search(text)
            if match:
                groups = match.groups()
                choice = self._extract_choice(groups, text, found_techs)
                if choice:
                    alternatives = self._extract_alternatives(text, choice, found_techs)
                    rationale = self._extract_rationale(text, match)

                    return Decision(
                        decision_type=decision_type,
                        choice=choice,
                        alternatives=alternatives,
                        rationale=rationale or "패턴 기반 감지",
                        is_user_preference=False,
                    )

        # 패턴 매칭 실패 시 기술 키워드만으로 추론
        if len(found_techs) >= 1:
            # 첫 번째 기술 키워드를 선택으로 간주
            main_tech = found_techs[0]
            # 원래 케이스로 복원
            for tech in self.TECH_KEYWORDS:
                if tech.lower() == main_tech.lower():
                    main_tech = tech
                    break

            return Decision(
                decision_type="library",
                choice=main_tech,
                alternatives=[t for t in found_techs[1:] if t != main_tech],
                rationale="기술 키워드 감지",
                is_user_preference=False,
            )

        return None

    def _extract_choice(
        self,
        groups: tuple,
        text: str,
        found_techs: List[str],
    ) -> Optional[str]:
        """매칭된 그룹에서 선택 항목 추출"""
        for group in groups:
            if group:
                group_clean = group.strip()
                group_lower = group_clean.lower()

                # 기술 키워드와 정확히 매칭
                for tech in self.TECH_KEYWORDS:
                    if tech.lower() == group_lower:
                        return tech
                    # 부분 매칭 (예: "PostgreSQL" in "postgresql을")
                    if tech.lower() in group_lower:
                        return tech

                # 원본 그룹이 기술 키워드 형태면 반환
                if len(group_clean) > 1 and group_clean[0].isupper():
                    return group_clean

        # 폴백: found_techs에서 첫 번째 반환 (원본 케이스 유지)
        if found_techs:
            # 원본 케이스로 복원
            first_tech = found_techs[0]
            for tech in self.TECH_KEYWORDS:
                if tech.lower() == first_tech.lower():
                    return tech
            return first_tech

        return None

    def _extract_alternatives(
        self,
        text: str,
        choice: str,
        found_techs: List[str],
    ) -> List[str]:
        """대안 목록 추출"""
        alternatives = []

        # found_techs에서 choice 제외
        for tech in found_techs:
            if tech.lower() != choice.lower() and tech not in alternatives:
                alternatives.append(tech)

        # "대신", "보다" 패턴에서 추가 추출
        instead_pattern = r"(\S+)\s*(?:대신|보다|대신에)"
        matches = re.findall(instead_pattern, text)
        for match in matches:
            match_lower = match.lower()
            for tech in self.TECH_KEYWORDS:
                if tech.lower() in match_lower:
                    if tech not in alternatives and tech.lower() != choice.lower():
                        alternatives.append(tech)

        return alternatives

    def _extract_rationale(self, text: str, match: re.Match) -> Optional[str]:
        """선택 이유 추출"""
        # 매치 이후 텍스트에서 이유 추출 시도
        after_match = text[match.end() :].strip()
        if after_match:
            # 첫 문장 추출
            sentences = re.split(r"[.!?。]", after_match)
            if sentences and sentences[0].strip():
                return sentences[0].strip()[:100]

        return None

    def extract_from_question(
        self,
        question_output: str,
        alternatives: Optional[List[str]] = None,
    ) -> Optional[Decision]:
        """question 도구 결과에서 사용자 선택 추출

        Args:
            question_output: question 도구의 출력
            alternatives: 제시된 선택지 목록

        Returns:
            Decision 객체 (is_user_preference=True)
        """
        if not question_output:
            return None

        # 선택 패턴 매칭
        choice_patterns = [
            r"선택:\s*(.+)",
            r"selected:\s*(.+)",
            r"User selected:\s*(.+)",
            r"chose:\s*(.+)",
            r"choice:\s*(.+)",
            r"응답:\s*(.+)",
        ]

        choice = None
        for pattern in choice_patterns:
            match = re.search(pattern, question_output, re.IGNORECASE)
            if match:
                choice = match.group(1).strip()
                break

        if not choice:
            # 마지막 줄을 선택으로 간주
            lines = [
                line.strip() for line in question_output.split("\n") if line.strip()
            ]
            if lines:
                choice = lines[-1]

        if choice:
            # alternatives에서 choice 제외
            remaining_alternatives = []
            if alternatives:
                for alt in alternatives:
                    if alt.lower() != choice.lower():
                        remaining_alternatives.append(alt)

            return Decision(
                decision_type="user_choice",
                choice=choice,
                alternatives=remaining_alternatives,
                rationale="사용자 직접 선택",
                is_user_preference=True,
            )

        return None
