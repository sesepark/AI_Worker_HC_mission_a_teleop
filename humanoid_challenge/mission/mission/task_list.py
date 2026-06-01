#!/usr/bin/env python3
"""Mission A task list — OCR parts → {class_name: remaining_count}.

`monitor_ocr_node` 가 발행하는 한국어 부품명을 detector 의 class_name 으로 변환하고,
적재 진행에 따라 잔여 수량을 차감/조회하는 순수 자료구조 (ROS 의존 없음 → 단독 테스트 가능).

Reference:
- 5종 부품 class: humanoid_challenge/docs/PERCEPTION_INTERFACE.md "5종 부품 class"
- OCR JSON 구조: 같은 문서 monitor_ocr_node "/monitor_ocr/result JSON 구조"
"""
from __future__ import annotations


# 한국어 부품명 → detector class_name (PERCEPTION_INTERFACE.md 기준)
PART_NAME_TO_CLASS: dict[str, str] = {
    '플랜지너트': 'flange_nut',
    '기어링': 'gear_ring',
    '스페이서링': 'spacer_ring',
    '육각너트': 'hex_nut',
    '돔너트': 'dome_nut',
}

# 역매핑 (로그/디버그용)
CLASS_TO_PART_NAME: dict[str, str] = {
    'flange_nut': '플랜지 너트',
    'gear_ring': '기어 링',
    'spacer_ring': '스페이서 링',
    'hex_nut': '육각 너트',
    'dome_nut': '돔 너트',
}

# Perception task_management 의 canonical 표기(공백, 소문자) → detector class_name.
# 주의: canonical 은 'dom nut' (perception name_utils.py), 우리 class 는 'dome_nut'.
CANONICAL_TO_CLASS: dict[str, str] = {
    'flange nut': 'flange_nut',
    'gear ring': 'gear_ring',
    'spacer ring': 'spacer_ring',
    'hex nut': 'hex_nut',
    'dom nut': 'dome_nut',
    'dome nut': 'dome_nut',
}

VALID_CLASSES = frozenset(PART_NAME_TO_CLASS.values())


def canonical_to_class(name: str) -> str | None:
    """Perception canonical 부품명('flange nut') → class_name. 실패 시 None.

    공백/대소문자/언더스코어 정규화 후 매칭 (예: 'Flange_Nut' → 'flange_nut').
    """
    key = ' '.join(str(name).strip().lower().replace('_', ' ').split())
    return CANONICAL_TO_CLASS.get(key)


def normalize_part_name(name: str) -> str:
    """공백/유사문자 제거 후 매칭 키로 정규화. '플랜지 너트' → '플랜지너트'."""
    return ''.join(str(name).split())


def part_name_to_class(name: str) -> str | None:
    """한국어 부품명 → class_name. 매칭 실패 시 None."""
    return PART_NAME_TO_CLASS.get(normalize_part_name(name))


class TaskList:
    """class_name 별 잔여 적재 수량 관리."""

    def __init__(self) -> None:
        self._remaining: dict[str, int] = {}
        # OCR 에서 변환 실패한 원본 이름 (디버그 표시용)
        self.unmapped: list[str] = []

    # ── 빌드 ──────────────────────────────────────────────────────────────
    def build_from_ocr_parts(self, parts: list[dict]) -> 'TaskList':
        """OCR parts 배열 [{'name': '플랜지 너트', 'count': 1}, ...] → task_list.

        같은 class 가 여러 줄로 오면 count 합산. count 누락/음수는 무시.
        """
        self._remaining.clear()
        self.unmapped.clear()
        for item in parts or []:
            cls = part_name_to_class(item.get('name', ''))
            if cls is None:
                self.unmapped.append(str(item.get('name', '')))
                continue
            try:
                count = int(item.get('count', 0))
            except (TypeError, ValueError):
                count = 0
            if count <= 0:
                continue
            self._remaining[cls] = self._remaining.get(cls, 0) + count
        return self

    def build_from_task_list_payload(self, parts: list[dict]) -> 'TaskList':
        """Perception `/perception/task_list` parts 배열 → task_list.

        parts = [{'name': 'flange nut', 'count': <잔여>}, ...] (canonical 표기).
        count 는 이미 '잔여'(OCR목표 − 트레이관측) 이므로 그대로 사용.
        management_node 가 항상 5종을 발행 → count 0 도 그대로 반영.
        """
        self._remaining.clear()
        self.unmapped.clear()
        for item in parts or []:
            cls = canonical_to_class(item.get('name', ''))
            if cls is None:
                self.unmapped.append(str(item.get('name', '')))
                continue
            try:
                count = int(item.get('count', 0))
            except (TypeError, ValueError):
                count = 0
            # 0 도 저장 (빌드됨 표시 → is_empty() False). 음수는 0 으로.
            self._remaining[cls] = max(0, count)
        # 전부 0 이어도 '빌드됨' 으로 보이게: 모두 0 이면 is_complete()=True 가 맞음
        return self

    # ── 조회 ──────────────────────────────────────────────────────────────
    def remaining(self, class_name: str) -> int:
        return self._remaining.get(class_name, 0)

    def total_remaining(self) -> int:
        return sum(self._remaining.values())

    def is_complete(self) -> bool:
        """빌드된 항목이 있고 전부 0 이면 완료. (빈 task_list 는 미완료로 취급)"""
        return bool(self._remaining) and self.total_remaining() == 0

    def is_empty(self) -> bool:
        """유효 항목이 하나도 없음 (OCR 실패/미빌드)."""
        return not self._remaining

    def active_classes(self) -> list[str]:
        """잔여 > 0 인 class 목록 (잔여 많은 순)."""
        return [c for c, n in sorted(
            self._remaining.items(), key=lambda kv: kv[1], reverse=True) if n > 0]

    def next_target_class(self) -> str | None:
        """다음에 처리할 우선 class (잔여 최다). 없으면 None."""
        actives = self.active_classes()
        return actives[0] if actives else None

    # ── 갱신 ──────────────────────────────────────────────────────────────
    def decrement(self, class_name: str, n: int = 1) -> int:
        """class_name 잔여를 n 만큼 차감 (0 미만 방지). 차감 후 잔여 반환."""
        if class_name not in self._remaining:
            return 0
        self._remaining[class_name] = max(0, self._remaining[class_name] - n)
        return self._remaining[class_name]

    # ── 표시 ──────────────────────────────────────────────────────────────
    def as_dict(self) -> dict[str, int]:
        return dict(self._remaining)

    def __str__(self) -> str:
        if not self._remaining:
            return 'TaskList(empty)'
        body = ', '.join(f'{c}:{n}' for c, n in sorted(self._remaining.items()))
        tail = f' | unmapped={self.unmapped}' if self.unmapped else ''
        return f'TaskList({body}){tail}'
