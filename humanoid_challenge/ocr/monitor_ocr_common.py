#!/usr/bin/env python3
"""Monitor VLM OCR 통합 공용 모듈 — 미션(A|C) × 백엔드(gemini|qwen) 2×2.

기존 `mission_c_ocr_common.py`(미션 C 전용, 백엔드 무관 plumbing)를 재사용/확장해
미션 A(부품 수량 테이블)까지 한곳에서 처리한다. 두 축은 서로 독립이다:

  - mission:  'a' = parts count table   / 'c' = peg→nut sequence
              → 프롬프트 + JSON 스키마 + 정규화가 미션마다 다르다.
  - backend:  'gemini' = LetsUr 게이트웨이(Gemini, 외부 API)
              'qwen'   = 로컬 vLLM(Qwen2.5-VL, OpenAI 호환)
              → endpoint/auth/페이로드 강제방식(response_format vs guided_json)만 다르다.

두 백엔드 모두 OpenAI 호환 `/v1/chat/completions` 를 호출하므로 호출/파싱/정규화
로직은 공유된다(공정 비교 + 단일 코드). 무거운 의존성은 import 하지 않는다
(cv2 는 PPM 변환 시 함수 안에서 lazy import).
"""
from __future__ import annotations

import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

# 같은 디렉토리의 미션 C 공용 모듈을 재사용 (스크립트 직접 실행/패키지 양쪽 지원).
sys.path.insert(0, str(Path(__file__).resolve().parent))

from mission_c_ocr_common import (  # noqa: E402  (C 전용 모듈에서 공유 plumbing 재사용)
    CANONICAL_PARTS,
    LOCAL_IMAGE_SUFFIXES,
    NUM_PEGS,
    SEQ_SCHEMA,
    build_ssl_context,
    call_chat_completions,
    extract_output_text,
    format_result as format_sequence,
    frame_index_from_path,
    image_to_data_url,
    iter_frame_images,
    load_prompt,
    normalize_sequence_result,
    parse_model_json,
    sequence_mapping,
    usage_from_response,
)

# 재노출(이 모듈만 import 해도 plumbing 전부 사용 가능)
__all__ = [
    "CANONICAL_PARTS", "LOCAL_IMAGE_SUFFIXES", "NUM_PEGS", "SEQ_SCHEMA",
    "build_ssl_context", "call_chat_completions", "extract_output_text",
    "frame_index_from_path", "image_to_data_url", "iter_frame_images",
    "load_prompt", "normalize_sequence_result", "parse_model_json",
    "sequence_mapping", "usage_from_response", "format_sequence",
    "TOTAL_COUNT", "MAX_COUNT_PER_PART", "PARTS_SCHEMA",
    "normalize_parts_result", "format_parts", "parts_mapping",
    "MISSIONS", "BACKENDS", "MissionSpec", "BackendSpec",
    "resolve_gemini_api_key", "build_payload", "run_image",
]

# --------------------------------------------------------------------------- #
# 미션 A — 부품 수량 테이블
# --------------------------------------------------------------------------- #
TOTAL_COUNT = 5          # 5칸 수량의 합
MAX_COUNT_PER_PART = 3   # 한 부품 최대 수량

A_PROMPT_PATH = Path(__file__).with_name("gemini_mission_a_prompt.md")

PARTS_SCHEMA: dict[str, Any] = {
    "type": "object",
    "additionalProperties": False,
    "required": ["command", "fail", "fail_reason", "is_dummy"],
    "properties": {
        "command": {
            "type": "array",
            "minItems": len(CANONICAL_PARTS),
            "maxItems": len(CANONICAL_PARTS),
            "items": {
                "type": "object",
                "additionalProperties": False,
                "required": ["name", "count"],
                "properties": {
                    "name": {"type": "string", "enum": CANONICAL_PARTS},
                    "count": {"type": "integer", "minimum": -1,
                              "maximum": MAX_COUNT_PER_PART},
                },
            },
        },
        "fail": {"type": "boolean"},
        "fail_reason": {"type": "string"},
        "is_dummy": {"type": "boolean"},
    },
}


def _parts_fail(reason: str, is_dummy: bool = False) -> dict[str, Any]:
    return {
        "command": [{"name": n, "count": -1} for n in CANONICAL_PARTS],
        "fail": True,
        "fail_reason": reason,
        "is_dummy": is_dummy,
    }


def normalize_parts_result(result: dict[str, Any],
                           trust_valid_sum: bool = True) -> dict[str, Any]:
    """미션 A 모델 출력 정규화/검증 (gemini_monitor_command_ocr 포팅).

    command 를 CANONICAL_PARTS 순서로 재정렬하고, 5칸 수량 합=5 를 검증한다.
    trust_valid_sum=True 면 5칸 유효+합5 인데 모델이 fail 표시한 자기모순을 교정
    (게이트웨이 Gemini 에서 관측된 'sum is 5 인데 fail' 대응). 가림(-1)은 fail 유지.
    """
    if not isinstance(result, dict):
        return _parts_fail("model returned non-object")
    command_items = result.get("command")
    if not isinstance(command_items, list):
        return _parts_fail("model output did not contain a command array")

    by_name: dict[str, int] = {}
    duplicate_names: set[str] = set()
    invalid_items: list[str] = []
    for item in command_items:
        if not isinstance(item, dict):
            invalid_items.append(str(item))
            continue
        name = item.get("name")
        count = item.get("count")
        if name in by_name:
            duplicate_names.add(str(name))
        if name not in CANONICAL_PARTS or not isinstance(count, int):
            invalid_items.append(json.dumps(item, ensure_ascii=False))
            continue
        by_name[name] = count

    normalized = [{"name": n, "count": by_name.get(n, -1)} for n in CANONICAL_PARTS]

    fail = bool(result.get("fail", True))
    fail_reason = str(result.get("fail_reason") or "")
    is_dummy = bool(result.get("is_dummy", False))

    errors: list[str] = []
    if invalid_items:
        errors.append("invalid command items")
    if duplicate_names:
        errors.append(f"duplicate part names: {', '.join(sorted(duplicate_names))}")

    counts = [c["count"] for c in normalized]
    missing = [c["name"] for c in normalized if c["count"] == -1]
    out_of_range = [f"{c['name']}={c['count']}" for c in normalized
                    if c["count"] < -1 or c["count"] > MAX_COUNT_PER_PART]
    if out_of_range:
        errors.append(f"count outside -1..{MAX_COUNT_PER_PART}: {', '.join(out_of_range)}")
    if missing:
        errors.append(f"missing counts: {', '.join(missing)}")

    structurally_valid = not (missing or out_of_range or invalid_items or duplicate_names)
    if not missing and not out_of_range:
        total = sum(counts)
        if total != TOTAL_COUNT:
            errors.append(f"count sum {total} != {TOTAL_COUNT}")
            structurally_valid = False

    if errors:
        fail = True
        joined = "; ".join(errors)
        fail_reason = f"{fail_reason}; {joined}" if fail_reason else joined
    elif fail and trust_valid_sum and structurally_valid:
        fail = False

    if not fail:
        fail_reason = ""
        is_dummy = False
    elif not fail_reason:
        fail_reason = "model marked result as failed"

    return {"command": normalized, "fail": fail,
            "fail_reason": fail_reason, "is_dummy": is_dummy}


def format_parts(result: dict[str, Any]) -> str:
    if result.get("fail"):
        return f"FAIL: {result.get('fail_reason', '')}"
    parts = [f"{c['name']}:{c['count']}" for c in result.get("command", [])]
    return "OK  " + " | ".join(parts)


def parts_mapping(result: dict[str, Any]) -> dict[str, int] | None:
    """성공 결과를 {part_name: count} 매핑으로. fail 이면 None."""
    if result.get("fail") or not result.get("command"):
        return None
    return {c["name"]: c["count"] for c in result["command"]}


# --------------------------------------------------------------------------- #
# 미션 레지스트리 (프롬프트 + 스키마 + 정규화 + 포맷)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class MissionSpec:
    key: str
    prompt_path: Path
    schema: dict[str, Any]
    schema_name: str
    normalize: Callable[[dict[str, Any]], dict[str, Any]]
    formatter: Callable[[dict[str, Any]], str]


MISSIONS: dict[str, MissionSpec] = {
    "a": MissionSpec(
        key="a",
        prompt_path=A_PROMPT_PATH,
        schema=PARTS_SCHEMA,
        schema_name="monitor_command_result",
        normalize=normalize_parts_result,
        formatter=format_parts,
    ),
    "c": MissionSpec(
        key="c",
        prompt_path=Path(__file__).with_name("gemini_mission_c_prompt.md"),
        schema=SEQ_SCHEMA,
        schema_name="mission_c_sequence",
        normalize=normalize_sequence_result,
        formatter=format_sequence,
    ),
}


# --------------------------------------------------------------------------- #
# 백엔드 레지스트리 (endpoint/auth/페이로드 강제방식)
# --------------------------------------------------------------------------- #
# WARNING: 평문 게이트웨이 키 폴백. 환경변수(LETSUR_API_KEY)가 있으면 그것이 우선.
# 레포 공개 시 노출되므로 챌린지 종료 후 반드시 rotate 할 것.
DEFAULT_GEMINI_API_KEY = "sk-Cjf9jLcEW8zPDiNphJvflg"


def resolve_gemini_api_key() -> str:
    return (os.environ.get("LETSUR_API_KEY")
            or os.environ.get("OPENAI_API_KEY")
            or DEFAULT_GEMINI_API_KEY)


@dataclass(frozen=True)
class BackendSpec:
    key: str
    default_base_url: str
    default_model: str
    json_mode: str           # 'response_format' | 'guided_json'
    needs_api_key: bool
    reasoning_disable: bool   # Gemini 2.x thinking 토큰 절감
    temperature: float | None


BACKENDS: dict[str, BackendSpec] = {
    "gemini": BackendSpec(
        key="gemini",
        default_base_url=os.environ.get("LETSUR_BASE_URL", "https://gw.letsur.ai/v1"),
        default_model=os.environ.get("OCR_VISION_MODEL", "gemini-2.5-flash"),
        json_mode="response_format",
        needs_api_key=True,
        reasoning_disable=True,
        temperature=None,
    ),
    "qwen": BackendSpec(
        key="qwen",
        default_base_url=os.environ.get("QWEN_BASE_URL", "http://localhost:8000/v1"),
        default_model=os.environ.get("QWEN_MODEL", "Qwen/Qwen2.5-VL-7B-Instruct"),
        json_mode="guided_json",
        needs_api_key=False,
        reasoning_disable=False,
        temperature=0.0,
    ),
}


def build_payload(backend: BackendSpec, mission: MissionSpec,
                  image_data_url: str, prompt: str, model: str,
                  max_tokens: int = 1500) -> dict[str, Any]:
    """미션 스키마 + 백엔드 강제방식에 맞춘 OpenAI 호환 chat/completions payload."""
    payload: dict[str, Any] = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url",
                     "image_url": {"url": image_data_url, "detail": "high"}},
                ],
            }
        ],
        "max_tokens": max_tokens,
    }
    if backend.json_mode == "guided_json":
        payload["guided_json"] = mission.schema
    else:
        payload["response_format"] = {
            "type": "json_schema",
            "json_schema": {
                "name": mission.schema_name,
                "strict": True,
                "schema": mission.schema,
            },
        }
    if backend.reasoning_disable:
        payload["reasoning_effort"] = "disable"
    if backend.temperature is not None:
        payload["temperature"] = backend.temperature
    return payload


def run_image(mission_key: str, backend_key: str, image_path: Path,
              model: str | None = None, base_url: str | None = None,
              api_key: str | None = None, prompt_path: Path | None = None,
              timeout_sec: float = 120.0, ssl_ctx: Any = None) -> dict[str, Any]:
    """이미지 1장 → (미션, 백엔드) 선택 → VLM 호출 → 파싱/정규화한 결과 dict."""
    mission = MISSIONS[mission_key]
    backend = BACKENDS[backend_key]
    model = model or backend.default_model
    base_url = base_url or backend.default_base_url
    if api_key is None and backend.needs_api_key:
        api_key = resolve_gemini_api_key() if backend_key == "gemini" else None

    prompt = load_prompt(prompt_path or mission.prompt_path)
    data_url = image_to_data_url(image_path)
    payload = build_payload(backend, mission, data_url, prompt, model)
    resp = call_chat_completions(payload, base_url, api_key=api_key or None,
                                 timeout_sec=timeout_sec, ssl_context=ssl_ctx)
    text = extract_output_text(resp)
    parsed = parse_model_json(text)
    return mission.normalize(parsed)
