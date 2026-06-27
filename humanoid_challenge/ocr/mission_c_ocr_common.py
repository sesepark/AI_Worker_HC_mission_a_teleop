#!/usr/bin/env python3
"""Mission C 모니터 OCR 공용 모듈 (백엔드 무관).

`gemini_mission_c_ocr.py`(LetsUr 게이트웨이/Gemini)와 `qwen_mission_c_ocr.py`
(로컬 vLLM/Qwen2.5-VL)가 **동일한** 프롬프트·스키마·정규화·출력 포맷을 쓰도록
공통 로직을 한곳에 모은다. 그래야 두 백엔드의 성능 비교가 공정하다.

설계 원칙:
  - 무거운 의존성(torch/transformers/vllm/jsonschema)은 import 하지 않는다.
  - cv2 는 PPM/BMP → PNG 변환에만 함수 안에서 lazy import 한다.
  - 결과 정규화/검증은 순수 파이썬으로 작성해 GPU 없이도 단위테스트가 돈다.

모든 백엔드는 OpenAI 호환 `/v1/chat/completions` 를 호출하므로(게이트웨이도,
vLLM 서버도 동일), 요청 전송/응답 추출/이미지 인코딩도 여기서 공유한다.
"""
from __future__ import annotations

import base64
import json
import re
import ssl
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

# Mission C 모니터에 등장하는 다섯 가지 너트/링의 표준 한글 이름.
CANONICAL_PARTS = ["플랜지 너트", "기어 링", "스페이서 링", "육각 너트", "돔 너트"]

# Mission C 모니터는 Peg(=pipe) 1..4 각각에 너트 1종을 지정한다.
NUM_PEGS = 4

# 게이트웨이/vLLM 둘 다 이 스키마로 strict JSON 을 강제한다.
# (게이트웨이: response_format=json_schema / vLLM: guided_json)
SEQ_SCHEMA: dict[str, Any] = {
    "type": "object",
    "additionalProperties": False,
    "required": ["sequence", "fail", "fail_reason", "is_dummy"],
    "properties": {
        "sequence": {
            "type": "array",
            "minItems": NUM_PEGS,
            "maxItems": NUM_PEGS,
            "items": {
                "type": "object",
                "additionalProperties": False,
                "required": ["peg", "nut", "size_mm"],
                "properties": {
                    "peg": {"type": "integer", "minimum": 1, "maximum": NUM_PEGS},
                    "nut": {"type": "string", "enum": CANONICAL_PARTS},
                    "size_mm": {"type": "integer", "minimum": -1, "maximum": 200},
                },
            },
        },
        "fail": {"type": "boolean"},
        "fail_reason": {"type": "string"},
        "is_dummy": {"type": "boolean"},
    },
}

# 모든 백엔드가 동일한 프롬프트를 사용한다(공정 비교). 이미 검증된 Mission C 프롬프트.
DEFAULT_PROMPT_PATH = Path(__file__).with_name("gemini_mission_c_prompt.md")

SUPPORTED_IMAGE_MIME = {
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".webp": "image/webp",
    ".gif": "image/gif",
}
LOCAL_IMAGE_SUFFIXES = SUPPORTED_IMAGE_MIME.keys() | {".ppm", ".bmp"}


# --------------------------------------------------------------------------- #
# 프레임/이미지 유틸
# --------------------------------------------------------------------------- #
def load_prompt(prompt_path: Path | None = None) -> str:
    return (prompt_path or DEFAULT_PROMPT_PATH).read_text(encoding="utf-8").strip()


def frame_index_from_path(path: Path) -> int:
    match = re.search(r"_(\d{6})$", path.stem)
    if not match:
        raise ValueError(f"Could not parse frame index from image name: {path.name}")
    return int(match.group(1))


def iter_frame_images(image_dir: Path, frames: list[int] | None = None) -> list[Path]:
    """디렉토리에서 프레임 이미지를 인덱스 순으로 정렬해 반환.

    `frames` 가 주어지면 정렬된 목록에서 그 위치(0-based)의 이미지만 고른다.
    (gemini_mission_c_ocr.py 의 dir 모드와 동일한 의미.)
    """
    all_imgs = sorted(
        [p for p in image_dir.iterdir()
         if p.is_file() and p.suffix.lower() in LOCAL_IMAGE_SUFFIXES],
        key=frame_index_from_path,
    )
    if frames:
        return [all_imgs[i] for i in frames]
    return all_imgs


def _convert_image_to_png_bytes(image_path: Path) -> bytes:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            f"{image_path.suffix} is not directly supported by the vision API. "
            "Install opencv-python so this script can convert it to PNG."
        ) from exc
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not read image: {image_path}")
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise ValueError(f"Could not encode image as PNG: {image_path}")
    return bytes(encoded)


def image_to_data_url(image_path: Path) -> str:
    suffix = image_path.suffix.lower()
    if suffix in SUPPORTED_IMAGE_MIME:
        mime_type = SUPPORTED_IMAGE_MIME[suffix]
        image_bytes = image_path.read_bytes()
    else:
        mime_type = "image/png"
        image_bytes = _convert_image_to_png_bytes(image_path)
    b64 = base64.b64encode(image_bytes).decode("ascii")
    return f"data:{mime_type};base64,{b64}"


# --------------------------------------------------------------------------- #
# OpenAI 호환 chat/completions 호출 (게이트웨이 & vLLM 공용)
# --------------------------------------------------------------------------- #
def build_ssl_context() -> ssl.SSLContext:
    try:
        import certifi
    except ImportError:
        return ssl.create_default_context()
    return ssl.create_default_context(cafile=certifi.where())


def call_chat_completions(
    payload: dict[str, Any],
    base_url: str,
    api_key: str | None = None,
    timeout_sec: float = 120.0,
    ssl_context: ssl.SSLContext | None = None,
) -> dict[str, Any]:
    endpoint = base_url.rstrip("/") + "/chat/completions"
    headers = {"Content-Type": "application/json"}
    if api_key:
        headers["Authorization"] = f"Bearer {api_key}"
    request = urllib.request.Request(
        endpoint,
        data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        headers=headers,
        method="POST",
    )
    try:
        with urllib.request.urlopen(
            request, timeout=timeout_sec, context=ssl_context
        ) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"OpenAI-compat API HTTP {exc.code}: {body}") from exc


def extract_output_text(response: dict[str, Any]) -> str:
    choices = response.get("choices")
    if not isinstance(choices, list):
        return ""
    texts: list[str] = []
    for choice in choices:
        message = choice.get("message") if isinstance(choice, dict) else None
        if not isinstance(message, dict):
            continue
        content = message.get("content")
        if isinstance(content, str):
            texts.append(content)
        elif isinstance(content, list):
            for part in content:
                if isinstance(part, dict) and isinstance(part.get("text"), str):
                    texts.append(part["text"])
    return "\n".join(texts).strip()


def usage_from_response(response: dict[str, Any]) -> dict[str, int]:
    usage = response.get("usage")
    if not isinstance(usage, dict):
        return {}
    return {
        k: int(v)
        for k, v in usage.items()
        if isinstance(v, (int, float)) and "tokens" in k
    }


# --------------------------------------------------------------------------- #
# JSON 파싱 / 정규화 / 검증 (백엔드 무관, 동일 적용 → 공정 비교)
# --------------------------------------------------------------------------- #
def _fail(reason: str, is_dummy: bool = False) -> dict[str, Any]:
    return {"sequence": [], "fail": True, "fail_reason": reason, "is_dummy": is_dummy}


_JSON_OBJ_RE = re.compile(r"\{.*\}", re.DOTALL)


def parse_model_json(text: str) -> dict[str, Any]:
    """모델 출력 텍스트에서 JSON 객체를 추출/파싱.

    guided_json/response_format 이 켜져 있으면 보통 순수 JSON 이지만, 코드펜스나
    잡설이 섞이는 경우(가드 미적용/구형 서버)를 대비해 첫 ``{...}`` 블록을 회수한다.
    """
    text = text.strip()
    if not text:
        return _fail("empty model output")
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass
    match = _JSON_OBJ_RE.search(text)
    if match:
        try:
            return json.loads(match.group(0))
        except json.JSONDecodeError as exc:
            return _fail(f"non-JSON: {exc}: {text[:200]}")
    return _fail(f"non-JSON: no object found: {text[:200]}")


def normalize_sequence_result(result: dict[str, Any]) -> dict[str, Any]:
    """모델 결과를 SEQ_SCHEMA 기준으로 코드 검증/정규화.

    guided decoding 이 꺼져 있거나 모델이 스키마를 어겨도 여기서 잡아내고, 두 백엔드에
    똑같이 적용해 비교의 공정성을 보장한다. 통과 시 sequence 는 peg 오름차순 정렬.
    """
    if not isinstance(result, dict):
        return _fail("model returned non-object")
    if result.get("is_dummy"):
        return _fail(result.get("fail_reason") or "no readable instruction panel",
                     is_dummy=True)
    if result.get("fail"):
        return _fail(result.get("fail_reason") or "model reported fail")

    seq = result.get("sequence")
    if not isinstance(seq, list):
        return _fail("sequence is not a list")
    if len(seq) != NUM_PEGS:
        return _fail(f"sequence must have {NUM_PEGS} entries, got {len(seq)}")

    seen_pegs: dict[int, str] = {}
    norm: list[dict[str, Any]] = []
    for entry in seq:
        if not isinstance(entry, dict):
            return _fail("sequence entry is not an object")
        peg = entry.get("peg")
        nut = entry.get("nut")
        if peg not in (1, 2, 3, 4):
            return _fail(f"invalid peg: {peg!r}")
        if peg in seen_pegs:
            return _fail(f"duplicate peg: {peg}")
        if nut not in CANONICAL_PARTS:
            return _fail(f"invalid nut name: {nut!r}")
        try:
            size = int(entry.get("size_mm", -1))
        except (TypeError, ValueError):
            size = -1
        if size != -1 and not (0 <= size <= 200):
            size = -1
        seen_pegs[peg] = nut
        norm.append({"peg": peg, "nut": nut, "size_mm": size})

    if set(seen_pegs) != {1, 2, 3, 4}:
        return _fail(f"pegs must be exactly 1..4, got {sorted(seen_pegs)}")

    norm.sort(key=lambda e: e["peg"])
    return {"sequence": norm, "fail": False, "fail_reason": "", "is_dummy": False}


def sequence_mapping(result: dict[str, Any]) -> dict[int, str] | None:
    """성공 결과를 {peg: nut} 매핑으로. fail 이면 None."""
    if result.get("fail") or not result.get("sequence"):
        return None
    return {e["peg"]: e["nut"] for e in result["sequence"]}


def format_result(result: dict[str, Any]) -> str:
    if result.get("fail"):
        return f"FAIL: {result.get('fail_reason', '')}"
    parts = []
    for entry in sorted(result.get("sequence", []), key=lambda x: x.get("peg", 0)):
        parts.append(f"pipe{entry['peg']}={entry['nut']}({entry.get('size_mm', -1)}mm)")
    return "OK  " + " | ".join(parts)
