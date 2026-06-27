#!/usr/bin/env python3
"""Gemini(StaiX/LetsUr 게이트웨이) vision OCR — Mission C 모니터 순차 조립 지시.

Mission C 모니터는 '부품 순차 조립 지시(PART SEQUENTIAL ASSEMBLY INSTRUCTION)'
패널로, Peg(=pipe) 1..4 각각에 놓아야 할 너트 1종을 보여준다. 이 스크립트는
게이트웨이의 OpenAI 호환 `/v1/chat/completions` + `response_format=json_schema`
로 peg->nut 시퀀스(strict JSON)를 읽는다. (A 미션의 count-table OCR과 별개 스키마.)

설정(환경변수 우선):
  LETSUR_API_KEY    게이트웨이 API 키. 없으면 OPENAI_API_KEY, 그래도 없으면
                    아래 DEFAULT_API_KEY 폴백.
  LETSUR_BASE_URL   기본 'https://gw.letsur.ai/v1'.
  OCR_VISION_MODEL  기본 'gemini-2.5-flash'.

경고: DEFAULT_API_KEY 는 평문 키다. 레포가 공개되면 즉시 노출되므로 챌린지 종료
후 반드시 폐기/교체(rotate)할 것. 가능하면 환경변수 사용을 권장한다.

usage:
  python3 gemini_mission_c_ocr.py <image_dir_or_file> [frame_indices...]
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import re
import ssl
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

CANONICAL_PARTS = ["플랜지 너트", "기어 링", "스페이서 링", "육각 너트", "돔 너트"]

# WARNING: 평문 게이트웨이 키 폴백. 환경변수(LETSUR_API_KEY)가 있으면 그것이 우선.
# 레포 공개 시 노출되므로 챌린지 종료 후 반드시 rotate 할 것.
DEFAULT_API_KEY = "sk-Cjf9jLcEW8zPDiNphJvflg"

DEFAULT_BASE_URL = os.environ.get("LETSUR_BASE_URL", "https://gw.letsur.ai/v1")
DEFAULT_MODEL = os.environ.get("OCR_VISION_MODEL", "gemini-2.5-flash")
PROMPT_PATH = Path(__file__).with_name("gemini_mission_c_prompt.md")

SUPPORTED_IMAGE_MIME = {
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".webp": "image/webp",
    ".gif": "image/gif",
}
LOCAL_IMAGE_SUFFIXES = SUPPORTED_IMAGE_MIME.keys() | {".ppm", ".bmp"}

SEQ_SCHEMA: dict[str, Any] = {
    "type": "object",
    "additionalProperties": False,
    "required": ["sequence", "fail", "fail_reason", "is_dummy"],
    "properties": {
        "sequence": {
            "type": "array",
            "minItems": 4,
            "maxItems": 4,
            "items": {
                "type": "object",
                "additionalProperties": False,
                "required": ["peg", "nut", "size_mm"],
                "properties": {
                    "peg": {"type": "integer", "minimum": 1, "maximum": 4},
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


def resolve_api_key() -> str:
    return (
        os.environ.get("LETSUR_API_KEY")
        or os.environ.get("OPENAI_API_KEY")
        or DEFAULT_API_KEY
    )


def build_ssl_context() -> ssl.SSLContext:
    try:
        import certifi
    except ImportError:
        return ssl.create_default_context()
    return ssl.create_default_context(cafile=certifi.where())


def frame_index_from_path(path: Path) -> int:
    match = re.search(r"_(\d{6})$", path.stem)
    if not match:
        raise ValueError(f"Could not parse frame index from image name: {path.name}")
    return int(match.group(1))


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


def build_payload(image_data_url: str, prompt: str, model: str) -> dict[str, Any]:
    return {
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
        "response_format": {
            "type": "json_schema",
            "json_schema": {
                "name": "mission_c_sequence",
                "strict": True,
                "schema": SEQ_SCHEMA,
            },
        },
        "max_tokens": 1500,
        "reasoning_effort": "disable",
    }


def call_chat_completions(
    payload: dict[str, Any], api_key: str, base_url: str,
    timeout_sec: float, ssl_context: ssl.SSLContext,
) -> dict[str, Any]:
    endpoint = base_url.rstrip("/") + "/chat/completions"
    request = urllib.request.Request(
        endpoint,
        data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        headers={"Authorization": f"Bearer {api_key}",
                 "Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout_sec, context=ssl_context) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"Gateway API HTTP {exc.code}: {body}") from exc


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


def run_image(
    image_path: Path, model: str, base_url: str,
    api_key: str, ssl_ctx: ssl.SSLContext, timeout_sec: float = 60.0,
) -> dict[str, Any]:
    prompt = PROMPT_PATH.read_text(encoding="utf-8").strip()
    data_url = image_to_data_url(image_path)
    payload = build_payload(data_url, prompt, model)
    resp = call_chat_completions(payload, api_key, base_url, timeout_sec, ssl_ctx)
    text = extract_output_text(resp)
    try:
        return json.loads(text)
    except json.JSONDecodeError as exc:
        return {"sequence": [], "fail": True,
                "fail_reason": f"non-JSON: {exc}: {text[:200]}", "is_dummy": False}


def format_result(result: dict[str, Any]) -> str:
    if result.get("fail"):
        return f"FAIL: {result.get('fail_reason', '')}"
    parts = []
    for entry in sorted(result.get("sequence", []), key=lambda x: x.get("peg", 0)):
        parts.append(f"pipe{entry['peg']}={entry['nut']}({entry.get('size_mm', -1)}mm)")
    return "OK  " + " | ".join(parts)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("target", type=Path, help="image file or directory of frames")
    parser.add_argument("frames", nargs="*", type=int, help="frame indices (dir mode)")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--base-url", default=DEFAULT_BASE_URL)
    parser.add_argument("--out", type=Path, default=None, help="results JSON path")
    args = parser.parse_args()

    api_key = resolve_api_key()
    ssl_ctx = build_ssl_context()

    if args.target.is_dir():
        all_imgs = sorted(
            [p for p in args.target.iterdir()
             if p.suffix.lower() in LOCAL_IMAGE_SUFFIXES],
            key=frame_index_from_path,
        )
        images = [all_imgs[i] for i in args.frames] if args.frames else all_imgs
    else:
        images = [args.target]

    results = []
    for img in images:
        try:
            res = run_image(img, args.model, args.base_url, api_key, ssl_ctx)
        except Exception as exc:  # noqa: BLE001
            res = {"sequence": [], "fail": True,
                   "fail_reason": f"exception: {exc}", "is_dummy": False}
        fi = frame_index_from_path(img)
        print(f"[f{fi:03d}] {format_result(res)}")
        results.append({"frame": fi, "image": img.name, "result": res})

    if args.out is not None:
        args.out.write_text(json.dumps(results, ensure_ascii=False, indent=2),
                            encoding="utf-8")
        print(f"\nsaved -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
