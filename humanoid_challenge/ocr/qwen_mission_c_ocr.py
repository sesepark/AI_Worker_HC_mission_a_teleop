#!/usr/bin/env python3
"""로컬 VLM(Qwen2.5-VL) Mission C 모니터 OCR — vLLM OpenAI 호환 서버 백엔드.

`gemini_mission_c_ocr.py` 와 **동일한 프롬프트·스키마·정규화·출력 포맷**을 쓰되,
LetsUr 게이트웨이 대신 로컬에서 띄운 vLLM OpenAI 호환 서버
(`/v1/chat/completions`)를 호출한다. 따라서 두 백엔드의 결과를 1:1 로 비교할 수 있다.

전제: 이 PC 가 아니라 GPU 머신에서 아래처럼 vLLM 서버를 먼저 띄운다.

    # 3B
    vllm serve Qwen/Qwen2.5-VL-3B-Instruct \\
        --port 8000 --max-model-len 8192 --limit-mm-per-prompt image=1
    # 7B
    vllm serve Qwen/Qwen2.5-VL-7B-Instruct \\
        --port 8001 --max-model-len 8192 --limit-mm-per-prompt image=1

JSON 스키마 강제:
  vLLM 은 OpenAI 표준 `response_format` 외에 vLLM 고유 `guided_json` 을 지원한다.
  기본은 `guided_json`(가장 안정적). `--json-mode response_format` 으로 OpenAI
  스타일 강제도 가능(신형 vLLM). 둘 중 무엇을 쓰든 출력은 코드에서 한 번 더
  `normalize_sequence_result` 로 검증하므로 결과 형식은 동일하다.

설정(환경변수 우선):
  QWEN_BASE_URL   기본 'http://localhost:8000/v1'
  QWEN_MODEL      기본 'Qwen/Qwen2.5-VL-7B-Instruct'
  QWEN_API_KEY    vLLM 은 보통 인증 불필요. --api-key 로 띄웠을 때만 사용.

usage:
  python3 qwen_mission_c_ocr.py <image_dir_or_file> [frame_indices...] \\
      [--model Qwen/Qwen2.5-VL-3B-Instruct] [--base-url http://localhost:8000/v1]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any

# 같은 디렉토리의 공용 모듈 import (스크립트 직접 실행/패키지 양쪽 지원).
sys.path.insert(0, str(Path(__file__).resolve().parent))

from mission_c_ocr_common import (  # noqa: E402
    DEFAULT_PROMPT_PATH,
    LOCAL_IMAGE_SUFFIXES,
    SEQ_SCHEMA,
    build_ssl_context,
    call_chat_completions,
    extract_output_text,
    format_result,
    frame_index_from_path,
    image_to_data_url,
    iter_frame_images,
    load_prompt,
    normalize_sequence_result,
    parse_model_json,
)

DEFAULT_BASE_URL = os.environ.get("QWEN_BASE_URL", "http://localhost:8000/v1")
DEFAULT_MODEL = os.environ.get("QWEN_MODEL", "Qwen/Qwen2.5-VL-7B-Instruct")
DEFAULT_API_KEY = os.environ.get("QWEN_API_KEY", "")  # vLLM 기본은 인증 없음


def build_payload(
    image_data_url: str,
    prompt: str,
    model: str,
    json_mode: str = "guided_json",
    max_tokens: int = 1500,
    temperature: float = 0.0,
) -> dict[str, Any]:
    """vLLM OpenAI 호환 chat/completions 페이로드.

    Gemini 페이로드와 거의 동일하되 (1) Qwen 엔 무의미한 `reasoning_effort` 제거,
    (2) 결정적 비교를 위해 `temperature=0`, (3) 스키마 강제 방식 선택.
    """
    payload: dict[str, Any] = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url",
                     "image_url": {"url": image_data_url}},
                ],
            }
        ],
        "max_tokens": max_tokens,
        "temperature": temperature,
    }
    if json_mode == "response_format":
        payload["response_format"] = {
            "type": "json_schema",
            "json_schema": {
                "name": "mission_c_sequence",
                "strict": True,
                "schema": SEQ_SCHEMA,
            },
        }
    else:  # guided_json (vLLM 고유, 기본/가장 안정적)
        payload["guided_json"] = SEQ_SCHEMA
    return payload


def run_image(
    image_path: Path,
    model: str = DEFAULT_MODEL,
    base_url: str = DEFAULT_BASE_URL,
    api_key: str = DEFAULT_API_KEY,
    prompt_path: Path = DEFAULT_PROMPT_PATH,
    json_mode: str = "guided_json",
    timeout_sec: float = 180.0,
    ssl_ctx: Any = None,
) -> dict[str, Any]:
    prompt = load_prompt(prompt_path)
    data_url = image_to_data_url(image_path)
    payload = build_payload(data_url, prompt, model, json_mode=json_mode)
    resp = call_chat_completions(
        payload, base_url, api_key=api_key or None,
        timeout_sec=timeout_sec, ssl_context=ssl_ctx,
    )
    text = extract_output_text(resp)
    parsed = parse_model_json(text)
    return normalize_sequence_result(parsed)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("target", type=Path, help="image file or directory of frames")
    parser.add_argument("frames", nargs="*", type=int, help="frame indices (dir mode)")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--base-url", default=DEFAULT_BASE_URL)
    parser.add_argument("--api-key", default=DEFAULT_API_KEY)
    parser.add_argument("--prompt", type=Path, default=DEFAULT_PROMPT_PATH)
    parser.add_argument("--json-mode", choices=["guided_json", "response_format"],
                        default="guided_json")
    parser.add_argument("--out", type=Path, default=None, help="results JSON path")
    args = parser.parse_args()

    ssl_ctx = build_ssl_context()

    if args.target.is_dir():
        images = iter_frame_images(args.target, args.frames or None)
    else:
        images = [args.target]

    results = []
    for img in images:
        try:
            res = run_image(
                img, args.model, args.base_url, args.api_key,
                args.prompt, args.json_mode, ssl_ctx=ssl_ctx,
            )
        except Exception as exc:  # noqa: BLE001
            res = {"sequence": [], "fail": True,
                   "fail_reason": f"exception: {exc}", "is_dummy": False}
        try:
            fi = frame_index_from_path(img)
        except ValueError:
            fi = -1
        print(f"[f{fi:03d}] {format_result(res)}")
        results.append({"frame": fi, "image": img.name, "result": res})

    if args.out is not None:
        args.out.write_text(json.dumps(results, ensure_ascii=False, indent=2),
                            encoding="utf-8")
        print(f"\nsaved -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
