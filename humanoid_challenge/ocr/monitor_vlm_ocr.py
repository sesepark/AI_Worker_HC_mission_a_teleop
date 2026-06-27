#!/usr/bin/env python3
"""Monitor VLM OCR 통합 진입점 — 미션(A|C) × 백엔드(gemini|qwen) 단일 CLI.

미션 A(부품 수량 테이블)와 미션 C(peg→nut 순차 조립)를 하나의 코드로 읽고,
게이트웨이 Gemini 와 로컬 Qwen2.5-VL(vLLM) 백엔드를 인자로 골라 쓴다.

usage:
  # 미션 C, 게이트웨이 Gemini (기본 백엔드)
  python3 monitor_vlm_ocr.py --mission c <image_dir_or_file> [frame_idx...]
  # 미션 A, 로컬 Qwen
  python3 monitor_vlm_ocr.py --mission a --backend qwen <image> \\
      --base-url http://localhost:8000/v1 --model Qwen/Qwen2.5-VL-7B-Instruct

환경변수:
  LETSUR_API_KEY / OPENAI_API_KEY   gemini 백엔드 키(없으면 하드코딩 폴백)
  LETSUR_BASE_URL, OCR_VISION_MODEL gemini 기본 endpoint/model 오버라이드
  QWEN_BASE_URL, QWEN_MODEL, QWEN_API_KEY   qwen 기본값 오버라이드
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from monitor_ocr_common import (  # noqa: E402
    BACKENDS,
    LOCAL_IMAGE_SUFFIXES,
    MISSIONS,
    build_ssl_context,
    frame_index_from_path,
    iter_frame_images,
    run_image,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("target", type=Path, help="image file or directory of frames")
    parser.add_argument("frames", nargs="*", type=int, help="frame indices (dir mode)")
    parser.add_argument("--mission", choices=sorted(MISSIONS), required=True,
                        help="a=부품 수량 테이블 / c=peg→nut 순차")
    parser.add_argument("--backend", choices=sorted(BACKENDS), default="gemini",
                        help="gemini=게이트웨이 / qwen=로컬 vLLM (기본 gemini)")
    parser.add_argument("--model", default=None, help="백엔드 기본 모델 오버라이드")
    parser.add_argument("--base-url", default=None, help="백엔드 기본 endpoint 오버라이드")
    parser.add_argument("--api-key", default=None)
    parser.add_argument("--prompt", type=Path, default=None,
                        help="미션 기본 프롬프트 오버라이드")
    parser.add_argument("--out", type=Path, default=None, help="results JSON path")
    args = parser.parse_args()

    ssl_ctx = build_ssl_context()

    if args.target.is_dir():
        images = iter_frame_images(args.target, args.frames or None)
    else:
        images = [args.target]

    mission = MISSIONS[args.mission]
    results = []
    for img in images:
        try:
            res = run_image(
                args.mission, args.backend, img,
                model=args.model, base_url=args.base_url, api_key=args.api_key,
                prompt_path=args.prompt, ssl_ctx=ssl_ctx,
            )
        except Exception as exc:  # noqa: BLE001
            res = {"fail": True, "fail_reason": f"exception: {exc}", "is_dummy": False}
        try:
            fi = frame_index_from_path(img)
        except ValueError:
            fi = -1
        print(f"[f{fi:03d}] {mission.formatter(res)}")
        results.append({"frame": fi, "image": img.name,
                        "mission": args.mission, "backend": args.backend,
                        "result": res})

    if args.out is not None:
        args.out.write_text(json.dumps(results, ensure_ascii=False, indent=2),
                            encoding="utf-8")
        print(f"\nsaved -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
