#!/usr/bin/env python3
"""Mission C 모니터 OCR A/B 비교 — Gemini(게이트웨이) vs 로컬 Qwen2.5-VL(vLLM).

동일한 프레임 집합을 여러 백엔드에 통과시켜 (1) 서로 간 일치율, (2) 지연시간,
(3) 실패율, (4) (정답 제공 시) peg→nut 정확도를 한 표로 정리한다. 두 백엔드 모두
`mission_c_ocr_common.normalize_sequence_result` 로 정규화한 뒤 비교하므로 공정하다.

백엔드 지정:
  --gemini                 Gemini(게이트웨이) 백엔드 포함 (기본 포함; --no-gemini 로 제외)
  --qwen 'label=URL|MODEL' 로컬 Qwen vLLM 백엔드 추가(반복 가능). 예:
        --qwen '3B=http://localhost:8000/v1|Qwen/Qwen2.5-VL-3B-Instruct'
        --qwen '7B=http://localhost:8001/v1|Qwen/Qwen2.5-VL-7B-Instruct'
  미지정 시 위 3B/7B 두 개를 기본 사용.

정답(ground truth, 선택):
  --gt path.json  형식 둘 다 허용:
    A) {"5": {"1":"플랜지 너트","2":"기어 링","3":"스페이서 링","4":"육각 너트"}, ...}
    B) gemini/qwen 스크립트의 --out 결과(list of {frame,result}) — 신뢰 백엔드로 GT 부트스트랩.

usage:
  python3 compare_mission_c_ocr.py <image_dir> [frame_indices...] \\
      [--qwen '3B=...|...'] [--no-gemini] [--gt gt.json] [--out compare.json]

이 PC 엔 GPU 가 없어 vLLM 백엔드는 응답하지 않는다. 그 경우 해당 백엔드는 프레임마다
연결 실패(FAIL: exception ...)로 기록되며, GPU 머신에서 vLLM 서버를 띄운 뒤 동일 명령으로
실제 비교 표를 얻는다. (게이트웨이 백엔드만 이 PC 에서 동작.)
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Callable

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mission_c_ocr_common as common  # noqa: E402
import qwen_mission_c_ocr as qwen  # noqa: E402

Runner = Callable[[Path], dict[str, Any]]

DEFAULT_QWEN_BACKENDS = [
    "3B=http://localhost:8000/v1|Qwen/Qwen2.5-VL-3B-Instruct",
    "7B=http://localhost:8001/v1|Qwen/Qwen2.5-VL-7B-Instruct",
]


# --------------------------------------------------------------------------- #
# 백엔드 러너 구성
# --------------------------------------------------------------------------- #
def make_gemini_runner(ssl_ctx: Any) -> Runner:
    # 게이트웨이 스크립트는 raw JSON 을 돌려주므로 공용 정규화를 한 번 더 적용한다.
    import gemini_mission_c_ocr as gemini

    api_key = gemini.resolve_api_key()

    def _run(image_path: Path) -> dict[str, Any]:
        raw = gemini.run_image(
            image_path, gemini.DEFAULT_MODEL, gemini.DEFAULT_BASE_URL,
            api_key, ssl_ctx,
        )
        return common.normalize_sequence_result(raw)

    return _run


def make_qwen_runner(base_url: str, model: str, ssl_ctx: Any) -> Runner:
    def _run(image_path: Path) -> dict[str, Any]:
        return qwen.run_image(
            image_path, model=model, base_url=base_url, ssl_ctx=ssl_ctx,
        )

    return _run


def parse_qwen_spec(spec: str) -> tuple[str, str, str]:
    """'label=URL|MODEL' → (label, url, model)."""
    if "=" not in spec or "|" not in spec:
        raise ValueError(f"bad --qwen spec {spec!r}; expected 'label=URL|MODEL'")
    label, rest = spec.split("=", 1)
    url, model = rest.split("|", 1)
    return label.strip(), url.strip(), model.strip()


# --------------------------------------------------------------------------- #
# Ground truth 로딩
# --------------------------------------------------------------------------- #
def load_ground_truth(path: Path) -> dict[int, dict[int, str]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    gt: dict[int, dict[int, str]] = {}
    if isinstance(data, dict):  # 형식 A
        for frame_key, mapping in data.items():
            gt[int(frame_key)] = {int(p): n for p, n in mapping.items()}
    elif isinstance(data, list):  # 형식 B (스크립트 --out 결과)
        for row in data:
            res = row.get("result", {})
            m = common.sequence_mapping(common.normalize_sequence_result(res))
            if m is not None:
                gt[int(row["frame"])] = m
    else:
        raise ValueError("unsupported ground-truth JSON shape")
    return gt


# --------------------------------------------------------------------------- #
# 비교 실행 + 지표
# --------------------------------------------------------------------------- #
def run_comparison(
    images: list[Path],
    runners: dict[str, Runner],
    gt: dict[int, dict[int, str]] | None,
) -> dict[str, Any]:
    per_frame: list[dict[str, Any]] = []
    for img in images:
        try:
            fi = common.frame_index_from_path(img)
        except ValueError:
            fi = -1
        row: dict[str, Any] = {"frame": fi, "image": img.name, "backends": {}}
        for label, runner in runners.items():
            t0 = time.perf_counter()
            try:
                res = runner(img)
            except Exception as exc:  # noqa: BLE001
                res = {"sequence": [], "fail": True,
                       "fail_reason": f"exception: {exc}", "is_dummy": False}
            dt = time.perf_counter() - t0
            row["backends"][label] = {
                "result": res,
                "latency_sec": round(dt, 3),
                "mapping": common.sequence_mapping(res),
            }
        per_frame.append(row)
        _print_frame_row(row, list(runners), gt)

    summary = _summarize(per_frame, list(runners), gt)
    return {"frames": per_frame, "summary": summary}


def _print_frame_row(row, labels, gt) -> None:
    fi = row["frame"]
    print(f"\n[f{fi:03d}] {row['image']}")
    gt_map = gt.get(fi) if gt else None
    if gt_map:
        print(f"   GT   : {_fmt_map(gt_map)}")
    for label in labels:
        b = row["backends"][label]
        line = common.format_result(b["result"])
        acc = ""
        if gt_map and b["mapping"] is not None:
            n = sum(1 for p in (1, 2, 3, 4) if b["mapping"].get(p) == gt_map.get(p))
            acc = f"  [{n}/4]"
        print(f"   {label:<5}: {line}  ({b['latency_sec']}s){acc}")


def _fmt_map(m: dict[int, str]) -> str:
    return " | ".join(f"pipe{p}={m[p]}" for p in sorted(m))


def _summarize(per_frame, labels, gt) -> dict[str, Any]:
    summary: dict[str, Any] = {"n_frames": len(per_frame), "backends": {}}
    ref = labels[0] if labels else None
    for label in labels:
        n_ok = lat_sum = 0
        exact = peg_correct = peg_total = 0
        agree_ref = agree_total = 0
        for row in per_frame:
            b = row["backends"][label]
            lat_sum += b["latency_sec"]
            m = b["mapping"]
            if m is not None:
                n_ok += 1
            # vs ground truth
            if gt is not None:
                gt_map = gt.get(row["frame"])
                if gt_map is not None and m is not None:
                    peg_total += 4
                    hit = sum(1 for p in (1, 2, 3, 4) if m.get(p) == gt_map.get(p))
                    peg_correct += hit
                    if hit == 4:
                        exact += 1
            # vs reference backend (보통 gemini)
            if ref and label != ref:
                rm = row["backends"][ref]["mapping"]
                if rm is not None and m is not None:
                    agree_total += 1
                    if rm == m:
                        agree_ref += 1
        n = len(per_frame) or 1
        entry: dict[str, Any] = {
            "n_ok": n_ok,
            "fail_rate": round(1 - n_ok / n, 3),
            "mean_latency_sec": round(lat_sum / n, 3),
        }
        if gt is not None and peg_total:
            entry["exact_match_rate"] = round(exact / (peg_total // 4), 3)
            entry["per_peg_accuracy"] = round(peg_correct / peg_total, 3)
        if ref and label != ref and agree_total:
            entry[f"agreement_vs_{ref}"] = round(agree_ref / agree_total, 3)
        summary["backends"][label] = entry
    return summary


def _print_summary(summary) -> None:
    print("\n" + "=" * 64)
    print(f"SUMMARY  ({summary['n_frames']} frames)")
    print("=" * 64)
    for label, e in summary["backends"].items():
        bits = [f"ok={e['n_ok']}", f"fail_rate={e['fail_rate']}",
                f"mean_lat={e['mean_latency_sec']}s"]
        if "exact_match_rate" in e:
            bits.append(f"exact={e['exact_match_rate']}")
            bits.append(f"peg_acc={e['per_peg_accuracy']}")
        for k in e:
            if k.startswith("agreement_vs_"):
                bits.append(f"{k}={e[k]}")
        print(f"  {label:<6}: " + "  ".join(bits))


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image_dir", type=Path)
    parser.add_argument("frames", nargs="*", type=int)
    parser.add_argument("--gemini", dest="gemini", action="store_true", default=True)
    parser.add_argument("--no-gemini", dest="gemini", action="store_false")
    parser.add_argument("--qwen", action="append", default=None,
                        help="repeatable 'label=URL|MODEL'")
    parser.add_argument("--gt", type=Path, default=None, help="ground-truth JSON")
    parser.add_argument("--out", type=Path, default=None, help="comparison JSON path")
    args = parser.parse_args()

    ssl_ctx = common.build_ssl_context()

    runners: dict[str, Runner] = {}
    if args.gemini:
        try:
            runners["gemini"] = make_gemini_runner(ssl_ctx)
        except Exception as exc:  # noqa: BLE001
            print(f"WARN: gemini backend unavailable: {exc}", file=sys.stderr)

    for spec in (args.qwen if args.qwen is not None else DEFAULT_QWEN_BACKENDS):
        label, url, model = parse_qwen_spec(spec)
        runners[label] = make_qwen_runner(url, model, ssl_ctx)

    if not runners:
        print("no backends configured", file=sys.stderr)
        return 2

    if args.image_dir.is_dir():
        images = common.iter_frame_images(args.image_dir, args.frames or None)
    else:
        images = [args.image_dir]

    gt = load_ground_truth(args.gt) if args.gt else None

    report = run_comparison(images, runners, gt)
    _print_summary(report["summary"])

    if args.out is not None:
        args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2),
                            encoding="utf-8")
        print(f"\nsaved -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
