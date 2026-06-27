#!/usr/bin/env python3
"""Unit tests for compare_mission_c_ocr — 네트워크/GPU 없이 stub 러너로 검증."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

import compare_mission_c_ocr as cmp  # noqa: E402
import mission_c_ocr_common as common  # noqa: E402


def ok_result(mapping):
    return common.normalize_sequence_result({
        "sequence": [{"peg": p, "nut": n, "size_mm": -1} for p, n in mapping.items()],
        "fail": False, "fail_reason": "", "is_dummy": False,
    })


GT_MAP = {1: "플랜지 너트", 2: "기어 링", 3: "스페이서 링", 4: "육각 너트"}
WRONG_MAP = {1: "돔 너트", 2: "기어 링", 3: "스페이서 링", 4: "육각 너트"}  # peg1 틀림


class SpecTests(unittest.TestCase):
    def test_parse_qwen_spec(self):
        label, url, model = cmp.parse_qwen_spec(
            "7B=http://localhost:8001/v1|Qwen/Qwen2.5-VL-7B-Instruct")
        self.assertEqual(label, "7B")
        self.assertEqual(url, "http://localhost:8001/v1")
        self.assertEqual(model, "Qwen/Qwen2.5-VL-7B-Instruct")

    def test_parse_qwen_spec_bad(self):
        with self.assertRaises(ValueError):
            cmp.parse_qwen_spec("no-separators")


class GroundTruthTests(unittest.TestCase):
    def test_load_format_a(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "gt.json"
            p.write_text(json.dumps({"5": {str(k): v for k, v in GT_MAP.items()}}),
                         encoding="utf-8")
            gt = cmp.load_ground_truth(p)
            self.assertEqual(gt[5], GT_MAP)

    def test_load_format_b_from_script_output(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "out.json"
            rows = [{"frame": 5, "image": "x.ppm", "result": ok_result(GT_MAP)}]
            p.write_text(json.dumps(rows, ensure_ascii=False), encoding="utf-8")
            gt = cmp.load_ground_truth(p)
            self.assertEqual(gt[5], GT_MAP)


class ComparisonTests(unittest.TestCase):
    def _images(self):
        return [Path("frame_000005.png"), Path("frame_000006.png")]

    def test_summary_metrics_vs_gt_and_reference(self):
        # gemini 항상 정답, qwen 은 f5 에서 peg1 오답
        runners = {
            "gemini": lambda img: ok_result(GT_MAP),
            "7B": lambda img: ok_result(
                WRONG_MAP if "000005" in img.name else GT_MAP),
        }
        gt = {5: GT_MAP, 6: GT_MAP}
        report = cmp.run_comparison(self._images(), runners, gt)
        g = report["summary"]["backends"]["gemini"]
        q = report["summary"]["backends"]["7B"]

        self.assertEqual(g["exact_match_rate"], 1.0)
        self.assertEqual(g["per_peg_accuracy"], 1.0)
        # qwen: f5 3/4 + f6 4/4 = 7/8
        self.assertEqual(q["per_peg_accuracy"], round(7 / 8, 3))
        self.assertEqual(q["exact_match_rate"], 0.5)  # f6 만 4/4
        # 일치율: f5 불일치, f6 일치 → 0.5
        self.assertEqual(q["agreement_vs_gemini"], 0.5)

    def test_fail_backend_counts(self):
        def boom(img):
            raise RuntimeError("connection refused")  # GPU 없는 PC 의 vLLM 백엔드 모사

        runners = {"gemini": lambda img: ok_result(GT_MAP), "7B": boom}
        report = cmp.run_comparison(self._images(), runners, None)
        q = report["summary"]["backends"]["7B"]
        self.assertEqual(q["n_ok"], 0)
        self.assertEqual(q["fail_rate"], 1.0)
        # 예외가 잡혀 결과 dict 으로 기록되는지
        self.assertTrue(report["frames"][0]["backends"]["7B"]["result"]["fail"])

    def test_latency_recorded(self):
        runners = {"gemini": lambda img: ok_result(GT_MAP)}
        report = cmp.run_comparison([Path("frame_000005.png")], runners, None)
        self.assertIn("latency_sec", report["frames"][0]["backends"]["gemini"])
        self.assertIn("mean_latency_sec", report["summary"]["backends"]["gemini"])


if __name__ == "__main__":
    unittest.main()
