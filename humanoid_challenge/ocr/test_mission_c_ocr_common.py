#!/usr/bin/env python3
"""Unit tests for mission_c_ocr_common — GPU/네트워크/무거운 의존성 없이 동작."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

import mission_c_ocr_common as common  # noqa: E402

VALID_SEQ = [
    {"peg": 1, "nut": "플랜지 너트", "size_mm": 19},
    {"peg": 2, "nut": "기어 링", "size_mm": 22},
    {"peg": 3, "nut": "스페이서 링", "size_mm": -1},
    {"peg": 4, "nut": "육각 너트", "size_mm": 17},
]


def valid_result():
    return {"sequence": [dict(e) for e in VALID_SEQ],
            "fail": False, "fail_reason": "", "is_dummy": False}


class ParseModelJsonTests(unittest.TestCase):
    def test_plain_json(self):
        out = common.parse_model_json('{"sequence": [], "fail": true}')
        self.assertTrue(out["fail"])

    def test_codefenced_json(self):
        text = "```json\n{\"fail\": false, \"sequence\": []}\n```"
        out = common.parse_model_json(text)
        self.assertFalse(out["fail"])

    def test_json_with_prose(self):
        text = 'Here is the answer: {"fail": false, "sequence": [1]} done'
        out = common.parse_model_json(text)
        self.assertEqual(out["sequence"], [1])

    def test_empty(self):
        out = common.parse_model_json("   ")
        self.assertTrue(out["fail"])
        self.assertIn("empty", out["fail_reason"])

    def test_garbage(self):
        out = common.parse_model_json("not json at all")
        self.assertTrue(out["fail"])
        self.assertIn("non-JSON", out["fail_reason"])


class NormalizeTests(unittest.TestCase):
    def test_valid_passes_and_sorts(self):
        scrambled = valid_result()
        scrambled["sequence"] = list(reversed(scrambled["sequence"]))
        out = common.normalize_sequence_result(scrambled)
        self.assertFalse(out["fail"], out["fail_reason"])
        self.assertEqual([e["peg"] for e in out["sequence"]], [1, 2, 3, 4])

    def test_duplicate_peg_fails(self):
        r = valid_result()
        r["sequence"][1]["peg"] = 1
        out = common.normalize_sequence_result(r)
        self.assertTrue(out["fail"])
        self.assertIn("duplicate", out["fail_reason"])

    def test_bad_nut_fails(self):
        r = valid_result()
        r["sequence"][0]["nut"] = "이상한 너트"
        out = common.normalize_sequence_result(r)
        self.assertTrue(out["fail"])
        self.assertIn("invalid nut", out["fail_reason"])

    def test_wrong_count_fails(self):
        r = valid_result()
        r["sequence"] = r["sequence"][:3]
        out = common.normalize_sequence_result(r)
        self.assertTrue(out["fail"])
        self.assertIn("4 entries", out["fail_reason"])

    def test_dummy_propagates(self):
        out = common.normalize_sequence_result(
            {"sequence": [], "fail": True, "fail_reason": "x", "is_dummy": True})
        self.assertTrue(out["fail"])
        self.assertTrue(out["is_dummy"])

    def test_model_fail_propagates(self):
        out = common.normalize_sequence_result(
            {"sequence": [], "fail": True, "fail_reason": "blurry", "is_dummy": False})
        self.assertTrue(out["fail"])
        self.assertIn("blurry", out["fail_reason"])

    def test_out_of_range_size_clamped(self):
        r = valid_result()
        r["sequence"][0]["size_mm"] = 9999
        out = common.normalize_sequence_result(r)
        self.assertFalse(out["fail"], out["fail_reason"])
        self.assertEqual(out["sequence"][0]["size_mm"], -1)

    def test_non_int_size_clamped(self):
        r = valid_result()
        r["sequence"][0]["size_mm"] = "19mm"
        out = common.normalize_sequence_result(r)
        self.assertFalse(out["fail"], out["fail_reason"])
        self.assertEqual(out["sequence"][0]["size_mm"], -1)

    def test_peg_zero_fails(self):
        r = valid_result()
        r["sequence"][0]["peg"] = 0
        out = common.normalize_sequence_result(r)
        self.assertTrue(out["fail"])


class HelperTests(unittest.TestCase):
    def test_sequence_mapping(self):
        m = common.sequence_mapping(common.normalize_sequence_result(valid_result()))
        self.assertEqual(m, {1: "플랜지 너트", 2: "기어 링",
                             3: "스페이서 링", 4: "육각 너트"})

    def test_sequence_mapping_none_on_fail(self):
        self.assertIsNone(common.sequence_mapping(
            {"sequence": [], "fail": True, "fail_reason": "x", "is_dummy": False}))

    def test_format_ok(self):
        s = common.format_result(common.normalize_sequence_result(valid_result()))
        self.assertTrue(s.startswith("OK"))
        self.assertIn("pipe1=플랜지 너트", s)

    def test_format_fail(self):
        s = common.format_result({"fail": True, "fail_reason": "nope"})
        self.assertTrue(s.startswith("FAIL"))

    def test_frame_index(self):
        p = Path("1782487026_672061633_000042.ppm")
        self.assertEqual(common.frame_index_from_path(p), 42)

    def test_frame_index_bad(self):
        with self.assertRaises(ValueError):
            common.frame_index_from_path(Path("nope.png"))

    def test_extract_output_text_str(self):
        resp = {"choices": [{"message": {"content": "hello"}}]}
        self.assertEqual(common.extract_output_text(resp), "hello")

    def test_extract_output_text_list(self):
        resp = {"choices": [{"message": {"content": [
            {"type": "text", "text": "a"}, {"type": "text", "text": "b"}]}}]}
        self.assertEqual(common.extract_output_text(resp), "a\nb")

    def test_iter_frame_images_orders_and_selects(self, ):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            dd = Path(d)
            for i in (2, 0, 1):
                (dd / f"x_000_00000{i}.ppm").write_bytes(b"")
            (dd / "ignore.txt").write_text("x")
            ordered = common.iter_frame_images(dd)
            self.assertEqual([common.frame_index_from_path(p) for p in ordered],
                             [0, 1, 2])
            picked = common.iter_frame_images(dd, [0, 2])
            self.assertEqual([common.frame_index_from_path(p) for p in picked],
                             [0, 2])


if __name__ == "__main__":
    unittest.main()
