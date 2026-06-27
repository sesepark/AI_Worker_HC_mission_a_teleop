#!/usr/bin/env python3
"""Unit tests for qwen_mission_c_ocr — HTTP 모킹, GPU/vLLM 불필요."""
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

import qwen_mission_c_ocr as qwen  # noqa: E402

DATA_URL = "data:image/png;base64,AAAA"

OK_MODEL_JSON = {
    "sequence": [
        {"peg": 1, "nut": "플랜지 너트", "size_mm": 19},
        {"peg": 2, "nut": "기어 링", "size_mm": 22},
        {"peg": 3, "nut": "스페이서 링", "size_mm": 17},
        {"peg": 4, "nut": "육각 너트", "size_mm": 24},
    ],
    "fail": False, "fail_reason": "", "is_dummy": False,
}


def fake_response(model_json: dict) -> dict:
    return {"choices": [{"message": {"content": json.dumps(model_json,
                                                           ensure_ascii=False)}}]}


class BuildPayloadTests(unittest.TestCase):
    def test_guided_json_default(self):
        p = qwen.build_payload(DATA_URL, "PROMPT", "Qwen/Qwen2.5-VL-7B-Instruct")
        self.assertIn("guided_json", p)
        self.assertNotIn("response_format", p)
        self.assertNotIn("reasoning_effort", p)  # Qwen 엔 무의미 → 빠져야 함
        self.assertEqual(p["temperature"], 0.0)  # 결정적 비교

    def test_response_format_mode(self):
        p = qwen.build_payload(DATA_URL, "PROMPT", "m", json_mode="response_format")
        self.assertIn("response_format", p)
        self.assertNotIn("guided_json", p)
        self.assertEqual(p["response_format"]["json_schema"]["name"],
                         "mission_c_sequence")

    def test_image_and_text_parts(self):
        p = qwen.build_payload(DATA_URL, "PROMPT", "m")
        content = p["messages"][0]["content"]
        kinds = {c["type"] for c in content}
        self.assertEqual(kinds, {"text", "image_url"})
        self.assertEqual(content[1]["image_url"]["url"], DATA_URL)


class RunImageTests(unittest.TestCase):
    def test_run_image_success(self):
        with patch.object(qwen, "image_to_data_url", return_value=DATA_URL), \
             patch.object(qwen, "call_chat_completions",
                          return_value=fake_response(OK_MODEL_JSON)):
            out = qwen.run_image(Path("frame_000005.png"))
        self.assertFalse(out["fail"], out["fail_reason"])
        self.assertEqual([e["peg"] for e in out["sequence"]], [1, 2, 3, 4])

    def test_run_image_normalizes_bad_schema(self):
        bad = json.loads(json.dumps(OK_MODEL_JSON))
        bad["sequence"][0]["nut"] = "엉터리"  # 스키마 위반 → 코드에서 fail 처리
        with patch.object(qwen, "image_to_data_url", return_value=DATA_URL), \
             patch.object(qwen, "call_chat_completions",
                          return_value=fake_response(bad)):
            out = qwen.run_image(Path("frame_000005.png"))
        self.assertTrue(out["fail"])
        self.assertIn("invalid nut", out["fail_reason"])

    def test_run_image_non_json_output(self):
        resp = {"choices": [{"message": {"content": "sorry I cannot"}}]}
        with patch.object(qwen, "image_to_data_url", return_value=DATA_URL), \
             patch.object(qwen, "call_chat_completions", return_value=resp):
            out = qwen.run_image(Path("frame_000005.png"))
        self.assertTrue(out["fail"])

    def test_run_image_passes_model_and_url(self):
        seen = {}

        def _capture(payload, base_url, **kw):
            seen["model"] = payload["model"]
            seen["base_url"] = base_url
            return fake_response(OK_MODEL_JSON)

        with patch.object(qwen, "image_to_data_url", return_value=DATA_URL), \
             patch.object(qwen, "call_chat_completions", side_effect=_capture):
            qwen.run_image(Path("frame_000005.png"),
                           model="Qwen/Qwen2.5-VL-3B-Instruct",
                           base_url="http://localhost:8000/v1")
        self.assertEqual(seen["model"], "Qwen/Qwen2.5-VL-3B-Instruct")
        self.assertEqual(seen["base_url"], "http://localhost:8000/v1")


if __name__ == "__main__":
    unittest.main()
