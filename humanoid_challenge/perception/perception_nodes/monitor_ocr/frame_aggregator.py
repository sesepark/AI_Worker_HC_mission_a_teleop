"""부품 수량 테이블 OCR 결과를 여러 프레임에 걸쳐 안정화."""
from collections import Counter


class FrameAggregatorParts:
    """
    부품 수량 테이블 OCR 결과 집계 (슬라이딩 윈도우 다수결).

    수량은 프레임 간 변하지 않으므로 다수결로 안정화.
    이름은 행 순서 기반 PART_NAMES 고정값 사용.
    """

    def __init__(self, window: int = 10):
        self.window   = window
        self._history = []

    def update(self, result: dict) -> dict:
        # 화면은 감지됐더라도 OCR로 읽힌 수량이 하나도 없으면 미감지로 처리
        if result.get("screen_detected") and result.get("parts"):
            if all(p["count"] < 0 for p in result["parts"]):
                result = dict(result, screen_detected=False)

        self._history.append(result)
        if len(self._history) > self.window:
            self._history.pop(0)
        return self._aggregate()

    def _aggregate(self) -> dict:
        from perception_nodes.monitor_ocr.ocr_pipeline_parts import PART_NAMES, N_ROWS

        hist   = self._history
        latest = hist[-1]

        # 최신 프레임에서 화면 미감지 → 즉시 -1 반환 (이전 값 유지 안 함)
        if not latest.get("screen_detected", False):
            return {
                "frames_used":            len(hist),
                "parts":                  [{"name": PART_NAMES[i], "count": -1} for i in range(N_ROWS)],
                "latest_elapsed_ms":      latest.get("elapsed_ms"),
                "latest_screen_detected": False,
            }

        # 화면 감지된 경우: 유효값의 다수결
        parts = []
        for i in range(N_ROWS):
            counts = [
                h["parts"][i]["count"]
                for h in hist
                if h.get("screen_detected") and h.get("parts")
                and len(h["parts"]) > i
                and h["parts"][i]["count"] >= 0
            ]
            count = Counter(counts).most_common(1)[0][0] if counts else -1
            parts.append({"name": PART_NAMES[i], "count": count})

        return {
            "frames_used":            len(hist),
            "parts":                  parts,
            "latest_elapsed_ms":      latest.get("elapsed_ms"),
            "latest_screen_detected": True,
        }

    def reset(self):
        self._history.clear()
