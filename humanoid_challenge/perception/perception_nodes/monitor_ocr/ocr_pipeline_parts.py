"""
부품 수량 테이블 OCR 파이프라인

모니터 형식 (5행 테이블, 흰 배경):
  ┌─────────────────────────────┐
  │ [아이콘] │ 부품명   │ 수량 │
  │    ...   │  ...    │  ... │
  └─────────────────────────────┘

테이블 행 순서는 고정된 PART_NAMES 순서를 사용한다.

열 구분: Hough 수직선 자동 감지 → 실패 시 기본값 폴백
OCR:    수량=영어 det=True + 0~5 클램핑
"""
import cv2
import numpy as np
import os
import re
import time

from perception_nodes.monitor_ocr.paddle_ocr import ocr_run


# ── 부품 이름 ──────────────────────────────────────────────────────────────────
PART_NAMES = ["플랜지 너트", "기어 링", "스페이서 링", "육각 너트", "돔 너트"]
N_ROWS = len(PART_NAMES)

# ── 열 x 비율 폴백값 (bbox 기준) ──────────────────────────────────────────────
_NAME_X  = (0.19, 0.76)
_COUNT_X = (0.76, 0.99)

# ── 행 y 패딩 ───────────────────────────────────────────────────────────────────
_ROW_PAD        = 0.018  # 이름 크롭: 위아래 패딩 (행 경계선 제외)
_COUNT_BOT_EXT  = 0.12   # 수량 크롭: 하단 확장 (카메라 각도로 숫자가 셀 하단~다음 행 초입에 위치)

# ── OCR confidence 임계값 ──────────────────────────────────────────────────────
_COUNT_CONF_THRESH = 0.2   # 수량 숫자 최소 confidence

# ── 유효 수량 범위 ─────────────────────────────────────────────────────────────
_VALID_COUNTS = list(range(6))  # 0~5

# ── 모니터 감지 ───────────────────────────────────────────────────────────────
# YOLO warp 출력 크기. 부품 수량 테이블 감지에만 사용한다.
_FALLBACK_BBOX = (58, 30, 406, 216)
_WARP_W = _FALLBACK_BBOX[2]
_WARP_H = int(_FALLBACK_BBOX[3] * 1.4)
_yolo_model = None


def default_yolo_model_path() -> str:
    """설치된 perception 패키지 기준 기본 monitor OCR YOLO 모델 경로."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('perception'),
            'model',
            'monitor_ocr_best.pt',
        )
    except Exception:
        return os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                '..',
                '..',
                'model',
                'monitor_ocr_best.pt',
            )
        )


def load_yolo_model(model_path: str | None = None) -> str | None:
    """
    YOLO 세그멘테이션 모델을 로드한다.

    Returns
    -------
    str | None
        로드된 모델 경로. 파일이 없으면 None.
    """
    path = model_path or default_yolo_model_path()
    if not path or not os.path.isfile(path):
        return None

    global _yolo_model
    from ultralytics import YOLO
    _yolo_model = YOLO(path)
    return path


def _sort_quad_corners(pts):
    """4점을 [TL, TR, BR, BL] 순서로 정렬."""
    pts = pts.reshape(4, 2)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).ravel()
    return np.array(
        [pts[s.argmin()], pts[d.argmin()], pts[s.argmax()], pts[d.argmax()]],
        dtype=np.float32,
    )


def _mask_to_quad(mask_xy):
    """세그멘테이션 폴리곤을 perspective warp용 사각형으로 변환."""
    pts = np.array(mask_xy, dtype=np.int32)
    if len(pts) < 4:
        return None
    hull = cv2.convexHull(pts)
    rect = cv2.minAreaRect(hull)
    box = cv2.boxPoints(rect).astype(np.float32)
    return _sort_quad_corners(box)


def find_display_yolo(img, conf_thresh=0.50):
    """
    YOLO-seg로 모니터를 감지해 정면화한다.

    Returns
    -------
    tuple | None
        (warped_img, bx, by, bw, bh) 또는 None.
    """
    if _yolo_model is None:
        return None

    results = _yolo_model(img, verbose=False)[0]
    masks = results.masks
    boxes = results.boxes
    if masks is None or len(masks.xy) == 0:
        return None

    best = int(boxes.conf.argmax())
    if float(boxes.conf[best]) < conf_thresh:
        return None

    x1, y1, x2, y2 = boxes.xyxy[best].cpu().numpy().astype(int)
    det_w, det_h = x2 - x1, y2 - y1
    if det_h == 0 or det_w / det_h > 6.0:
        return None

    corners = _mask_to_quad(masks.xy[best])
    if corners is None:
        corners = np.array(
            [[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32
        )

    dst = np.array(
        [[0, 0], [_WARP_W - 1, 0], [_WARP_W - 1, _WARP_H - 1], [0, _WARP_H - 1]],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(corners, dst)
    warped = cv2.warpPerspective(img, matrix, (_WARP_W, _WARP_H))
    return warped, 0, 0, _WARP_W, _WARP_H


def find_display_hsv(img):
    """HSV 어두운 모니터 영역으로 bbox를 찾는다. 실패 시 None."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h_img = img.shape[0]
    dark = cv2.inRange(hsv, (0, 0, 0), (180, 255, 70))
    dark[h_img // 2:, :] = 0
    k = np.ones((10, 10), np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, k)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, k)
    cnts, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    x, y, w, h = cv2.boundingRect(max(cnts, key=cv2.contourArea))
    y_end = min(y + int(h * 1.35), h_img)
    return (x, y, w, y_end - y)


# ── 화면 감지 ─────────────────────────────────────────────────────────────────

def _has_column_separators(img: np.ndarray, bbox) -> bool:
    """bbox 영역에 수직 구분선(열 경계)이 존재하는지 확인."""
    bx, by, bw, bh = bbox
    crop = img[by:by+bh, bx:bx+bw]
    name_x, _ = _detect_column_ratios(crop)
    return name_x != _NAME_X  # 폴백값이 아니면 구분선 감지 성공


def _trim_bbox_bottom(img: np.ndarray, bbox, bright_thresh: int = 180) -> tuple:
    """bbox 하단을 밝기 프로파일로 트리밍 (베젤·어두운 영역 제거)."""
    bx, by, bw, bh = bbox
    gray = cv2.cvtColor(img[by:by+bh, bx:bx+bw], cv2.COLOR_BGR2GRAY)
    x1, x2 = bw // 6, bw * 5 // 6
    for y in range(bh - 1, bh // 2, -1):
        if np.mean(gray[y, x1:x2]) > bright_thresh:
            return (bx, by, bw, y + 1)
    return bbox


def _bbox_center_inside(inner, outer) -> bool:
    """inner bbox 중심점이 outer bbox 안에 있는지 확인."""
    if outer is None:
        return True
    x, y, w, h = inner
    ox, oy, ow, oh = outer
    cx = x + w / 2
    cy = y + h / 2
    return ox <= cx <= ox + ow and oy <= cy <= oy + oh


def find_display_parts(img: np.ndarray):
    """
    흰색 테이블 영역 감지.
    열 구분선이 있는 후보 우선 → 없으면 가장 큰 후보 → find_display() 폴백.
    감지된 bbox는 하단 베젤·어두운 영역을 제거하여 트리밍.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h_img, w_img = img.shape[:2]

    bright = cv2.inRange(gray, 190, 255)
    k = np.ones((10, 10), np.uint8)
    bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, k)
    bright = cv2.morphologyEx(bright, cv2.MORPH_OPEN,  k)

    cnts, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    monitor_bbox = find_display_hsv(img)
    candidates = []
    for cnt in cnts:
        x, y, w, h = cv2.boundingRect(cnt)
        bbox = (x, y, w, h)
        if (
            w > w_img * 0.20
            and h > h_img * 0.15
            and w > h * 1.3
            and _bbox_center_inside(bbox, monitor_bbox)
        ):
            candidates.append((cv2.contourArea(cnt), (x, y, w, h)))

    if not candidates:
        return monitor_bbox

    # 열 구분선이 감지되는 후보 중 가장 큰 것 우선
    candidates.sort(key=lambda c: c[0], reverse=True)
    for _, bbox in candidates:
        if _has_column_separators(img, bbox):
            return _trim_bbox_bottom(img, bbox)

    # 구분선 없으면 가장 큰 후보
    return _trim_bbox_bottom(img, candidates[0][1])


# ── 수평/수직 구분선 자동 감지 ───────────────────────────────────────────────

def _hough_separators(table_img: np.ndarray, vertical: bool) -> list:
    """
    Hough 선 검출로 수직(vertical=True) 또는 수평(False) 구분선의 비율 목록 반환.
    클러스터링 후 대표값 반환. 감지 실패 시 빈 리스트.
    """
    gray = cv2.cvtColor(table_img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    size = w if vertical else h
    cross = h if vertical else w

    edges = cv2.Canny(gray, 20, 80, apertureSize=3)
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180,
        threshold=cross // 4,
        minLineLength=cross // 3,
        maxLineGap=30,
    )
    if lines is None:
        return []

    coords = []
    for x1, y1, x2, y2 in lines[:, 0]:
        angle = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1)))
        if vertical and angle > 75:
            coords.append((x1 + x2) / 2 / size)
        elif not vertical and angle < 15:
            coords.append((y1 + y2) / 2 / size)

    if not coords:
        return []

    coords.sort()
    clusters, group = [], [coords[0]]
    for c in coords[1:]:
        if c - group[-1] < 20 / size:
            group.append(c)
        else:
            clusters.append(round(float(np.mean(group)), 3))
            group = [c]
    clusters.append(round(float(np.mean(group)), 3))
    return clusters


def _detect_column_ratios(table_img: np.ndarray):
    """수직선으로 열 경계 감지. 실패 시 기본값 반환."""
    if table_img.shape[0] < 20 or table_img.shape[1] < 20:
        return _NAME_X, _COUNT_X

    clusters = _hough_separators(table_img, vertical=True)
    icon_seps  = [s for s in clusters if 0.05 < s < 0.35]
    count_seps = [s for s in clusters if 0.50 < s < 0.95]

    if not count_seps:
        return _NAME_X, _COUNT_X

    sep2 = count_seps[0]
    # icon_seps 없으면 기본값 사용 (아이콘 열 폭 고정)
    sep1 = icon_seps[0] if icon_seps else _NAME_X[0]
    if sep1 >= sep2:
        return _NAME_X, _COUNT_X

    return (sep1, sep2), (sep2, 0.99)


def _detect_row_ys(table_img: np.ndarray) -> list:
    """
    수평선으로 행 경계 y 비율 목록 감지 (N_ROWS+1개).

    1순위: 내부 구분선 N_ROWS-1개 정확히 감지 → 그대로 사용
    2순위: 상단 경계 + 내부 구분선 간격으로 행 높이 추정 → 외삽
    3순위: 상단 경계 + 하단 경계 감지 → 그 구간 등분
    4순위: 폴백 → 전체 등분
    """
    default = [i / N_ROWS for i in range(N_ROWS + 1)]
    if table_img.shape[0] < 20 or table_img.shape[1] < 20:
        return default

    clusters = _hough_separators(table_img, vertical=False)
    if not clusters:
        return default

    # 1순위: 내부 구분선 정확히 N_ROWS-1개 + head_space ≤ avg_gap*1.5 (상단 경계와 혼동 방지)
    inner = sorted([s for s in clusters if 0.10 < s < 0.90])
    if len(inner) == N_ROWS - 1:
        avg_gap = (inner[-1] - inner[0]) / max(1, len(inner) - 1)
        if avg_gap > 0 and inner[0] <= avg_gap * 1.5:
            return [0.0] + inner + [1.0]

    # 2순위: 균일 행 높이 추정 → 최적 시작점 탐색 후 외삽
    cs = sorted(clusters)
    valid_gaps = [cs[i + 1] - cs[i] for i in range(len(cs) - 1)
                  if 0.08 < cs[i + 1] - cs[i] < 0.25]
    if valid_gaps:
        row_h = float(np.median(valid_gaps))
        tol = row_h * 0.25
        best_start, best_count = None, 0
        for start in cs:
            count, pos = 0, start
            for _ in range(N_ROWS + 1):
                if min(abs(c - pos) for c in cs) <= tol:
                    count += 1
                    pos += row_h
                else:
                    break
            if count > best_count:
                best_count, best_start = count, start
        if best_start is not None and best_count >= 2:
            if best_count >= 3:
                # 3개 이상 연속 → best_start가 테이블 상단 (테이블 위 여백 있어도 무방)
                ys = [best_start + row_h * i for i in range(N_ROWS + 1)]
            else:
                # 2개만 감지 → 역방향 외삽으로 테이블 상단 추정
                n_above = max(0, min(N_ROWS - 2, int(best_start / row_h)))
                table_top = best_start - n_above * row_h
                ys = [table_top + row_h * i for i in range(N_ROWS + 1)]
            if ys[-1] <= 1.05:
                return [max(0.0, min(1.0, y)) for y in ys]

    # 3순위: 상단 + 하단 Hough 경계 구간 등분
    top_candidates = [s for s in clusters if 0.05 < s < 0.35]
    if top_candidates:
        table_top = top_candidates[0]
        bot_candidates = [s for s in clusters if 0.70 < s < 0.98]
        table_bot = bot_candidates[-1] if bot_candidates else 1.0
        span = table_bot - table_top
        if span > 0.3:
            return [table_top + span * i / N_ROWS for i in range(N_ROWS + 1)]

    return default


# ── 전처리 ────────────────────────────────────────────────────────────────────

def _preprocess(img: np.ndarray, scale: int) -> np.ndarray:
    img  = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    blur = cv2.GaussianBlur(img, (0, 0), 1.0)
    return cv2.addWeighted(img, 1.5, blur, -0.5, 0)


# ── 파싱 헬퍼 ─────────────────────────────────────────────────────────────────

def _extract_count(text: str) -> int:
    """
    OCR 텍스트 → 0~5 정수. 숫자를 찾지 못하면 -1.
    영어 OCR의 흔한 오인식(O→0, I→1, S→5)을 보정 후 추출.
    """
    t = (text.strip()
         .replace('O', '0').replace('o', '0')
         .replace('I', '1').replace('l', '1')
         .replace('S', '5').replace('s', '5'))
    m = re.search(r'\d+', t)
    if not m:
        return -1
    return min(_VALID_COUNTS, key=lambda x: abs(x - int(m.group())))


# ── 메인 처리 ─────────────────────────────────────────────────────────────────

def process_frame_parts(ocr_en, img: np.ndarray) -> dict:
    """
    부품 수량 테이블 OCR.

    Returns
    -------
    dict
        screen_detected : bool
        bbox            : [x, y, w, h] or None
        col_ratios      : {"name_x": [...], "count_x": [...]}
        parts           : [{"name": str, "count": int}, ...]  PART_NAMES 순서, count=-1=미인식
        elapsed_ms      : float
    """
    t0 = time.time()

    # YOLO로 모니터 감지 + 정면화 → 실패 시 원본 이미지에서 테이블 영역 감지
    yolo = find_display_yolo(img)
    work_img = yolo[0] if yolo is not None else img
    H, W = work_img.shape[:2]

    if yolo is not None:
        bbox = yolo[1:]
    else:
        bbox = find_display_parts(work_img)

    if not bbox:
        return {
            "screen_detected": False,
            "bbox": None,
            "col_ratios": None,
            "parts": [{"name": n, "count": -1} for n in PART_NAMES],
            "elapsed_ms": round((time.time() - t0) * 1000, 1),
        }

    bx, by, bw, bh = bbox
    # 상단 1행 여백 추가 (커브드 모니터 warp 시 첫 행 잘림 방지)
    row_ext = bh // N_ROWS
    by  = max(0, by - row_ext)
    bh  = min(H - by, bh + row_ext)
    table_crop      = work_img[by:by+bh, bx:bx+bw]
    name_x, count_x = _detect_column_ratios(table_crop)

    # ── 수량 열 전체 OCR (det=True → 박스 y좌표 확보) ────────────────────────
    cx1 = max(0, int(bx + count_x[0] * bw))
    cx2 = min(W, int(bx + count_x[1] * bw))
    # 마지막 행 숫자가 bbox 하단에 걸릴 수 있으므로 아래로 확장
    cy2 = min(H, by + bh + int(bh * _COUNT_BOT_EXT))
    count_col = work_img[by:cy2, cx1:cx2]

    best_partial: list[tuple[float, int, float]] = []
    counts = [-1] * N_ROWS
    for scale in (4, 2, 6):
        proc = _preprocess(count_col, scale)
        candidates: list[tuple[float, int, float]] = []
        for box, (text, conf) in ocr_run(ocr_en, proc):
            if conf < _COUNT_CONF_THRESH:
                continue
            v = _extract_count(text)
            if v < 0:
                continue
            y_center = sum(pt[1] for pt in box) / 4 / scale / bh
            candidates.append((y_center, v, float(conf)))

        candidates.sort(key=lambda item: item[0])
        deduped: list[tuple[float, int, float]] = []
        for y_center, value, conf in candidates:
            if deduped and abs(y_center - deduped[-1][0]) < 0.08:
                if conf > deduped[-1][2]:
                    deduped[-1] = (y_center, value, conf)
            else:
                deduped.append((y_center, value, conf))

        if len(deduped) > len(best_partial):
            best_partial = deduped
        if len(deduped) >= N_ROWS:
            counts = [value for _, value, _ in deduped[:N_ROWS]]
            break

    if all(count < 0 for count in counts) and best_partial:
        row_best: list[tuple[float, int]] = [(-1.0, -1) for _ in range(N_ROWS)]
        for y_center, value, conf in best_partial:
            row = int(round(y_center * N_ROWS - 0.5))
            row = min(max(row, 0), N_ROWS - 1)
            if conf > row_best[row][0]:
                row_best[row] = (conf, value)
        counts = [value for _, value in row_best]

    return {
        "screen_detected": True,
        "bbox": [bx, by, bw, bh],
        "col_ratios": {
            "name_x": list(name_x),
            "count_x": list(count_x),
        },
        "parts": [
            {"name": name, "count": count}
            for name, count in zip(PART_NAMES, counts)
        ],
        "elapsed_ms": round((time.time() - t0) * 1000, 1),
    }
