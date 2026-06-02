"""PaddleOCR 3.x helpers."""
from paddleocr import PaddleOCR as _PaddleOCR


def make_ocr(lang: str, det_thresh=0.3, det_box_thresh=0.5, det_unclip=1.5) -> _PaddleOCR:
    """Create a PaddleOCR 3.x engine."""
    return _PaddleOCR(
        lang=lang,
        use_textline_orientation=True,
        text_det_thresh=det_thresh,
        text_det_box_thresh=det_box_thresh,
        text_det_unclip_ratio=det_unclip,
    )


def ocr_run(engine: _PaddleOCR, img) -> list:
    """
    Run OCR and normalize PaddleOCR 3.x output.

    반환: [ [box_pts, (text, conf)], ... ]
      box_pts: [[x0,y0],[x1,y1],[x2,y2],[x3,y3]]
    """
    try:
        result = engine.predict(img)
        if not result:
            return []
        r = result[0]
        boxes  = r.get('dt_polys',  [])
        texts  = r.get('rec_texts', [])
        scores = r.get('rec_scores', [])
        return [
            [box.tolist() if hasattr(box, 'tolist') else box, (text, float(score))]
            for box, text, score in zip(boxes, texts, scores)
        ]
    except Exception:
        return []
