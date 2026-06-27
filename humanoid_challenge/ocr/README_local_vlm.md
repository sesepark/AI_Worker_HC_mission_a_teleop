# 로컬 VLM(Qwen2.5-VL) vs Gemini — Mission C 모니터 OCR 비교

Mission C 모니터(부품 순차 조립 지시, Peg 1..4 → 너트 1종)를 읽는 OCR을 **로컬
VLM(Qwen2.5-VL 3B/7B-Instruct)** 으로도 수행하고, 기존 **Gemini(LetsUr 게이트웨이)**
결과와 1:1 비교하기 위한 도구 모음이다.

두 백엔드는 **동일한 프롬프트([gemini_mission_c_prompt.md](gemini_mission_c_prompt.md))·
동일한 JSON 스키마(`SEQ_SCHEMA`)·동일한 정규화(`normalize_sequence_result`)** 를
공유한다(→ [mission_c_ocr_common.py](mission_c_ocr_common.py)). 따라서 모델 자체의
판독 성능만 차이로 남고 비교가 공정하다.

> 이 PC 에는 GPU/대용량 메모리가 없어 **로컬 VLM 추론은 실행하지 않는다.** 아래 코드·
> 스크립트·테스트·문서는 전부 완성되어 있으며, GPU 머신에서 vLLM 서버만 띄우면 그대로
> 동작한다. 게이트웨이(Gemini) 경로와 모든 단위테스트는 이 PC 에서 검증 완료.

## 구성 파일

| 파일 | 역할 |
|------|------|
| [mission_c_ocr_common.py](mission_c_ocr_common.py) | 공용: 스키마·이미지 인코딩·OpenAI 호환 호출·JSON 파싱·정규화·포맷 (무거운 의존성 없음) |
| [qwen_mission_c_ocr.py](qwen_mission_c_ocr.py) | 로컬 Qwen2.5-VL 백엔드 (vLLM OpenAI 호환 서버 호출, `guided_json`) |
| [gemini_mission_c_ocr.py](gemini_mission_c_ocr.py) | 기존 Gemini(게이트웨이) 백엔드 (변경 없음) |
| [compare_mission_c_ocr.py](compare_mission_c_ocr.py) | A/B 비교 하니스 (일치율·지연·실패율·정확도 표) |
| `test_*.py` | 단위테스트 37종 (HTTP 모킹, GPU/네트워크 불필요) |
| [requirements-qwen-vllm.txt](requirements-qwen-vllm.txt) | GPU 머신용 vLLM 의존성 |

## 왜 vLLM(OpenAI 호환)인가

vLLM 서버는 게이트웨이와 **동일한 `/v1/chat/completions` 인터페이스**를 노출하고,
`guided_json` 으로 우리 스키마에 맞춘 strict JSON 출력을 강제할 수 있다. 덕분에
클라이언트 코드(요청 페이로드/응답 파싱/정규화)가 게이트웨이 경로와 거의 동일해져
**비교의 변수를 모델 가중치만으로 좁힐 수 있다.**

## 1. GPU 머신에서 vLLM 서버 띄우기

```bash
python3 -m venv qwen_vllm_venv && source qwen_vllm_venv/bin/activate
pip install -U pip && pip install -r requirements-qwen-vllm.txt

# 3B (포트 8000)
vllm serve Qwen/Qwen2.5-VL-3B-Instruct \
    --port 8000 --max-model-len 8192 --limit-mm-per-prompt image=1

# 7B (포트 8001) — 별도 터미널/GPU
vllm serve Qwen/Qwen2.5-VL-7B-Instruct \
    --port 8001 --max-model-len 8192 --limit-mm-per-prompt image=1
```

대략적 VRAM 요구(bf16, KV 캐시 포함, 단일 이미지 입력 기준):
- **3B-Instruct**: 약 9–12 GB → 12 GB 카드에서도 가동 가능
- **7B-Instruct**: 약 18–22 GB → 24 GB 카드 권장

VRAM 이 빠듯하면 `--max-model-len` 을 줄이거나 vLLM 의 AWQ/GPTQ 양자화 모델
(`Qwen/Qwen2.5-VL-7B-Instruct-AWQ` 등)로 교체한다. `--gpu-memory-utilization 0.9`,
한 GPU 에 둘을 올릴 땐 `CUDA_VISIBLE_DEVICES` 로 분리한다.

> 서버가 떴는지 확인: `curl http://localhost:8000/v1/models`

## 2. 단일 백엔드로 한 디렉토리 판독

```bash
# 3B
python3 qwen_mission_c_ocr.py zed_rgb_100_20260626_151706 \
    --model Qwen/Qwen2.5-VL-3B-Instruct --base-url http://localhost:8000/v1 \
    --out qwen3b_results.json

# 특정 프레임만 (정렬된 목록의 0-based 인덱스)
python3 qwen_mission_c_ocr.py zed_rgb_100_20260626_151706 5 17 42 \
    --model Qwen/Qwen2.5-VL-7B-Instruct --base-url http://localhost:8001/v1
```

`--json-mode` 로 스키마 강제 방식 선택: `guided_json`(기본, vLLM 고유·가장 안정) /
`response_format`(OpenAI 표준, 신형 vLLM). 어느 쪽이든 결과는 코드에서 한 번 더
검증되므로 출력 형식은 같다.

## 3. Gemini vs Qwen A/B 비교

```bash
python3 compare_mission_c_ocr.py zed_rgb_100_20260626_151706 \
    --qwen '3B=http://localhost:8000/v1|Qwen/Qwen2.5-VL-3B-Instruct' \
    --qwen '7B=http://localhost:8001/v1|Qwen/Qwen2.5-VL-7B-Instruct' \
    --out compare.json
```

- `--qwen` 미지정 시 위 3B/7B 두 백엔드를 기본 사용.
- `--no-gemini` 로 게이트웨이 백엔드 제외(로컬 모델끼리만 비교).
- 프레임마다 각 백엔드의 결과·지연(초)·(정답 제공 시) `[n/4]` 정확도를 출력하고,
  마지막에 요약(실패율·평균지연·정확도·게이트웨이 대비 일치율)을 표로 보여준다.

### 정답(ground truth)으로 정확도 측정

```bash
python3 compare_mission_c_ocr.py zed_rgb_100_20260626_151706 --gt gt.json
```

`--gt` 는 두 형식 모두 허용:

```jsonc
// 형식 A: 프레임 인덱스 → {peg: nut}
{ "5": {"1":"플랜지 너트","2":"기어 링","3":"스페이서 링","4":"육각 너트"} }
```

```jsonc
// 형식 B: 스크립트 --out 결과(list of {frame,result}) — 신뢰 백엔드로 GT 부트스트랩
// 예: gemini_mission_c_ocr.py ... --out gemini.json  →  --gt gemini.json
```

정답이 없으면 **백엔드 간 일치율**(`agreement_vs_gemini`)과 실패율·지연만 계산된다.
정답을 빠르게 만들려면 신뢰하는 백엔드(보통 Gemini) 출력에서 명백히 맞는 프레임을
형식 A 로 골라 적으면 된다.

## 4. 출력 지표 의미

| 지표 | 뜻 |
|------|----|
| `n_ok` / `fail_rate` | 정상 판독 프레임 수 / 실패 비율 |
| `mean_latency_sec` | 프레임당 평균 응답 시간(클라우드 왕복 vs 로컬 추론 비교의 핵심) |
| `exact_match_rate` | (GT 대비) 4개 peg 모두 정확한 프레임 비율 |
| `per_peg_accuracy` | (GT 대비) peg 단위 정확도 = 맞은 peg 수 / 전체 peg 수 |
| `agreement_vs_gemini` | Gemini 와 peg→nut 매핑이 완전히 같은 비율 |

## 5. 테스트 (이 PC 에서 검증 완료)

```bash
cd humanoid_challenge/ocr
python3 -m unittest test_mission_c_ocr_common test_qwen_mission_c_ocr test_compare_mission_c_ocr -v
# Ran 37 tests ... OK
```

GPU/네트워크/무거운 의존성 없이 HTTP 를 모킹해 페이로드 구성·스키마 검증·JSON
파싱 폴백·지표 계산을 모두 검증한다.

## 알아둘 점 / 차이

- **스키마 강제**: 게이트웨이(Gemini)는 `response_format=json_schema`, vLLM 은
  `guided_json`. 둘 다 동일한 `SEQ_SCHEMA` 를 쓰고, 클라이언트가 다시
  `normalize_sequence_result` 로 검증한다. (가드가 꺼져도 잘못된 출력은 fail 처리.)
- **결정성**: Qwen 호출은 `temperature=0`. Gemini 는 게이트웨이 기본값.
- **이미지**: PPM/BMP(로컬 ZED 프레임)는 opencv 로 PNG 변환 후 base64 data URL 로
  전송. PNG/JPEG/WEBP/GIF 는 그대로.
- **인증**: vLLM 은 기본적으로 인증 없음(`--api-key` 로 띄웠을 때만 `QWEN_API_KEY`).
  게이트웨이 키는 환경변수(`LETSUR_API_KEY`) 사용 권장.
- **`reasoning_effort`**: Gemini 전용 토큰 절감 옵션이라 Qwen 페이로드에선 제거.
