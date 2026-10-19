# GUI 연동용 림 선명도 계산

확정한 **방사형 ESF v2 + Lorentzian 폭 측정** 버전입니다. 이전 전달본처럼 이미 선택한 bbox crop 한 장을 받아 결과 dict와 선택적 오버레이를 반환합니다. `analyze_representative_crop` 호출 방식과 기존 결과 키를 유지하며, 모델폭과 품질 정보를 추가했습니다.

## 설치·실행

압축을 푼 폴더에서 실행합니다. 검증 환경: Windows, Python 3.12.14, NumPy 1.26.4, OpenCV 4.11.0.86. 학습 가중치·GPU·SciPy·ROS는 필요하지 않습니다.

```bash
python -m pip install -r requirements.txt
python verify_package.py
python analyze_rim.py --image examples/frame_005023_crop.png --json result.json --overlay overlay.png --overlay-scale 4
```

이미 같은 버전의 `opencv-python`이 있다면 `opencv-python-headless`를 중복 설치하지 않아도 됩니다. GUI 프로젝트에 복사할 때는 `analyze_rim.py`와 `rim_sharpness_core/`를 함께 옮기세요.

## GUI 호출

```python
from analyze_rim import analyze_representative_crop

result, overlay_bgr = analyze_representative_crop(
    crop,                       # uint8, 이미 선택한 대표 bbox crop
    color_order="gray",         # mono8: gray / OpenCV 컬러: bgr / RGB: rgb
    include_overlay=True,
)
pin_info["rim"] = result
```

입력은 2차원 mono8 또는 3채널 uint8 영상입니다. 반환 오버레이는 BGR이며 입력 영상은 수정하지 않습니다. 원본 bbox·timestamp·pin ID와 대표 관측 선택은 호출 측에서 관리합니다. `include_diagnostics=True`를 추가하면 상세 진단 정보를 반환합니다.

**원형 점 표시 수정:** 작은 crop에 그린 점을 나중에 확대하면 별모양처럼 보일 수 있어, 표시할 크기로 배경을 먼저 조정하고 부드러운 원형 점을 그립니다. 기본 반환 크기는 기존처럼 입력과 같습니다. GUI에서 확대 표시할 때는 `overlay_size=(표시_너비, 표시_높이)`를 호출에 추가하세요. 창 크기만 바뀌면 계산을 다시 하지 않고 아래처럼 오버레이만 갱신할 수 있습니다. 포함된 예제 오버레이는 원본의 4배 크기로 그렸습니다.

```python
from analyze_rim import render_rim_overlay
overlay_bgr = render_rim_overlay(
    crop, result, color_order="gray", output_size=(display_width, display_height)
)
```

## 결과 해석

| 키 | 의미 |
|---|---|
| `rim_sharpness_raw` | 주 선명도 점수. 관측폭 기반 ESF v2 점수 유지 |
| `median_edge_width_r` | 채택 구간의 관측 FWHM 중앙값 |
| `lorentzian_median_width_r` | 적합을 통과한 구간의 Lorentzian 모델폭 중앙값 |
| `accepted_sectors` / `total_sectors` | 최종 채택 구간 수 / 전체 32구간 |
| `lorentzian_fitted_sectors` | 모델폭까지 확보한 채택 구간 수 |
| `observed_only_sectors` | 관측폭만 사용하는 채택 구간 수 |
| `rejected_selected_sectors` | 선택 후보가 있으나 최종 제외된 구간 수 |
| `no_selected_candidate_sectors` | 표시할 선택 후보가 없는 구간 수 |
| `status`, `reason` | 전체 판정과 사유 |
| `width_status`, `width_reason` | 관측폭 확보 여부와 사유 |
| `model_width_status`, `model_width_reason` | 모델폭 확보 여부와 사유 |
| `sector_measurements` | 구간별 위치·관측폭·모델폭·적합 품질 |

오버레이의 **파랑은 모델폭 확보**, **노랑은 관측폭만 채택**, **빨강은 최종 제외된 후보**입니다. 후보가 없는 구간은 점을 만들지 않습니다. 빨강은 정답 대비 오탐 판정이 아닙니다.

`pending`인 결과는 확정값으로 사용하지 마세요. 확보하지 못한 폭은 `null`이며 0으로 대체하지 않습니다. 폭 단위는 정규화한 bbox 반경(`normalized_bbox_radius`)으로, mm나 원본 픽셀폭이 아닙니다. 점수는 확률·지질학적 나이·물리적 깊이가 아니며, 촬영 조건이 유사한 영상끼리 비교해야 합니다. 실제 림 위치의 정답 검증은 별도입니다.

이번 버전에서 주 점수가 동일하게 유지되는 기준은 검증에 사용한 **ESF v2**입니다. 이전 배포 ZIP의 v1과 수치가 같다는 뜻은 아닙니다. 알고리즘 버전은 `radial_esf_hybrid_v2_dual_20261007`입니다.

## 방법·시간

기존 방사형 ESF의 후보 선택과 연결 구간을 유지하고, 채택한 경계의 미분 프로파일에 Lorentzian을 적합합니다. 근거는 [Tan 등(2024), §2.2.4, 식 (6)](https://www.mdpi.com/2306-5354/11/6/614)입니다. 논문은 MRI 팬텀의 선명도 측정 연구이며, 크레이터 후보 선택·연결·품질 조건은 우리 구현입니다. 단일 논문의 전체 재현으로 설명하지 않습니다. 관측폭 기반 점수와 모델폭은 구분해서 반환합니다.

기존 실측: Core Ultra 9 185H, OpenCV 1스레드, 사전 로드한 mono8 영상 184개 × 5회에서 계산 함수 평균 **31.28 ms**, 중앙값 **30.25 ms**, P95 **52.90 ms**. 파일 읽기·CDA·ROS·GUI 어댑터·오버레이 등은 제외한 값입니다.

## 구성

- `analyze_rim.py`: GUI 호출 함수와 단일 이미지 CLI
- `rim_sharpness_core/`: ESF v2와 Lorentzian 결합 계산 코드
- `requirements.txt`: 실행 의존성
- `examples/`: 실제 crop 4개, 결과 JSON, 빨간 점을 포함한 오버레이, 기대값
- `verify_package.py`, `package_manifest.json`: 독립 실행·예제 회귀·파일 무결성 확인

`verify_package.py`에서 `status: passed`가 출력되면 포함된 예제와 기본 입력 계약 확인을 통과한 것입니다. 이는 실제 림 정답에 대한 정확도 검증을 의미하지 않습니다.
