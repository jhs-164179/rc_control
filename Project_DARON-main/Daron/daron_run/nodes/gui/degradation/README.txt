크레이터 상대비교 — 경사 0.5 / 영상 림 선명도 0.5
버전: equalweight-20261013-v1

1. 적용식
  D = 50 × (1 - uθ) + 50 × (1 - uS)
  uθ = clip((θ - θ_low)/(θ_high - θ_low), 0, 1)
  uS = clip((S - S_low)/(S_high - S_low), 0, 1)
  θ: 원시 Depth로 측정한 관측 내벽 경사 P95, 단위 도(°)
  S: 기존 림 분석기의 rim_sharpness_raw. 0~1로 제한된 값이 아님.
  D: 작을수록 해당 비교군 안에서 상대적으로 보존도가 높음.
  두 항이 각각 0~50을 기여함. 한 항이 없으면 D를 산출하지 않음.

  1:1은 이번 운영 기준임. 원문에서 현재 P95와 영상 S의 최적 계수를
  검증한 것으로 표현하지 않음. MorphoPy의 0.34/0.66은 다른 지표의
  결합이므로 이 실행 경로에는 사용하지 않음.

2. 폴더를 그대로 옮겨 사용
  crater_rank/: 표준 라이브러리만 사용하는 집계·정규화·비교·기록 코드
  run_equalweight.py: 가중치 1:1로 고정한 배치 실행 진입점
  integration/record_adapter.py: 기존 측정 결과 → 관측 기록 변환
  integration/observed_features.py: 기존 crater_analysis용 원시 Depth 측정 어댑터
  policy.json: 관측 선별·시간 집계 기준
  tests/: 단위 검증 코드. 시험용 입력은 실제 영상 실험 결과가 아님.
  docs/CLAUDE_INTEGRATION.txt: 현장 프로그램 연결 지침

  비교 코어에는 NumPy, Numba, SciPy, ROS 의존성이 없음.
  기존 NumPy 1.21.5를 변경하거나 패키지를 설치할 필요가 없음.
  원시 Depth 측정 어댑터는 기존 crater_analysis와 NumPy를 사용함.
  이번 PC의 어댑터 실험은 기존 NumPy 1.26.4 환경에서 수행했으며,
  대상 PC의 NumPy 1.21.5 런타임까지 실행 검증한 것은 아님.

3. 입력을 기록하는 순서
  기존 CDA 탐지 → 동기화된 IR/Depth 한 쌍 고정 → 경사·림 측정 →
  make_observation → ObservationStore.add → evaluate로 전체 기록 갱신.
  동일 프레임의 재전송은 중복 제거됨. 같은 ID에 다른 내용이 들어오면 오류.
  프레임 ID만으로 크레이터를 구분하지 말고 tracker/SLAM의 지속 ID를 사용.
  같은 대상의 재방문 여부가 미확정이면 identity_confirmed=False를 유지.
  acquisition_verified는 자세·조명·동일 내벽 구간의 비교 가능성이 확인될
  때만 True. 카메라 정보가 없다는 이유로 임의 True를 넣으면 안 됨.

4. 고정 정규화 기준 만들기
  python -m crater_rank audit --input observations.jsonl --out audit
  python -m crater_rank calibrate --input observations.jsonl --out calibration --group-index 0 --reference-id mission-v1

  audit.json에서 동일 protocol/comparison_group/view_cell/support_id인지
  먼저 확인. 조건을 만족하는 4개 이상의 크레이터 대표값으로 초기 범위를
  산출함. reference.json을 한 번 저장한 뒤 뒤따르는 관측에도 유지함.
  파일이 이미 있으면 자동 덮어쓰지 않음. 다른 비교 조건은 별도 reference.
  충분한 초기 자료가 없으면 현장 기준을 아직 만들 수 없음.
  실험 폴더의 reference는 실험 재현용이며 현장에 자동 복사하지 않음.

5. 고정 가중치 실행
  python run_equalweight.py --input observations.jsonl --reference calibration/reference.json --out result

  ranking.json / qualified.csv / pending.csv 출력.
  ranked: 관측·식별·비교 조건과 반복 관측 기준을 만족한 결과.
  preview: 수치·신호 조건은 통과했으나 식별·촬영 조건·관측량 확인 등이
           남은 대상. provisional_index는 계산하지만 확정 순위에 포함하지 않음.
  audit: 결측·신호 부족·중복 대상별 집계. 기록을 조용히 삭제하지 않음.
  unscored_protocols: 주어진 reference와 조건이 달라 계산하지 않은 그룹.

  프로그램 내부에서는 다음 공개 API 사용:
    crater_rank.equalweight.evaluate(observations, reference, policy)
    crater_rank.equalweight.index(slope_deg, rim_score, reference)
  index는 순수 수식 함수이며 관측 품질을 검사하지 않음.
  현장 순위에는 반드시 evaluate를 사용.
  기존 crater_rank.core.rank의 임의 가중치 API는 연구용으로 남겼으며
  현장 진입점 run_equalweight.py는 항상 1:1임.

6. 반복 관측과 측정 기준
  같은 대상·같은 조건에서 5초 구간당 한 쌍을 선정함.
  깊이 유효 비율, 원래 상자 크기, 포화·동기화 상태로 선정하며
  높은 경사·높은 림 점수를 고르는 방식은 사용하지 않음.
  선정된 구간들의 중앙값으로 대표 경사와 림 값을 계산함.
  기본 5개 구간·20초, 경사 IQR 8° 이하를 확인함.
  내벽의 양의 상승 구간만 유효 구간 수로 셈.
  신호가 배경 오차보다 약하거나 함몰 관측 근거가 약한 프레임은 보류.
  이 조건 때문에 얕거나 오래된 크레이터의 측정이 누락될 수도 있으므로
  누락을 열화 판정으로 바꾸지 않고, 보류 비율과 재관측 필요를 함께 기록함.
  거리 자체에 대한 상한은 추가하지 않음.

  policy.json의 수치는 현장 조정을 위한 예비 공학 설정이며 문헌 상수가 아님.
  단순히 순위를 만들기 위해 조건을 낮추지 말고 측정·반복성 실험으로 조정.
  10분 촬영 전체 길이보다 각 대상의 유효 관측 구간 수가 중요함.

7. 실제 검증
  영상 4 실제 CDA 상자 59건, RealOutput 385건을 원시 IR·Depth로 재측정.
  경사·림 두 값이 함께 나온 관측은 각각 33건, 239건.
  재측정 경사·림 값은 이전 동일 입력 측정과 일치함.
  실행 조건을 통과한 확정 순위는 현재 기록에서 0개이며 보류 값을 구분 출력.
  이전 영상 4 비교는 보정한 라벨 영역의 저장값 재현으로 별도 기록함.
  실제 실험 자료와 그림은 코드 ZIP에 섞지 않고 별도 결과 폴더에 제공함.

8. 테스트
  python -m unittest discover -s tests -v
  표준 라이브러리 비교 코어: Python 3.12에서 30개 단위 테스트 통과.
  Python 3.8 문법 호환 검사 포함. Python 3.8 실제 런타임 검증과는 다름.
