# `/cda/detections` 출력 계약

토픽 형식은 `std_msgs/msg/String`이며 `data`에 JSON 1개가 들어갑니다.

```json
{
  "stamp_ns": 11530698200,
  "camera": "infra1",
  "source": "yolo",
  "dets": [
    {
      "box": [600.14, 295.07, 691.09, 346.50],
      "score": 0.521212,
      "class": "crater",
      "depth_m": 0.62,
      "depth_filter_applied": true,
      "depth_profile_pass": true,
      "depth_decision_state": "confirmed_crater",
      "depth_reason": "ground_corrected_3d_depression_pass"
    }
  ]
}
```

지도 핀 모듈의 필수 입력은 기존과 동일합니다.

- `stamp_ns`
- `dets[].box`
- `dets[].score`
- `dets[].class`

Depth 필드는 진단/근거리 검증용 추가 정보이며 지도 모듈이 무시해도 기존 인터페이스는 유지됩니다.

## Depth 판정 상태

- `confirmed_crater`: 분석 가능한 Depth에서 설정된 함몰 형상 기준을 충족함.
- `rejected_flat`: 신뢰 가능한 근거리 Depth에서 평탄함이 강하게 확인되어 제거됨.
- `indeterminate_pass`: 근거리 Depth는 분석했지만 확정적인 양/음성 근거가 부족하여 Recall 보호 차원에서 유지.
- `advisory_pass`: 원거리, Depth 부족, ground plane 품질 부족 등으로 Depth가 veto에 사용되지 않음.

`depth_profile_pass=true`는 출력 후보 유지 여부임. 실제 계산 여부는 `depth_profile_computed`, 후보 제외 권한의 필터 적용 여부는 `depth_filter_applied`, 처리 결과는 `depth_decision_state`로 구분함.

1 m 밖도 기본적으로 계산함. 지면·프로파일 계산이 완료되면 `depth_profile_computed=true`이고, 기본 원거리 정책의 `depth_filter_applied=false`와 동시에 나올 수 있음. 이는 계산을 건너뛴 상태와 다름.

추가 진단 필드:

- `depth_profile_computed`
- `depth_rejection_allowed`
- `depth_reliable_range_max_m`

- `depth_valid_ratio`
- `depth_reliable_valid_ratio`
- `depression_ratio`
- `coherent_depression_ratio`
- `center_depression_m`
- `bowl_amplitude_m`
- `center_valid_ratio`
- `effective_depth_threshold_m`
- `ground_plane_inlier_ratio`
- `ground_plane_rmse_m`
- `camera_xyz_m`
