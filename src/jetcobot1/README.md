# JetCobot1 — Pick & Place 자동화 시스템

MyCobot280 + ROS2 + OpenCV(HSV/IBVS) 기반 비전 픽앤플레이스 시스템.
박스를 카메라로 감지 → 그리퍼 정렬 → 픽업 → 대기 슬롯에 적재 →
핑키(Pinky AGV) 도착 시 priority aging 순으로 출고.

## 실행

```bash
python3 main.py                 # 기본 실행
python3 main.py --flip-x         # J1 회전 방향 반전
python3 main.py --stream         # MJPEG 스트리밍 서버 동시 실행
python3 main.py --voice-server   # 음성 명령 수신 서버(8011) 동시 실행
```

키보드 명령(실행 중): `I` 입고 트리거 · `P` 핑키 도착 · `S` 테이블 현황 ·
`R` 일시정지/재시작 · `Q` 종료

## 파일 구성 및 의존 관계

```
main.py                 — 전체 런처. 아래 모듈들을 조립해 실행한다.
├── box_tracker.py        카메라 박스 감지(HSV) + 그리퍼 중앙 정렬(IBVS/EMA)
├── pick_and_place.py      Z 자동 계산 + 픽업/플레이스 시퀀스
│   └── box_tracker.py 의 BoxTracker/GripperAligner 등을 사용
├── weight_aging.py        대기 테이블(슬롯 3개) + priority aging + 핑키 출고
│   └── pick_and_place.py 의 PLACE_ANGLES_LIST, 내부 헬퍼 함수들을 사용
└── pause_control.py       관리자 일시정지/재시작 공용 모듈 (PAUSE_EVENT)
    └── 위 세 모듈이 모두 import해서 같은 이벤트를 공유

voice_command_server.py  — Jetson에서 실행. HTTP로 pause/resume 수신 → pause_control 제어
voice_pause_client.py     — 노트북에서 실행. STT(Whisper) → 키워드/LLM 분류 → 서버로 전송
```

`pause_control.py`는 비상정지(E-stop)가 아니라, 관리자가 정리/청소 등을 위해
다음 동작 시작 전에 로봇을 잠시 멈추는 운영성 일시정지를 위한 모듈이다.

## TODO (실제 연동 시 교체 필요)

- `weight_aging.py`의 `simulate_pinky_arrive()` → Task Manager ROS2 액션 수신으로 교체
- `weight_aging.py`의 `_notify_pinky_depart()` → ROS2 액션으로 PinkyPro 출발 신호 전송으로 교체
