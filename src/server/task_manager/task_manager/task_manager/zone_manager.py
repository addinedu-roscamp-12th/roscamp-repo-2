"""
ZoneManager - 구역 점유/잠금 관리
- free:     비어있음, 누구나 점유 가능
- locked:   특정 로봇이 이동 중 (예약 상태)
- occupied: 물건이 적재되어 있음 (영구 점유, 명시적 release 필요)

`home`은 잠금 대상에서 제외 (여러 로봇이 동시에 대기 가능).
"""

import threading


class ZoneManager:
    # home은 잠금 대상 제외 (다중 로봇 동시 대기 허용)
    UNLOCKED_ZONES = {"home"}

    def __init__(self, zones: dict[str, str]):
        # zones: {"zone_1": "free", ...} — 외부 dict를 그대로 참조하여 동기화 유지
        self._zones = zones
        self._holder: dict[str, str | None] = {z: None for z in zones}
        self._lock = threading.Lock()

    def try_acquire(self, zone: str, robot_id: str) -> bool:
        """zone 잠금 시도. free 상태일 때만 성공."""
        if zone in self.UNLOCKED_ZONES:
            return True
        with self._lock:
            if zone not in self._zones:
                return False
            state = self._zones[zone]
            # 같은 로봇이 이미 holder면 재진입 허용 (멱등성)
            if state == "free" or self._holder.get(zone) == robot_id:
                self._zones[zone] = "locked"
                self._holder[zone] = robot_id
                return True
            return False

    def release(self, zone: str):
        """zone을 free로 해제."""
        if zone in self.UNLOCKED_ZONES:
            return
        with self._lock:
            if zone in self._zones:
                self._zones[zone] = "free"
                self._holder[zone] = None

    def mark_occupied(self, zone: str):
        """zone을 occupied로 승격 (물건 적재 완료)."""
        if zone in self.UNLOCKED_ZONES:
            return
        with self._lock:
            if zone in self._zones:
                self._zones[zone] = "occupied"
                # holder는 유지 — 물건 소유 로봇 추적용이 아니므로 None으로 비움
                self._holder[zone] = None

    def is_free(self, zone: str) -> bool:
        if zone in self.UNLOCKED_ZONES:
            return True
        with self._lock:
            return self._zones.get(zone) == "free"

    def get_holder(self, zone: str) -> str | None:
        with self._lock:
            return self._holder.get(zone)
