"""
Dispatcher - 멀티 핑키 배차 전략
나중에 home으로 귀환한 idle 핑키 = 줄 바깥쪽에 위치 = 다음 출동 우선순위.
`last_home_arrival_ts` 기준으로 가장 최근 귀환한 idle 핑키를 반환.
"""

import threading
import time


class Dispatcher:
    def __init__(self):
        self._lock = threading.Lock()

    def pick_pinky(self, robots: dict) -> str | None:
        """idle pinky 중 last_home_arrival_ts가 가장 큰(=가장 최근 귀환) 로봇 반환."""
        with self._lock:
            candidates = [
                (rid, r.get("last_home_arrival_ts", 0.0))
                for rid, r in robots.items()
                if r.get("type") == "pinky" and r.get("status") == "idle"
            ]
            if not candidates:
                return None
            # 가장 최근 귀환한 (ts가 가장 큰) 로봇 우선
            candidates.sort(key=lambda x: x[1], reverse=True)
            return candidates[0][0]

    def on_home_arrived(self, robots: dict, robot_id: str):
        """핑키가 home에 도착했을 때 귀환 시각을 갱신."""
        with self._lock:
            if robot_id in robots:
                robots[robot_id]["last_home_arrival_ts"] = time.time()
