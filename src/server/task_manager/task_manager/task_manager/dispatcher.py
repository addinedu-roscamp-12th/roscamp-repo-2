"""
Dispatcher - 멀티 핑키 배차 전략
home_stack 기준으로 가장 최근 귀환한(바깥쪽) idle 핑키를 우선 배차.
home_stack: [inner, ..., outer] — top(마지막) = 바깥쪽 = 먼저 배차.
"""

import threading


class Dispatcher:
    def __init__(self):
        self._lock = threading.Lock()

    def pick_pinky(self, robots: dict, home_stack: list = None) -> str | None:
        """
        home_stack top(바깥쪽)부터 idle 핑키 탐색.
        스택에 없는 idle 핑키는 fallback으로 반환.
        """
        with self._lock:
            if home_stack:
                for pinky_id in reversed(home_stack):
                    if robots.get(pinky_id, {}).get("status") == "idle":
                        return pinky_id
            # fallback: 스택 외 idle 핑키
            for rid, r in robots.items():
                if r.get("type") == "pinky" and r.get("status") == "idle":
                    return rid
            return None
