"""
TaskQueue - zone 점유/리소스 부족으로 즉시 dispatch 불가한 태스크 대기열.
"""

import threading
from collections import deque


class TaskQueue:
    def __init__(self):
        self._q: deque = deque()
        self._lock = threading.Lock()

    def submit(self, task, try_dispatch_fn) -> bool:
        """
        태스크 제출. try_dispatch_fn(task) → bool 호출하여 즉시 실행 시도.
        성공하면 True, 실패하면 대기열에 추가하고 False 반환.
        """
        if try_dispatch_fn(task):
            return True
        with self._lock:
            self._q.append(task)
        return False

    def drain(self, try_dispatch_fn):
        """대기열의 모든 태스크에 try_dispatch_fn을 적용. 성공한 것만 큐에서 제거."""
        with self._lock:
            pending = list(self._q)
            self._q.clear()

        remaining = []
        for task in pending:
            if not try_dispatch_fn(task):
                remaining.append(task)

        if remaining:
            with self._lock:
                # 새로 들어온 태스크보다 기존 대기 태스크가 먼저 처리되도록 앞쪽에 삽입
                for t in reversed(remaining):
                    self._q.appendleft(t)

    def pending_count(self) -> int:
        with self._lock:
            return len(self._q)
