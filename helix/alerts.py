from __future__ import annotations
from dataclasses import dataclass, field
from typing import List
import time


@dataclass
class Alert:
    message: str
    level: str = "info"
    timestamp: float = field(default_factory=time.time)

    def formatted(self) -> str:
        t = time.strftime("%H:%M:%S", time.localtime(self.timestamp))
        return f"[{t}] {self.level.upper():4s} {self.message}"


class AlertLog:
    def __init__(self, max_len: int = 100):
        self.max_len = max_len
        self._items: List[Alert] = []

    def push(self, message: str, level: str = "info") -> None:
        self._items.insert(0, Alert(message, level))
        if len(self._items) > self.max_len:
            self._items.pop()

    @property
    def items(self) -> List[Alert]:
        return list(self._items)

    def __iter__(self):
        return iter(self._items)