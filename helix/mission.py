from __future__ import annotations
from dataclasses import dataclass
from typing import List

PHASES: List[str] = [
    "TAKEOFF", "TRANSIT", "RECON AREA", "ORBIT", "RETURN", "LAND"
]


@dataclass
class MissionState:
    phase_idx: int = 3

    @property
    def phase(self) -> str:
        return PHASES[self.phase_idx]

    def advance(self) -> bool:
        if self.phase_idx < len(PHASES) - 1:
            self.phase_idx += 1
            return True
        return False

    def set_phase(self, idx: int) -> None:
        self.phase_idx = max(0, min(len(PHASES) - 1, idx))