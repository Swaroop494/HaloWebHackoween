"""Small engine-owned fetch state machine and bounded recovery policy."""
from __future__ import annotations

from enum import Enum


class FetchPhase(str, Enum):
    IDLE = "IDLE"
    LOCATE = "LOCATE"
    APPROACH = "APPROACH"
    PICKUP = "PICKUP"
    CARRY = "CARRY"
    DROP = "DROP"
    DONE = "DONE"
    BLOCKED = "BLOCKED"


class FetchState:
    def __init__(self):
        self.phase = FetchPhase.IDLE
        self.previous_phase = FetchPhase.LOCATE
        self.retry_count = 0
        self.step = 0
        self.collided = False
        self.outcome: str | None = None

    def start(self):
        self.phase = FetchPhase.LOCATE

    def collision(self):
        self.collided = True
        self.previous_phase = self.phase
        self.retry_count += 1
        self.phase = FetchPhase.DONE if self.retry_count >= 3 else FetchPhase.BLOCKED
        if self.phase == FetchPhase.DONE:
            self.outcome = "fail_collision"

    def recover(self):
        if self.phase != FetchPhase.BLOCKED:
            return
        self.collided = False
        self.phase = self.previous_phase

    def picked_up(self, success: bool, holding: bool = False):
        self.phase = FetchPhase.CARRY if success or holding else FetchPhase.LOCATE

    def dropped(self, released: bool, in_zone: bool, holding: bool):
        if released and in_zone:
            self.phase, self.outcome = FetchPhase.DONE, "pass"
        elif released:
            self.phase = FetchPhase.CARRY
            self.retry_count += 1
            if self.retry_count >= 3:
                self.phase, self.outcome = FetchPhase.DONE, "fail_not_reached"
        else:
            self.phase = FetchPhase.CARRY if holding else FetchPhase.LOCATE

    def action(self):
        if self.phase == FetchPhase.LOCATE:
            self.phase = FetchPhase.APPROACH

