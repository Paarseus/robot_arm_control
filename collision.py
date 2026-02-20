"""
Collision Detection and Compliant Reaction

Monitors motor torque feedback and position error to detect collisions.
On detection, drops kp to 0 (compliant mode) so the arm yields to contact.
After the collision clears, ramps kp back to normal over recovery_time.

Follows K-Scale protective_torque + protection_time pattern and ISO 15066 principles.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional

MIN_RECOVERY_TIME = 0.1  # seconds, floor to prevent div-by-zero or instant snap-back


@dataclass
class JointCollisionConfig:
    protective_torque: float = 8.0  # Nm threshold for collision
    protection_time: float = 0.03  # seconds torque must exceed threshold
    position_error_threshold: float = 15.0  # degrees, backup detection
    recovery_time: float = 1.0  # seconds to ramp kp back to normal
    # Stall detection (AND): |velocity| < threshold AND |position_error| > threshold
    velocity_threshold: float = 0.0  # deg/s, 0 = stall detection disabled
    stall_position_error: float = 5.0  # degrees
    stall_time: float = 0.05  # seconds (50ms)


@dataclass
class JointCollisionState:
    torque_exceed_start: Optional[float] = None  # timestamp when torque first exceeded
    collision_active: bool = False
    collision_start_time: float = 0.0  # when collision was confirmed
    recovery_start_time: Optional[float] = None  # when collision cleared, recovery began
    stall_start: Optional[float] = None  # when stall condition first met


class CollisionDetector:
    """
    Per-joint collision detection with torque threshold and position error backup.

    Usage:
        detector = CollisionDetector(configs)
        # Each control cycle (order matters: update THEN get_kp_scale):
        detector.update("elbow", torque_nm=5.2, position_error_deg=2.0, timestamp=t)
        kp_scale = detector.get_kp_scale("elbow", timestamp=t)  # 0.0 to 1.0
        effective_kp = base_kp * kp_scale
    """

    def __init__(self, configs: Dict[str, JointCollisionConfig]):
        self._configs = configs
        self._states: Dict[str, JointCollisionState] = {
            name: JointCollisionState() for name in configs
        }

    def update(
        self,
        joint: str,
        torque_nm: float,
        position_error_deg: float,
        timestamp: float,
        velocity_dps: Optional[float] = None,
    ):
        cfg = self._configs.get(joint)
        state = self._states.get(joint)
        if cfg is None or state is None:
            return

        torque_exceeded = abs(torque_nm) > cfg.protective_torque
        position_exceeded = abs(position_error_deg) > cfg.position_error_threshold

        if state.collision_active:
            # Clear collision when torque drops below threshold.
            # Position error is NOT checked here because kp=0 during collision
            # means the motor can't return to position — the error stays large
            # even after the external force is removed.
            if not torque_exceeded:
                state.collision_active = False
                state.recovery_start_time = timestamp
                state.torque_exceed_start = None
                state.stall_start = None
        else:
            # Finalize recovery if complete (moved here from get_kp_scale to keep getter pure)
            if state.recovery_start_time is not None:
                recovery_time = max(cfg.recovery_time, MIN_RECOVERY_TIME)
                if (timestamp - state.recovery_start_time) >= recovery_time:
                    state.recovery_start_time = None

            # During recovery (kp < 1), skip position error detection since the
            # motor is still ramping back and position error will be large
            in_recovery = state.recovery_start_time is not None

            # Stall detection (AND): low velocity + position error, sustained
            if (
                velocity_dps is not None
                and cfg.velocity_threshold > 0
                and not in_recovery
            ):
                stalled = (
                    abs(velocity_dps) < cfg.velocity_threshold
                    and abs(position_error_deg) > cfg.stall_position_error
                )
                if stalled:
                    if state.stall_start is None:
                        state.stall_start = timestamp
                    if (timestamp - state.stall_start) >= cfg.stall_time:
                        state.collision_active = True
                        state.collision_start_time = timestamp
                        state.stall_start = None
                        state.recovery_start_time = None
                        return  # Don't also check torque
                else:
                    state.stall_start = None

            # Check for new collision
            # Backup: position error triggers immediately (only when kp is at full)
            if position_exceeded and not in_recovery:
                state.collision_active = True
                state.collision_start_time = timestamp
                state.torque_exceed_start = None
                state.recovery_start_time = None
            # Primary: torque must be sustained for protection_time
            elif torque_exceeded:
                if state.torque_exceed_start is None:
                    state.torque_exceed_start = timestamp
                if (
                    state.torque_exceed_start is not None
                    and (timestamp - state.torque_exceed_start) >= cfg.protection_time
                ):
                    state.collision_active = True
                    state.collision_start_time = timestamp
                    state.torque_exceed_start = None
                    state.recovery_start_time = None
            else:
                state.torque_exceed_start = None

    def in_collision(self, joint: str) -> bool:
        state = self._states.get(joint)
        return state.collision_active if state else False

    def any_collision(self) -> bool:
        return any(s.collision_active for s in self._states.values())

    def get_kp_scale(self, joint: str, timestamp: float) -> float:
        """
        Returns kp multiplier: 0.0 during collision, linear ramp 0->1 during recovery, 1.0 normal.
        Pure query — does not mutate state.
        """
        state = self._states.get(joint)
        cfg = self._configs.get(joint)
        if state is None or cfg is None:
            return 1.0

        if state.collision_active:
            return 0.0

        # Recovery ramp (clamped to [0, 1] for safety)
        if state.recovery_start_time is not None:
            recovery_time = max(cfg.recovery_time, MIN_RECOVERY_TIME)
            elapsed = timestamp - state.recovery_start_time
            scale = elapsed / recovery_time
            return max(0.0, min(1.0, scale))

        return 1.0

    def get_active_collisions(self) -> List[str]:
        return [name for name, s in self._states.items() if s.collision_active]


if __name__ == "__main__":
    print("CollisionDetector self-test...")

    configs = {
        "shoulder": JointCollisionConfig(
            protective_torque=10.0, protection_time=0.03, recovery_time=1.0
        ),
        "elbow": JointCollisionConfig(
            protective_torque=6.0,
            protection_time=0.02,
            position_error_threshold=10.0,
            recovery_time=0.5,
        ),
    }
    det = CollisionDetector(configs)

    # Test 1: No collision when below threshold
    det.update("shoulder", torque_nm=5.0, position_error_deg=2.0, timestamp=0.0)
    assert not det.in_collision("shoulder"), "Should not collide below threshold"
    assert det.get_kp_scale("shoulder", 0.0) == 1.0, "kp should be 1.0"

    # Test 2: Torque exceeded but not yet sustained long enough
    det.update("shoulder", torque_nm=12.0, position_error_deg=2.0, timestamp=0.01)
    assert not det.in_collision("shoulder"), "Should not collide yet (protection_time)"
    det.update("shoulder", torque_nm=12.0, position_error_deg=2.0, timestamp=0.02)
    assert not det.in_collision("shoulder"), "Still under protection_time"

    # Test 3: Torque sustained past protection_time -> collision
    det.update("shoulder", torque_nm=12.0, position_error_deg=2.0, timestamp=0.05)
    assert det.in_collision("shoulder"), "Should be in collision now"
    assert det.get_kp_scale("shoulder", 0.05) == 0.0, "kp should be 0 during collision"
    assert det.any_collision(), "any_collision should be True"
    assert "shoulder" in det.get_active_collisions()

    # Test 4: Position error triggers immediate collision on elbow
    det.update("elbow", torque_nm=1.0, position_error_deg=15.0, timestamp=0.01)
    assert det.in_collision("elbow"), "Position error should trigger immediate collision"

    # Test 5: Collision clears when torque drops (position error ignored during clear)
    # Position error stays large because kp=0 means motor can't return
    det.update("shoulder", torque_nm=3.0, position_error_deg=30.0, timestamp=1.0)
    assert not det.in_collision("shoulder"), "Should clear on torque alone (pos error ignored)"

    # Test 6: Recovery ramp
    scale = det.get_kp_scale("shoulder", 1.0)
    assert scale == 0.0, f"kp should be 0 at start of recovery, got {scale}"

    scale = det.get_kp_scale("shoulder", 1.5)
    assert 0.49 < scale < 0.51, f"kp should be ~0.5 halfway, got {scale}"

    scale = det.get_kp_scale("shoulder", 2.0)
    assert scale == 1.0, f"kp should be 1.0 after recovery, got {scale}"

    # Test 7: Elbow clears and recovers (shorter recovery_time)
    det.update("elbow", torque_nm=1.0, position_error_deg=3.0, timestamp=2.0)
    assert not det.in_collision("elbow"), "Elbow should have cleared"
    scale = det.get_kp_scale("elbow", 2.25)
    assert 0.49 < scale < 0.51, f"Elbow kp should be ~0.5, got {scale}"
    scale = det.get_kp_scale("elbow", 2.5)
    assert scale == 1.0, f"Elbow kp should be 1.0, got {scale}"

    # Test 8: No collision on unknown joint
    assert not det.in_collision("unknown")
    assert det.get_kp_scale("unknown", 0.0) == 1.0

    # Test 9: kp_scale is always clamped to [0, 1] even with backward timestamps
    det2 = CollisionDetector(
        {"j": JointCollisionConfig(protective_torque=5.0, recovery_time=1.0)}
    )
    det2.update("j", torque_nm=10.0, position_error_deg=0.0, timestamp=0.1)
    det2.update("j", torque_nm=10.0, position_error_deg=0.0, timestamp=0.2)
    det2.update("j", torque_nm=1.0, position_error_deg=0.0, timestamp=1.0)  # clears
    # Simulate backward timestamp
    scale = det2.get_kp_scale("j", 0.5)  # before recovery_start_time
    assert scale == 0.0, f"kp_scale with backward timestamp should be 0, got {scale}"
    scale = det2.get_kp_scale("j", 1.0)
    assert scale == 0.0, f"kp_scale at recovery start should be 0, got {scale}"

    # Test 10: get_kp_scale is idempotent (pure query, no mutation)
    s1 = det.get_kp_scale("shoulder", 100.0)
    s2 = det.get_kp_scale("shoulder", 100.0)
    assert s1 == s2, f"get_kp_scale should be idempotent: {s1} != {s2}"

    # Test 11: recovery_time=0 doesn't cause division by zero (uses MIN_RECOVERY_TIME)
    det3 = CollisionDetector(
        {"j": JointCollisionConfig(protective_torque=5.0, recovery_time=0.0)}
    )
    det3.update("j", torque_nm=10.0, position_error_deg=0.0, timestamp=0.1)
    det3.update("j", torque_nm=10.0, position_error_deg=0.0, timestamp=0.2)
    det3.update("j", torque_nm=1.0, position_error_deg=0.0, timestamp=1.0)
    scale = det3.get_kp_scale("j", 1.05)
    assert 0.0 <= scale <= 1.0, f"recovery_time=0 should not break scale: {scale}"

    # Test 12: Position error does NOT re-trigger collision during recovery
    det5 = CollisionDetector(
        {"j": JointCollisionConfig(protective_torque=5.0, protection_time=0.0, recovery_time=1.0)}
    )
    det5.update("j", torque_nm=10.0, position_error_deg=0.0, timestamp=0.0)
    assert det5.in_collision("j"), "Should be in collision"
    det5.update("j", torque_nm=1.0, position_error_deg=25.0, timestamp=1.0)  # torque low, pos error high
    assert not det5.in_collision("j"), "Should clear (torque low, pos error ignored)"
    # During recovery, large position error should NOT re-trigger collision
    det5.update("j", torque_nm=1.0, position_error_deg=25.0, timestamp=1.2)
    assert not det5.in_collision("j"), "Position error should not re-trigger during recovery"
    scale = det5.get_kp_scale("j", 1.5)
    assert 0.49 < scale < 0.51, f"Should be recovering, got {scale}"

    # Test 13: Collision at timestamp=0 works correctly
    det4 = CollisionDetector(
        {"j": JointCollisionConfig(protective_torque=5.0, protection_time=0.0)}
    )
    det4.update("j", torque_nm=10.0, position_error_deg=0.0, timestamp=0.0)
    assert det4.in_collision("j"), "Should detect collision at t=0 with protection_time=0"

    # --- Stall detection tests ---

    # Shared config: torque threshold set high so only stall path triggers
    stall_cfg = JointCollisionConfig(
        protective_torque=100.0,
        velocity_threshold=2.0,  # deg/s
        stall_position_error=5.0,  # degrees
        stall_time=0.05,  # 50ms
    )

    # Test 14: Stall triggers — low velocity + position error sustained
    det_stall = CollisionDetector({"j": stall_cfg})
    # Stall condition met but not yet sustained
    det_stall.update("j", torque_nm=1.0, position_error_deg=10.0, timestamp=0.0, velocity_dps=0.5)
    assert not det_stall.in_collision("j"), "Stall should not trigger before stall_time"
    # Still stalled, just under stall_time
    det_stall.update("j", torque_nm=1.0, position_error_deg=10.0, timestamp=0.04, velocity_dps=0.5)
    assert not det_stall.in_collision("j"), "Stall should not trigger at 40ms < 50ms"
    # Past stall_time — should trigger
    det_stall.update("j", torque_nm=1.0, position_error_deg=10.0, timestamp=0.06, velocity_dps=0.5)
    assert det_stall.in_collision("j"), "Stall should trigger after 50ms sustained"

    # Test 15: No stall — velocity above threshold
    det_stall = CollisionDetector({"j": stall_cfg})
    det_stall.update("j", torque_nm=1.0, position_error_deg=10.0, timestamp=0.0, velocity_dps=5.0)
    det_stall.update("j", torque_nm=1.0, position_error_deg=10.0, timestamp=0.1, velocity_dps=5.0)
    assert not det_stall.in_collision("j"), "No stall when velocity > threshold"

    # Test 16: No stall — position error below stall threshold
    det_stall = CollisionDetector({"j": stall_cfg})
    det_stall.update("j", torque_nm=1.0, position_error_deg=2.0, timestamp=0.0, velocity_dps=0.5)
    det_stall.update("j", torque_nm=1.0, position_error_deg=2.0, timestamp=0.1, velocity_dps=0.5)
    assert not det_stall.in_collision("j"), "No stall when position error < stall threshold"

    # Test 17: Backward compat — no velocity passed, stall detection skipped
    det_stall = CollisionDetector({"j": stall_cfg})
    det_stall.update("j", torque_nm=1.0, position_error_deg=10.0, timestamp=0.0)
    det_stall.update("j", torque_nm=1.0, position_error_deg=10.0, timestamp=0.1)
    assert not det_stall.in_collision("j"), "No stall when velocity_dps not passed"

    # Test 18: Stall disabled — velocity_threshold=0, even with velocity passed
    stall_disabled_cfg = JointCollisionConfig(
        protective_torque=100.0,
        velocity_threshold=0.0,  # disabled
        stall_position_error=5.0,
        stall_time=0.05,
    )
    det_stall = CollisionDetector({"j": stall_disabled_cfg})
    det_stall.update("j", torque_nm=1.0, position_error_deg=10.0, timestamp=0.0, velocity_dps=0.5)
    det_stall.update("j", torque_nm=1.0, position_error_deg=10.0, timestamp=0.1, velocity_dps=0.5)
    assert not det_stall.in_collision("j"), "No stall when velocity_threshold=0"

    print("All tests passed!")
