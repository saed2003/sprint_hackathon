"""
simulate_follow.py — line-follower simulator (no robot needed).

Mocks the RasBot hardware and tests advanced_follow.py logic against
synthetic tape patterns: straight lines, 90° turns, T-junctions, dead-ends,
and the black-square terminal.

Run: python3 src/tape_following/simulate_follow.py
"""

import os
import sys
import time
import random

# ============================================================================
#   CONTROLLER TUNING CONSTANTS (copied from advanced_follow.py)
# ============================================================================
BASE_SPEED          = 120
Kp                  = 40
Kd                  = 25
LOOP_DELAY          = 0.01
DEADBAND            = 1.0
MOTOR_TRIM          = 1.00
TURN_SPEED          = 130
JUNCTION_CLEAR_TIME = 0.15
SPIN_180_TIME       = 0.90
DEAD_END_CONFIRM    = 8
PWM_MIN, PWM_MAX    = -255, 255


def clamp(val, lo=PWM_MIN, hi=PWM_MAX):
    return max(lo, min(val, hi))


def sensor_error(L1, L2, R1, R2):
    """Weighted lateral error. Returns None if line is lost."""
    if L2 and R1:            return 0.0
    if L2 and not R1:        return -1.0
    if L1 and L2:            return -1.5
    if L1 and not L2:        return -2.0
    if R1 and not L2:        return 1.0
    if R1 and R2:            return 1.5
    if R2 and not R1:        return 2.0
    return None


def drive(bot, left, right):
    """Differential drive with MOTOR_TRIM applied."""
    left = clamp(int(left * MOTOR_TRIM))
    right = clamp(int(right))
    bot._apply_motors(left, left, right, right)


# ============================================================================
#   MOCK RASBOT (replaces hardware with synthetic sensor data)
# ============================================================================

class MockRasBot:
    """Fake robot: simulates IR line sensors and motor commands."""

    def __init__(self, tape_path="straight"):
        self.tape_path = tape_path
        self.position = 0.0           # progress along the tape (0 → 1)
        self.lateral_error = 0.0      # mm offset from center (-50 … +50)
        self.velocity = 0.5           # simulation units/loop
        self.motor_history = []       # log of all motor commands
        self.step = 0

    def read_line_sensors(self):
        """Return (L1, L2, R1, R2) based on position + lateral error."""
        # Get the tape profile at this position
        profile = self._tape_profile_at(self.position)
        # Tape profile is a list of 4 bools: [L1, L2, R1, R2]
        # Add realistic noise: small drift
        self.lateral_error += random.gauss(0, 0.5)
        self.lateral_error = max(-50, min(50, self.lateral_error))

        # Sensors fire if lateral error is within their detection zone
        # L1 outer: -50 to -25, L2 inner: -25 to 0, R1 inner: 0 to 25, R2 outer: 25 to 50
        L1 = profile[0] and -50 <= self.lateral_error < -25
        L2 = profile[1] and -25 <= self.lateral_error < 0
        R1 = profile[2] and 0 <= self.lateral_error < 25
        R2 = profile[3] and 25 <= self.lateral_error < 50

        return L1, L2, R1, R2

    def _tape_profile_at(self, pos):
        """Return tape profile (4 bools) at progress position [0, 1]."""
        patterns = {
            "straight": self._pattern_straight,
            "left_90": self._pattern_left_90,
            "right_90": self._pattern_right_90,
            "t_junction": self._pattern_t_junction,
            "dead_end_1": self._pattern_dead_end_1,
            "circuit": self._pattern_circuit,
        }
        fn = patterns.get(self.tape_path, self._pattern_straight)
        return fn(pos)

    @staticmethod
    def _pattern_straight(pos):
        """Straight line: all 4 sensors always see tape (center)."""
        return [True, True, True, True]

    @staticmethod
    def _pattern_left_90(pos):
        """Straight (0–0.4) → left turn (0.4–0.6) → straight again (0.6–1)."""
        if pos < 0.4:
            return [True, True, True, True]
        if pos < 0.6:
            # Hard left: only L sensors
            return [True, True, False, False]
        return [True, True, True, True]

    @staticmethod
    def _pattern_right_90(pos):
        """Straight (0–0.4) → right turn (0.4–0.6) → straight again (0.6–1)."""
        if pos < 0.4:
            return [True, True, True, True]
        if pos < 0.6:
            # Hard right: only R sensors
            return [False, False, True, True]
        return [True, True, True, True]

    @staticmethod
    def _pattern_t_junction(pos):
        """Straight (0–0.3) → T-junction (0.3–0.5) → continuation (0.5–1)."""
        if pos < 0.3:
            return [True, True, True, True]
        if pos < 0.5:
            # T: left branch + straight
            return [True, True, True, False]
        return [True, True, True, True]

    @staticmethod
    def _pattern_dead_end_1(pos):
        """Straight (0–0.5) → dead end (0.5–1): all white."""
        if pos < 0.5:
            return [True, True, True, True]
        # Dead end: nothing
        return [False, False, False, False]

    @staticmethod
    def _pattern_circuit(pos):
        """Full loop: straight → right turn → straight → left turn → dead end."""
        if pos < 0.2:       return [True, True, True, True]
        if pos < 0.35:      return [False, False, True, True]  # right 90°
        if pos < 0.5:       return [True, True, True, True]
        if pos < 0.65:      return [True, True, False, False]  # left 90°
        if pos < 0.75:      return [True, True, True, True]
        return [False, False, False, False]  # dead end

    def _apply_motors(self, lf, lr, rf, rr):
        """Capture motor commands. Simulate motion: avg both motors."""
        self.motor_history.append((lf, lr, rf, rr))
        # Simulate differential drive: average left/right to get forward
        # + compute turning effect
        left_avg = (lf + lr) / 2.0 if (lf + lr) else 0
        right_avg = (rf + rr) / 2.0 if (rf + rr) else 0
        fwd = (left_avg + right_avg) / 2.0 / 255.0 * self.velocity
        turn = (left_avg - right_avg) / 255.0 * 0.1  # turn effect
        self.position += fwd
        self.lateral_error -= turn * 20  # steering moves the error
        self.position = max(0, min(1, self.position))
        self.step += 1

    def stop(self):
        """Motor off."""
        self._apply_motors(0, 0, 0, 0)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.stop()


# ============================================================================
#   TEST RUNNER
# ============================================================================

def run_simulation(tape_path="straight", max_steps=500, verbose=True):
    """Run the line follower on a simulated tape pattern."""
    print("=" * 70)
    print(f"SIMULATION: {tape_path.upper()}")
    print("=" * 70)

    # Run the controller
    bot = MockRasBot(tape_path=tape_path)
    last_error = 0.0
    turning_state = "none"
    dead_end_count = 0
    lost_streak = 0
    loop = 0
    step_log = []

    try:
        while loop < max_steps:
            # --- run one iteration of advanced_follow's main loop logic ---
            L1, L2, R1, R2 = bot.read_line_sensors()

            # PROBLEM 4: BLACK SQUARE
            if L1 and L2 and R1 and R2:
                print(f"\n[OK] STEP {loop}: BLACK SQUARE REACHED — STOP")
                bot.stop()
                step_log.append(("BLACK_SQUARE", bot.position, turning_state, dead_end_count))
                return True, loop, step_log

            # PROBLEM 1: TURN LOCKOUT
            if turning_state != "none":
                if L2 and R1:
                    turning_state = "none"
                    last_error = 0.0
                else:
                    if turning_state == "left":
                        bot._apply_motors(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
                    else:
                        bot._apply_motors(TURN_SPEED, TURN_SPEED, -TURN_SPEED, -TURN_SPEED)
                    step_log.append(("LOCKED", bot.position, turning_state, dead_end_count))
                    loop += 1
                    continue

            # PROBLEM 3a: T-JUNCTION
            t_left = (L1 and L2 and R1 and not R2)
            t_right = (not L1 and L2 and R1 and R2)
            if t_left or t_right:
                choice = random.choice(["left", "right"])
                if verbose:
                    print(f"  STEP {loop}: T-JUNCTION → {choice.upper()}")
                for _ in range(int(JUNCTION_CLEAR_TIME / LOOP_DELAY)):
                    if choice == "left":
                        bot._apply_motors(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
                    else:
                        bot._apply_motors(TURN_SPEED, TURN_SPEED, -TURN_SPEED, -TURN_SPEED)
                last_error = 0.0
                lost_streak = 0
                step_log.append(("JUNCTION", bot.position, choice, dead_end_count))
                loop += 1
                continue

            # compute error
            error = sensor_error(L1, L2, R1, R2)

            # PROBLEM 3b: DEAD-END / LOOP PREVENTION
            if error is None:
                lost_streak += 1
                if lost_streak < DEAD_END_CONFIRM:
                    nudge = clamp(int(Kp * (last_error if last_error else 0)))
                    bot._apply_motors(
                        int((BASE_SPEED * 0.6 + nudge) * MOTOR_TRIM),
                        int((BASE_SPEED * 0.6 + nudge) * MOTOR_TRIM),
                        int(BASE_SPEED * 0.6 - nudge),
                        int(BASE_SPEED * 0.6 - nudge),
                    )
                    step_log.append(("LOST", bot.position, f"drift {lost_streak}/{DEAD_END_CONFIRM}", dead_end_count))
                    loop += 1
                    continue

                # Confirmed dead-end
                dead_end_count += 1
                lost_streak = 0
                bot.stop()
                if dead_end_count == 1:
                    if verbose:
                        print(f"  STEP {loop}: DEAD-END #1 — spinning 180°")
                    # Simulate 180° spin (timed)
                    for _ in range(int(SPIN_180_TIME / LOOP_DELAY)):
                        bot._apply_motors(TURN_SPEED, TURN_SPEED, -TURN_SPEED, -TURN_SPEED)
                    bot.stop()
                    last_error = 0.0
                    turning_state = "none"
                    step_log.append(("DEAD_END_1", bot.position, "spinning back", dead_end_count))
                    loop += 1
                    continue
                else:
                    print(f"\n[OK] STEP {loop}: DEAD-END #2 — back at origin, STOP")
                    step_log.append(("DEAD_END_2", bot.position, "origin", dead_end_count))
                    return True, loop, step_log

            lost_streak = 0

            # PROBLEM 1: HARD TURN TRIGGER
            if L1 and not L2 and not R1 and not R2:
                turning_state = "left"
                if verbose:
                    print(f"  STEP {loop}: LOCKOUT LEFT (L1 only)")
                bot._apply_motors(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
                step_log.append(("HARD_LEFT", bot.position, "enter lockout", dead_end_count))
                loop += 1
                continue
            if R2 and not R1 and not L2 and not L1:
                turning_state = "right"
                if verbose:
                    print(f"  STEP {loop}: LOCKOUT RIGHT (R2 only)")
                bot._apply_motors(TURN_SPEED, TURN_SPEED, -TURN_SPEED, -TURN_SPEED)
                step_log.append(("HARD_RIGHT", bot.position, "enter lockout", dead_end_count))
                loop += 1
                continue

            # PROBLEM 2: DEADBAND + PD
            if abs(error) <= DEADBAND:
                correction = 0
            else:
                derivative = error - last_error
                correction = int(Kp * error + Kd * derivative)

            last_error = error
            left_speed = clamp(BASE_SPEED + correction)
            right_speed = clamp(BASE_SPEED - correction)
            drive(bot, left_speed, right_speed)

            step_log.append(("TRACK", bot.position, f"err={error:.1f} corr={correction}", dead_end_count))
            loop += 1

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        bot.stop()

    print(f"\nSimulation ended after {loop} steps (max {max_steps})")
    print(f"Final position: {bot.position:.2f}")
    return False, loop, step_log


def print_summary(success, steps, log):
    """Print a summary of the simulation."""
    print("\n" + "=" * 70)
    if success:
        print("[OK] SUCCESS — reached checkpoint or origin")
    else:
        print("[WARN] INCOMPLETE — ran to max steps")
    print(f"Steps: {steps}")
    print("\nSample log (every 10 steps):")
    for i in range(0, len(log), max(1, len(log) // 20)):
        action, pos, detail, decount = log[i]
        print(f"  {i:3d}: {action:12s} @ {pos:.2f}  {detail:30s}  dead_ends={decount}")


# ============================================================================
#   MAIN
# ============================================================================

def main():
    patterns = [
        "straight",
        "left_90",
        "right_90",
        "t_junction",
        "dead_end_1",
        "circuit",
    ]

    print("\n+=======================================================================+")
    print("|  ADVANCED LINE FOLLOWER SIMULATOR                                    |")
    print("|  Tests PD control, lockout, junctions, dead-end prevention           |")
    print("+=======================================================================╝\n")

    for pattern in patterns:
        success, steps, log = run_simulation(pattern, max_steps=300, verbose=True)
        print_summary(success, steps, log)
        print()
        time.sleep(0.5)

    print("\nAll patterns tested. Run with a specific pattern:")
    print("  python3 simulate_follow.py --pattern circuit")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Line follower simulator")
    ap.add_argument("--pattern", choices=[
        "straight", "left_90", "right_90", "t_junction", "dead_end_1", "circuit"
    ], default="circuit", help="Tape pattern to simulate")
    ap.add_argument("--steps", type=int, default=300, help="Max simulation steps")
    args = ap.parse_args()

    success, steps, log = run_simulation(args.pattern, max_steps=args.steps, verbose=True)
    print_summary(success, steps, log)
