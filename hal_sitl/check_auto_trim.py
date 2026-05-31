"""Verify enabling auto-trim does not DESTABILIZE flight (bounded, non-divergent).
WARNING: JSBSim c172p is a symmetric airframe, so learned trim ~= 0 here. And with
fixed throttle the longitudinal phugoid makes pitch/altitude oscillate slowly — that is
airframe physics, NOT auto-trim. This test therefore checks the auto-trim-relevant
invariant (roll, the axis auto-trim works on at centered stick, stays well-bounded) and
that nothing diverges. It does NOT verify auto-trim corrects real asymmetry (unverified;
needs an asymmetric airframe on real hardware)."""
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else 'sitl_log.csv')))
roll = [abs(float(r['roll_true'])) for r in rows]
pitch = [abs(float(r['pitch_true'])) for r in rows]
roll_max = max(roll)
pitch_max = max(pitch)
print('roll |max| (deg): ', round(roll_max, 1))
print('pitch |max| (deg):', round(pitch_max, 1))
# Roll is the centered-stick axis auto-trim acts on; it must stay tightly bounded.
assert roll_max < 15.0, 'auto-trim destabilized roll (centered-stick axis diverged)'
# Pitch is throttle/phugoid-dominated, not auto-trim; only require it stays bounded (no tumble).
assert pitch_max < 45.0, 'pitch diverged (not a controlled phugoid)'
print('AUTO-TRIM STABILITY OK (roll bounded; pitch is fixed-throttle phugoid, not auto-trim)')
print('NOTE: symmetric airframe -> trim~=0; correction-of-asymmetry UNVERIFIED (needs real HW)')
