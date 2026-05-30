"""Verify 9-axis AHRS roll estimate TRACKS JSBSim truth roll through a turn.
The 6-axis failure mode (design Appendix C) was: truth roll swept to ~84deg
while the accel-only estimate stayed pinned near -3deg. 9-axis must follow."""
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else 'sitl_log.csv')))

def at(t, col):
    best = min(rows, key=lambda r: abs(float(r['t']) - t))
    return float(best[col])

samples = [t/2.0 for t in range(8, 17)]  # 4.0 .. 8.0s
max_truth = max(abs(at(t, 'roll_true')) for t in samples)
max_err = max(abs(at(t, 'roll_true') - at(t, 'roll_est')) for t in samples)
print('max |truth roll| in turn:', round(max_truth,1), 'deg')
print('max |truth - est| in turn:', round(max_err,1), 'deg')
assert max_truth > 15.0, 'scenario did not produce a real banked turn'
assert max_err < 15.0, '9-axis AHRS estimate did not track truth roll in turn'
print('TURN-HOLD TRACKING OK')
