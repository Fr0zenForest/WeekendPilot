"""Verify Angle mode recovers roll toward level after a disturbance."""
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else 'sitl_log.csv')))
def roll_at(t):
    best = min(rows, key=lambda r: abs(float(r['t']) - t))
    return float(best['roll_true'])
peak = max(abs(roll_at(t/10.0)) for t in range(30, 40))   # 3.0-4.0s peak after kick
settled = abs(roll_at(8.0))                                # near end
print('peak roll after disturbance:', round(peak,1), 'deg')
print('roll at t=8s:', round(settled,1), 'deg')
assert settled < peak * 0.5, 'Angle mode did not recover toward level'
print('ANGLE-HOLD RECOVERY OK')
