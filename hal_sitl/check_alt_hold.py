"""Verify Alt-Hold recovers altitude toward the locked target after a pitch kick.
WARNING: SITL altitude is JSBSim truth (position/h-agl-ft), NOT a real barometer.
This proves the control loop converges; real-baro behavior is unverified."""
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else 'sitl_log.csv')))
def alt_at(t):
    best = min(rows, key=lambda r: abs(float(r['t']) - t))
    return float(best['alt_ft'])
lock = alt_at(2.5)                                   # 扰动前锁定高度
trough = min(alt_at(t/10.0) for t in range(36, 60))  # 3.6-6.0s 扰动后最低点
settled = alt_at(9.5)                                # 末段回收值
print('locked alt (ft):', round(lock,1))
print('trough after kick (ft):', round(trough,1))
print('settled alt at t=9.5s (ft):', round(settled,1))
dip = lock - trough
recover = lock - settled
assert dip > 1.0, 'no measurable disturbance — test invalid'
assert abs(recover) < dip * 0.6, 'Alt-Hold did not recover toward locked altitude'
print('ALT-HOLD RECOVERY OK')
