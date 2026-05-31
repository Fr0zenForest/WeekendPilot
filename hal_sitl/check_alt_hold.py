"""Verify Alt-Hold returns to and holds the locked altitude after a disturbance.
WARNING: SITL altitude is JSBSim truth (position/h-agl-ft), NOT a real barometer.
This proves the control loop converges; real-baro behavior is unverified.
Run with a >=30s log: python run_sitl.py --scenario althold --secs 30 --out sitl_alt.csv"""
import csv, sys, statistics
rows = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else 'sitl_log.csv')))

def alt_at(t):
    best = min(rows, key=lambda r: abs(float(r['t']) - t))
    return float(best['alt_ft'])

t_end = max(float(r['t']) for r in rows)
lock = alt_at(2.5)                          # 扰动前锁定高度 (kick 在 3.0-3.6s)
# 扰动后峰值偏离（任一方向；本场景 elevator 下压实际把机头推上去 -> 高度先上冲）
peak_dev = max(abs(alt_at(t / 10.0) - lock) for t in range(36, int(t_end * 10)))
# 末段收敛窗：最后 5 秒的均值偏差与峰峰值
tail = [float(r['alt_ft']) for r in rows if float(r['t']) >= t_end - 5.0]
settle_mean_dev = statistics.mean(tail) - lock
settle_spread = max(tail) - min(tail)

print('locked alt (ft):       ', round(lock, 1))
print('peak deviation (ft):   ', round(peak_dev, 1))
print('last-5s mean dev (ft): ', round(settle_mean_dev, 1))
print('last-5s spread (ft):   ', round(settle_spread, 1))

assert t_end >= 25.0, 'log too short — run with --secs 30 to see convergence'
assert peak_dev > 5.0, 'no measurable disturbance — test invalid'
# 收敛判据：末段回到锁定高度附近（均值偏差 < 峰值的 30%），且不再大幅振荡。
assert abs(settle_mean_dev) < peak_dev * 0.3, 'Alt-Hold did not converge back to target'
assert settle_spread < peak_dev * 0.5, 'Alt-Hold still oscillating at end of run'
print('ALT-HOLD CONVERGENCE OK')
