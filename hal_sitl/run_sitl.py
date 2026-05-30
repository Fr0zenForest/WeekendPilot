"""Run a closed-loop SITL: scripted RC -> core -> JSBSim, log state to CSV."""
import jsbsim, os, csv, argparse, glob, sys
from jsbsim_bridge import CoreController, read_imu, write_servos

JSBSIM_ROOT = 'C:/Repository/jsbsim'
BUILD_DIR = os.path.join(os.path.dirname(__file__), '..', 'build')


def resolve_dll():
    """Find the core C API DLL. MinGW builds prefix with 'lib', so try both
    the plain and lib-prefixed names, then fall back to a glob."""
    candidates = ['wp_core_capi.dll', 'libwp_core_capi.dll']
    for name in candidates:
        path = os.path.join(BUILD_DIR, name)
        if os.path.exists(path):
            return os.path.abspath(path)
    hits = glob.glob(os.path.join(BUILD_DIR, '*wp_core_capi*.dll'))
    if hits:
        return os.path.abspath(hits[0])
    raise FileNotFoundError(
        'core C API DLL not found in %s (looked for %s and *wp_core_capi*.dll)'
        % (os.path.abspath(BUILD_DIR), candidates))


def make_fdm(dt):
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_aircraft_path(JSBSIM_ROOT + '/aircraft')
    fdm.set_engine_path(JSBSIM_ROOT + '/engine')
    fdm.set_systems_path(JSBSIM_ROOT + '/systems')
    fdm.load_model('c172p')
    fdm.set_dt(dt)
    fdm.set_property_value('ic/h-agl-ft', 1000)
    fdm.set_property_value('ic/vc-kts', 90)
    fdm.set_property_value('ic/gamma-deg', 0)
    fdm.run_ic()
    return fdm


def rc_for_time(t):
    """Scripted RC. channels us[1000,2000]. ch0 roll, ch1 pitch, ch2 throttle,
       ch3 yaw, ch4 mode, ch5 gain. Sticks centered, Angle mode at full gain so
       the stabilizer holds wings level and recovers from a disturbance."""
    ch = [1500] * 16
    ch[2] = 1700       # throttle
    ch[4] = 1500       # mode = Angle
    ch[5] = 2000       # gain 100%
    return ch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dt', type=float, default=0.01)
    ap.add_argument('--secs', type=float, default=10.0)
    ap.add_argument('--out', default='sitl_log.csv')
    args = ap.parse_args()

    dll = resolve_dll()
    print('using DLL', dll)
    fdm = make_fdm(args.dt)
    core = CoreController(dll)

    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t','roll','pitch','yaw','alt_ft','vc_kts','srv0','srv1','srv2','srv3'])
        t = 0.0
        while t < args.secs:
            imu6 = read_imu(fdm)
            alt_m = fdm.get_property_value('position/h-agl-ft') * 0.3048
            servos = core.update(rc_for_time(t), imu6, alt_m, 1, args.dt, 1)
            write_servos(fdm, servos)
            # Roll disturbance at t~3.0s, applied AFTER the controller writes its
            # servos but BEFORE fdm.run(), so the dynamics propagate it and the
            # controller then has to counteract the induced bank.
            #
            # NOTE: the originally-specified method (set 'attitude/phi-rad'
            # +0.35 directly) is a verified NO-OP in JSBSim 1.3.1 -- phi-rad and
            # p-rad_sec are read-only outputs recomputed from the EOM each step,
            # so setting them does nothing. The realizable equivalent of a ~20deg
            # attitude kick is to override the aileron for a short window, which
            # builds a genuine bank the stabilizer must recover from.
            if 3.0 <= t < 3.4:
                fdm.set_property_value('fcs/aileron-cmd-norm', 0.8)  # roll kick
            fdm.run()
            t = fdm.get_sim_time()
            w.writerow([round(t,3),
                        round(fdm.get_property_value('attitude/phi-deg'),2),
                        round(fdm.get_property_value('attitude/theta-deg'),2),
                        round(fdm.get_property_value('attitude/psi-deg'),2),
                        round(fdm.get_property_value('position/h-agl-ft'),1),
                        round(fdm.get_property_value('velocities/vc-kts'),1),
                        servos[0], servos[1], servos[2], servos[3]])
    print('wrote', args.out)


if __name__ == '__main__':
    main()
