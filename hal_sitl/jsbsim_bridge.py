"""WeekendPilot SITL bridge: JSBSim dynamics <-> C++ control core via ctypes."""
import ctypes, os, math, shutil

RAD2DEG = 57.29577951308232
FT_S2_TO_G = 1.0 / 32.174


def _ensure_runtime_on_path():
    """The MinGW-built core DLL depends on libstdc++-6.dll (and friends) which
    live next to g++, not on the Python process's DLL search path. Add that
    directory so ctypes.CDLL can resolve dependencies. Returns the dir handle
    (kept alive by the caller) or None."""
    candidates = []
    gpp = shutil.which('g++') or shutil.which('gcc')
    if gpp:
        candidates.append(os.path.dirname(gpp))
    candidates += ['C:/Software/mingw64/bin', 'C:/mingw64/bin']
    for d in candidates:
        if d and os.path.exists(os.path.join(d, 'libstdc++-6.dll')):
            try:
                return os.add_dll_directory(d)
            except (AttributeError, OSError):
                return None
    return None


class CoreController:
    def __init__(self, dll_path):
        # Keep the dll-directory handle alive for the lifetime of this object.
        self._dll_dir = _ensure_runtime_on_path()
        self.lib = ctypes.CDLL(dll_path)
        self.lib.wp_controller_create.restype = ctypes.c_void_p
        self.lib.wp_controller_destroy.argtypes = [ctypes.c_void_p]
        self.lib.wp_controller_update.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint16),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_float, ctypes.c_int,
            ctypes.c_float, ctypes.c_int,
            ctypes.POINTER(ctypes.c_uint16),
        ]
        self.handle = self.lib.wp_controller_create()

    def update(self, channels16, imu6, baro_alt_m, baro_valid, dt_s, link_ok):
        ch = (ctypes.c_uint16 * 16)(*channels16)
        imu = (ctypes.c_float * 6)(*imu6)
        out = (ctypes.c_uint16 * 8)()
        self.lib.wp_controller_update(self.handle, ch, imu,
                                      ctypes.c_float(baro_alt_m), int(baro_valid),
                                      ctypes.c_float(dt_s), int(link_ok), out)
        return list(out)

    def __del__(self):
        if getattr(self, 'handle', None):
            self.lib.wp_controller_destroy(self.handle)


def read_imu(fdm):
    """Return [gx,gy,gz deg/s, ax,ay,az g] from JSBSim."""
    gx = fdm.get_property_value('velocities/p-rad_sec') * RAD2DEG
    gy = fdm.get_property_value('velocities/q-rad_sec') * RAD2DEG
    gz = fdm.get_property_value('velocities/r-rad_sec') * RAD2DEG
    ax = fdm.get_property_value('accelerations/Nx') * FT_S2_TO_G
    ay = fdm.get_property_value('accelerations/Ny') * FT_S2_TO_G
    az = fdm.get_property_value('accelerations/Nz') * FT_S2_TO_G
    return [gx, gy, gz, ax, ay, az]


def write_servos(fdm, servos8):
    """Map servo us[1000,2000] -> JSBSim cmd-norm.
       servo[0]=aileron, servo[1]=elevator, servo[2]=throttle, servo[3]=rudder."""
    def norm(us):
        return max(-1.0, min(1.0, (us - 1500) / 500.0))
    def unit(us):
        return max(0.0, min(1.0, (us - 1000) / 1000.0))
    fdm.set_property_value('fcs/aileron-cmd-norm',  norm(servos8[0]))
    fdm.set_property_value('fcs/elevator-cmd-norm', norm(servos8[1]))
    fdm.set_property_value('fcs/throttle-cmd-norm[0]', unit(servos8[2]))
    fdm.set_property_value('fcs/rudder-cmd-norm',   norm(servos8[3]))
