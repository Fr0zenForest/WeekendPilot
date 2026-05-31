"""WeekendPilot SITL bridge: JSBSim dynamics <-> C++ control core via ctypes."""
import ctypes, os, math, shutil

RAD2DEG = 57.29577951308232


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
            ctypes.POINTER(ctypes.c_float), ctypes.c_int,
            ctypes.c_float, ctypes.c_int,
            ctypes.c_float, ctypes.c_int,
            ctypes.POINTER(ctypes.c_uint16),
        ]
        self.lib.wp_controller_attitude.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_float)]
        self.handle = self.lib.wp_controller_create()

    def update(self, channels16, imu6, mag3, mag_valid, baro_alt_m, baro_valid, dt_s, link_ok):
        ch = (ctypes.c_uint16 * 16)(*channels16)
        imu = (ctypes.c_float * 6)(*imu6)
        mag = (ctypes.c_float * 3)(*mag3)
        out = (ctypes.c_uint16 * 8)()
        self.lib.wp_controller_update(self.handle, ch, imu,
                                      mag, int(mag_valid),
                                      ctypes.c_float(baro_alt_m), int(baro_valid),
                                      ctypes.c_float(dt_s), int(link_ok), out)
        return list(out)

    def attitude(self):
        rpy = (ctypes.c_float * 3)()
        self.lib.wp_controller_attitude(self.handle, rpy)
        return list(rpy)

    def __del__(self):
        if getattr(self, 'handle', None):
            self.lib.wp_controller_destroy(self.handle)


def read_imu(fdm):
    """Return [gx,gy,gz deg/s, ax,ay,az g] from JSBSim.

    JSBSim accelerations/Nx,Ny,Nz are load factors already expressed in g
    (Nz ~+1.0 in steady level flight), NOT ft/s^2.  The old code divided by
    32.174 which produced a bogus ~0.03 g gravity reference.

    Sign convention: JSBSim Nz is POSITIVE in level flight (~+1.0 g), which
    matches the AHRS expectation of accel_z = +1.0 for level (see
    test/test_ahrs/test_ahrs.cpp level_imu()).  No sign flip is needed.
    """
    gx = fdm.get_property_value('velocities/p-rad_sec') * RAD2DEG
    gy = fdm.get_property_value('velocities/q-rad_sec') * RAD2DEG
    gz = fdm.get_property_value('velocities/r-rad_sec') * RAD2DEG
    ax = fdm.get_property_value('accelerations/Nx')
    ay = fdm.get_property_value('accelerations/Ny')
    az = fdm.get_property_value('accelerations/Nz')
    return [gx, gy, gz, ax, ay, az]


# Earth magnetic field, normalized, northern-hemisphere inclination 60deg.
# NED frame: [north, east, down]. Same convention as test_ahrs synth_mag.
_INCL = math.radians(60.0)
_B_NED = (math.cos(_INCL), 0.0, math.sin(_INCL))


def synth_mag(fdm):
    """Synthesize a body-frame magnetometer from JSBSim truth attitude.
    JSBSim has no magnetic field, so rotate a fixed earth field through the
    truth DCM C_bn = Rx(phi)Ry(theta)Rz(psi). Returns [mx,my,mz] (unit-ish)."""
    phi = math.radians(fdm.get_property_value('attitude/phi-deg'))
    th  = math.radians(fdm.get_property_value('attitude/theta-deg'))
    psi = math.radians(fdm.get_property_value('attitude/psi-deg'))
    Bn, Be, Bd = _B_NED
    cph, sph = math.cos(phi), math.sin(phi)
    cth, sth = math.cos(th),  math.sin(th)
    cps, sps = math.cos(psi), math.sin(psi)
    # Rz(psi)
    x1 =  cps * Bn + sps * Be
    y1 = -sps * Bn + cps * Be
    z1 =  Bd
    # Ry(theta)
    x2 = cth * x1 - sth * z1
    y2 = y1
    z2 = sth * x1 + cth * z1
    # Rx(phi)
    mx = x2
    my = cph * y2 + sph * z2
    mz = -sph * y2 + cph * z2
    return [mx, my, mz]


def write_servos(fdm, servos8):
    """Map servo us[1000,2000] -> JSBSim cmd-norm.
       servo[0]=aileron, servo[1]=elevator, servo[2]=throttle, servo[3]=rudder."""
    def norm(us):
        return max(-1.0, min(1.0, (us - 1500) / 500.0))
    def unit(us):
        return max(0.0, min(1.0, (us - 1000) / 1000.0))
    fdm.set_property_value('fcs/aileron-cmd-norm',  norm(servos8[0]))
    # 升降舵反向：JSBSim 约定 elevator-cmd-norm>0 = 升降舵后缘下偏 = 低头；
    # 而本控制器约定 pitch 需求>0(servo>1500)= 抬头指令。两者符号相反，必须反接，
    # 否则俯仰内环正反馈发散（这是 SITL 里等价于真机的“舵机反向/舵面行程方向”设置）。
    fdm.set_property_value('fcs/elevator-cmd-norm', -norm(servos8[1]))
    fdm.set_property_value('fcs/throttle-cmd-norm[0]', unit(servos8[2]))
    fdm.set_property_value('fcs/rudder-cmd-norm',   norm(servos8[3]))
