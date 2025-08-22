# Spike v3 Python Robot Control Script
# Developed by Andy Pethan / GPT5 August 2025

from hub import port, light_matrix, motion_sensor, button, light
import runloop
import motor
import color
import color_sensor

# ====== Debug switches ======
DEBUG_ON = False
DBG_CREEP_SAMPLE_MS = 10
DBG_PRINT_THRESHOLDS_AT_START = True
DBG_PRINT_ON_HIT = True
DBG_PRINT_ON_MISS = True
DBG_PRINT_SENSOR_ERRORS = True
DBG_KEEP_SIMPLE = False

# ====== Motion tunables ======
TICK_MS = 10
BASE_VEL = 420
CREEP_VEL = 190
MAX_VEL_STRAIGHT = 520
MAX_VEL_TURN = 385
MAX_VEL_ARC = 420
SLEW_UP_DPS2 = 1500
SLEW_DOWN_DPS2 = 850
TURN_SLEW_DOWN_DPS2 = 2600
DECEL_WINDOW_ROT = 0.30
DECEL_WINDOW_DEG = 22.0

# Heading control
ARC_YAW_KP = 0.16
YAW_KP = 1.65
YAW_KD = 0.060
YAW_D_ALPHA = 0.35
TURN_MIN_VEL = 110
TURN_DEADBAND_DEG = 0.4
TURN_TOL_REENTRY_DEG = 3.0
TURN_ALLOW_REVERSAL_DEG = 12.0
TURN_SETTLE_MS = 220

# Yaw axis selection — SPIKE v3 tilt_angles()[0] as yaw on flat plane
YAW_AXIS_IDX = 0
YAW_SIGN = 1

# ====== Reflectance thresholds ======
ABS_BLACK_FIRE_MAX = 0.22        # primary detect threshold during creep
ABS_BLACK_CLEAR    = 0.27        # hysteresis clear for creep-only logic

# Derivative-drop assist (creep-only)
USE_DERIV_EDGE = True
DERIV_EDGE_MIN_DROP = 0.22
DERIV_EDGE_WINDOW_MS = 12

# Creep polling and debounce (creep-only)
SAMPLE_MS_CREEP = 2
MIN_BLACK_SAMPLES = 1
BLACK_CONFIRM_MS = 8

# ====== Back-off and final alignment (anti-bounce tuned) ======
BACKOFF_ROT = 0.120              # longer back-off to fully clear line
BACKOFF_VEL_DPS = 60             # very slow reverse to avoid re-entry overshoot
ALIGN_CREEP_DPS = 26             # ultra-slow forward creep for final alignment
FINAL_ALIGN_BLACK_THRESH = 0.30  # NO HYSTERESIS: stop a wheel on first hint of black (normalized <= 0.30)
ALIGN_SETTLE_MS = 60             # pause after both wheels stop to mechanically settle
ALIGN_TIMEOUT_MS = 7000

# Brake taps disabled to prevent bounce
USE_BRAKE_TAP_HIT = False
USE_BRAKE_TAP_ALIGN = False
BRAKE_TAP_DPS = 70
BRAKE_TAP_MS = 18

# Watchdogs
STALL_POS_EPS_ROT = 0.001
STALL_WINDOW_MS = 700
GLOBAL_MOVE_TIMEOUT_MS = 12000

# For helper predicates
DEFAULT_BLACK_FRAC = ABS_BLACK_FIRE_MAX
DEFAULT_WHITE_FRAC = 0.90

# ====== Utilities ======
def _resolve_port(p):
    if isinstance(p, str):
        return getattr(port, p)
    return p

def _normalize_dir(d):
    name = (str(d) if d is not None else "").strip().lower()
    if name in ("l", "left"):  return +1
    if name in ("r", "right"): return -1
    print("[direction] expected 'left' or 'right'; got:", d)
    return 0

async def wait_button(side='left', poll_ms=10):
    name = (side or 'left').lower()
    targets = [button.LEFT] if name == 'left' else [button.RIGHT]
    while True:
        for t in targets:
            if button.pressed(t):
                return
        await runloop.sleep_ms(int(poll_ms))

def set_status(c):
    col = getattr(color, str(c).upper(), c)
    light.color(light.POWER, col)

def _yaw_deg_raw():
    v = motion_sensor.tilt_angles()
    return YAW_SIGN * 0.1 * float(v[YAW_AXIS_IDX])

def yaw_raw():
    return _yaw_deg_raw()

# Simple, consistent report helper used by public methods
_def_round_keys = {"left_rot", "right_rot", "avg_rot", "yaw_deg_rel", "yaw_deg_world", "deg", "turned_rel", "world", "rotations", "rotations_to_line"}

def _fmt_field(k, v):
    try:
        if isinstance(v, float) and (k in _def_round_keys):
            return round(v, 3)
    except Exception:
        pass
    return v

def _report(tag, **fields):
    parts = []
    for k, v in fields.items():
        parts.append(f"{k}={_fmt_field(k, v)}")
    print(f"[{tag}] " + " ".join(parts))

def _clip(v, lo=-1000, hi=1000):
    if v < lo: return lo
    if v > hi: return hi
    return v

async def _sleep_ms(ms):
    await runloop.sleep_ms(int(ms))

def _ramp_mag(current, target, dt_ms, up_rate_dps2, down_rate_dps2):
    if dt_ms <= 0: return target
    delta = target - current
    max_step = (up_rate_dps2 if delta > 0 else down_rate_dps2) * (dt_ms / 1000.0)
    if abs(delta) <= abs(max_step): return target
    return current + (max_step if delta > 0 else -max_step)

def _decel_target_mag(remaining, base_vel, creep_vel, window):
    if remaining >= window: return base_vel
    frac = max(0.0, min(1.0, remaining / window))
    return creep_vel + (base_vel - creep_vel) * frac

def _run_or_stop(p, speed, min_abs=1):
    s = int(speed)
    if -min_abs < s < min_abs:
        motor.stop(p)
    else:
        motor.run(p, s)

def _wrap_diff(cur, start):
    d = cur - start
    if d < -180.0: d += 360.0
    elif d > 180.0: d -= 360.0
    return d

# ====== Drive Library ======
class DriveLib:
    def __init__(self, left_motor_port, right_motor_port, left_sensor_port, right_sensor_port,
                 track_width_cm=10.0, black_threshold=DEFAULT_BLACK_FRAC, white_threshold=DEFAULT_WHITE_FRAC,
                 left_polarity=-1, right_polarity=1):
        self.left = _resolve_port(left_motor_port)
        self.right = _resolve_port(right_motor_port)
        self.csL = _resolve_port(left_sensor_port)
        self.csR = _resolve_port(right_sensor_port)
        self.track_width_cm = float(track_width_cm)
        self.BLACK_FRAC = float(black_threshold)
        self.WHITE_FRAC = float(white_threshold)
        self.sL = 1 if left_polarity >= 0 else -1
        self.sR = 1 if right_polarity >= 0 else -1
        self._ready = False
        self._yaw0 = 0.0
        self._dbg_sensor_error_once = False

    # Motor sign helpers
    def _L(self, val):
        return self.sL * val
    def _R(self, val):
        return self.sR * val

    # ====== Sensor helpers ======
    def _ref_raw(self, p):
        try:
            return int(color_sensor.reflection(p))
        except Exception as e:
            if DBG_PRINT_SENSOR_ERRORS and not self._dbg_sensor_error_once:
                print("[ref_raw] EXC reading sensor:", e)
                self._dbg_sensor_error_once = True
            return 100

    def _ref_norm(self, p):
        return max(0.0, min(1.0, self._ref_raw(p) / 100.0))

    def _refs_now(self):
        l_raw = self._ref_raw(self.csL)
        r_raw = self._ref_raw(self.csR)
        l = max(0.0, min(1.0, l_raw / 100.0))
        r = max(0.0, min(1.0, r_raw / 100.0))
        return l, r, 0.5 * (l + r), l_raw, r_raw

    def _on_white_port(self, p): return self._ref_norm(p) >= self.WHITE_FRAC

    # ====== Odom / readiness ======
    def _avg_rot_abs(self):
        Ld = abs(motor.relative_position(self.left))
        Rd = abs(motor.relative_position(self.right))
        return (Ld + Rd) / 720.0

    def _fwd_rot_signed(self):
        Ld = motor.relative_position(self.left)  * (1 if self.sL > 0 else -1)
        Rd = motor.relative_position(self.right) * (1 if self.sR > 0 else -1)
        return (Ld + Rd) / (2.0 * 360.0)

    def _heading_world(self): return _yaw_deg_raw()
    def _heading_rel(self):   return _wrap_diff(_yaw_deg_raw(), self._yaw0)

    async def _ensure_ready(self):
        if self._ready: return
        for _ in range(10):
            try:
                motor.stop(self.left); motor.stop(self.right)
                motor.reset_relative_position(self.left, 0)
                motor.reset_relative_position(self.right, 0)
                self._ready = True
                return
            except OSError:
                await _sleep_ms(50)
        print("[DriveLib] WARNING: motor init retries exhausted; continuing anyway.")
        self._ready = True

    async def _prep_drive_move(self, reset_enc=False):
        await self._ensure_ready()
        if reset_enc:
            motor.reset_relative_position(self.left, 0)
            motor.reset_relative_position(self.right, 0)
        self._yaw0 = _yaw_deg_raw()
        await _sleep_ms(1)

    async def stop_hard(self):
        await self._ensure_ready()
        motor.stop(self.left); motor.stop(self.right)
        await _sleep_ms(10)

    def _odom(self):
        Ld = float(motor.relative_position(self.left))
        Rd = float(motor.relative_position(self.right))
        return {
            'left_deg': Ld, 'right_deg': Rd,
            'left_rot': Ld / 360.0, 'right_rot': Rd / 360.0,
            'avg_rot': (abs(Ld) + abs(Rd)) / 720.0,
            'yaw_deg_world': _yaw_deg_raw(),
            'yaw_deg_rel': self._heading_rel(),
        }

    # ====== Derivative ring (for fast drop detection during creep) ======
    def _ring_init(self, capacity=8):
        n = int(max(3, capacity))
        self._ring = [1.0] * n
        self._ring_idx = 0
        self._ring_len = n

    def _ring_push(self, v):
        self._ring[self._ring_idx] = v
        self._ring_idx = (self._ring_idx + 1) % self._ring_len

    def _ring_max_drop(self, now):
        worst = 0.0
        for past in self._ring:
            d = past - now
            if d > worst: worst = d
        return worst

    # ====== Straight with gyro ======
    async def straight(self, target_total_rot, base_velocity=None, Kp=4.0, report=False):
        await self._prep_drive_move(reset_enc=False)
        direction = 1 if target_total_rot >= 0 else -1
        target = max(0.0, abs(target_total_rot))
        base = BASE_VEL if base_velocity is None else float(base_velocity)
        creep = CREEP_VEL
        mag = 0.0
        elapsed = 0
        while True:
            traveled = self._avg_rot_abs()
            if traveled >= target: break
            remaining = target - traveled
            target_mag = _decel_target_mag(remaining, base, creep, DECEL_WINDOW_ROT)
            mag = _ramp_mag(mag, target_mag, TICK_MS, SLEW_UP_DPS2, SLEW_DOWN_DPS2)
            yaw_rel = self._heading_rel()
            turn = Kp * (0 - yaw_rel)
            l = _clip(direction * (mag - turn))
            r = _clip(direction * (mag + turn))
            _run_or_stop(self.left,  self._L(l))
            _run_or_stop(self.right, self._R(r))
            await _sleep_ms(TICK_MS)
            elapsed += TICK_MS
            if elapsed >= GLOBAL_MOVE_TIMEOUT_MS:
                print("[straight] TIMEOUT")
                break
        await self.stop_hard()
        if report or DEBUG_ON:
            o = self._odom()
            _report('straight', left_rot=o['left_rot'], right_rot=o['right_rot'],
                    avg_rot=o['avg_rot'], yaw_deg_rel=o['yaw_deg_rel'])

    # ====== Small distance helpers ======
    async def _drive_delta_rot(self, delta_rot, velocity=None):
        base = BASE_VEL if velocity is None else float(velocity)
        creep = CREEP_VEL
        mag = 0.0
        start = self._fwd_rot_signed()
        target = start + float(delta_rot)
        elapsed = 0
        last = start
        while True:
            cur = self._fwd_rot_signed()
            rem = target - cur
            if abs(rem) <= 0.0015: break
            target_mag = _decel_target_mag(abs(rem), base, creep, DECEL_WINDOW_ROT)
            mag = _ramp_mag(mag, target_mag, TICK_MS, SLEW_UP_DPS2, SLEW_DOWN_DPS2)
            cmd = _clip(mag) * (1 if rem >= 0 else -1)
            if velocity is not None:
                cmd = _clip(float(velocity)) * (1 if rem >= 0 else -1)
            _run_or_stop(self.left,  self._L(cmd))
            _run_or_stop(self.right, self._R(cmd))
            await _sleep_ms(TICK_MS)
            elapsed += TICK_MS
            if abs(cur - last) < STALL_POS_EPS_ROT:
                if elapsed >= STALL_WINDOW_MS:
                    print("[_drive_delta_rot] STALL watchdog")
                    break
            else:
                elapsed = 0
            last = cur
        await self.stop_hard()

    async def _drive_to_rot_abs(self, target_rot, velocity=None):
        cur = self._fwd_rot_signed()
        await self._drive_delta_rot(target_rot - cur, velocity=velocity)

    # ====== Independent ultra-slow squaring with NO HYSTERESIS and anti-bounce ======
    async def _square_independent_creep(self, creep_dps=ALIGN_CREEP_DPS):
        v = max(15, min(int(creep_dps), 60))
        left_done  = (self._ref_norm(self.csL) <= FINAL_ALIGN_BLACK_THRESH)
        right_done = (self._ref_norm(self.csR) <= FINAL_ALIGN_BLACK_THRESH)

        if not left_done:
            _run_or_stop(self.left,  self._L(v))
        else:
            motor.stop(self.left)
        if not right_done:
            _run_or_stop(self.right, self._R(v))
        else:
            motor.stop(self.right)

        elapsed = 0
        while True:
            if not left_done and self._ref_norm(self.csL) <= FINAL_ALIGN_BLACK_THRESH:
                motor.stop(self.left)
                left_done = True
            if not right_done and self._ref_norm(self.csR) <= FINAL_ALIGN_BLACK_THRESH:
                motor.stop(self.right)
                right_done = True
            if left_done and right_done:
                break
            await _sleep_ms(3)
            elapsed += 3
            if elapsed >= ALIGN_TIMEOUT_MS:
                print("[square_independent_creep] TIMEOUT")
                break

        await _sleep_ms(ALIGN_SETTLE_MS)
        if USE_BRAKE_TAP_ALIGN:
            _run_or_stop(self.left,  self._L(-BRAKE_TAP_DPS))
            _run_or_stop(self.right, self._R(-BRAKE_TAP_DPS))
            await _sleep_ms(BRAKE_TAP_MS)
        await self.stop_hard()
        ok = (self._ref_norm(self.csL) <= FINAL_ALIGN_BLACK_THRESH) and (self._ref_norm(self.csR) <= FINAL_ALIGN_BLACK_THRESH)
        if DEBUG_ON:
            l, r, _, lraw, rraw = self._refs_now()
            print("[square_independent_creep] ok=", ok, " L=", round(l,3), "(", lraw, ") R=", round(r,3), "(", rraw, ")",
                  " thr_final≤", FINAL_ALIGN_BLACK_THRESH)
        return ok

    # ====== Optional edge-centering from BLACK (unchanged, but caller can skip) ======
    async def _edge_center_from_black(self, back_limit=0.30, fwd_limit=0.20, speed=150):
        motor.reset_relative_position(self.left, 0)
        motor.reset_relative_position(self.right, 0)
        cond_white_both = lambda: (self._on_white_port(self.csL) and self._on_white_port(self.csR))
        v = max(60, min(int(speed), 180))
        _run_or_stop(self.left,  self._L(-v))
        _run_or_stop(self.right, self._R(-v))
        hit = False
        elapsed = 0
        while True:
            pos = self._fwd_rot_signed()
            if pos <= -back_limit: break
            if cond_white_both():
                hit = True
                break
            await _sleep_ms(5)
            elapsed += 5
            if elapsed >= GLOBAL_MOVE_TIMEOUT_MS:
                print("[edge_center] TIMEOUT backing to white")
                break
        await self.stop_hard()
        if not hit:
            print("[edge_center] WARNING: no white bracket found within back_limit")
        posW = self._fwd_rot_signed() if hit else -back_limit
        posB = 0.0
        for _ in range(8):
            mid = 0.5 * (posW + posB)
            await self._drive_to_rot_abs(mid, velocity=200)
            l, r, avg, lraw, rraw = self._refs_now()
            if avg >= 0.5:
                posW = mid
            else:
                posB = mid
        best_cost = abs(self._ref_norm(self.csL) - 0.5) + abs(self._ref_norm(self.csR) - 0.5)
        best_delta = 0.0
        for step in (0.02, -0.04, 0.02):
            await self._drive_delta_rot(step, velocity=180)
            cost = abs(self._ref_norm(self.csL) - 0.5) + abs(self._ref_norm(self.csR) - 0.5)
            if cost < best_cost:
                best_cost = cost
                best_delta += step
            else:
                await self._drive_delta_rot(-step, velocity=180)
        if DEBUG_ON:
            print("[edge_center] micro delta=", round(best_delta,4), " cost≈", round(best_cost,3))
        return True, (0.5 * (posW + posB) + best_delta)

    # ====== Turns (forced direction) ======
    async def _turn_to_heading_dir(self, target_deg, s, turn_velocity=None, tol_deg=1.0):
        await self._prep_drive_move(reset_enc=False)
        self._yaw0 = _yaw_deg_raw()
        base = MAX_VEL_TURN if turn_velocity is None else float(turn_velocity)
        mag = 0.0
        last_rel = None
        rate_ema = 0.0
        settled_ms = 0
        in_hold = False
        crossed = False
        signed_target = s * float(target_deg)
        motor.reset_relative_position(self.left, 0)
        motor.reset_relative_position(self.right, 0)
        kick_p = 190 if s > 0 else -190
        _run_or_stop(self.left,  self._L(-kick_p))
        _run_or_stop(self.right, self._R( kick_p))
        await _sleep_ms(180)
        Ld = abs(motor.relative_position(self.left)); Rd = abs(motor.relative_position(self.right))
        if Ld < 3 and Rd < 3:
            kick_p = 240 if s > 0 else -240
            _run_or_stop(self.left,  self._L(-kick_p))
            _run_or_stop(self.right, self._R( kick_p))
            await _sleep_ms(160)
        motor.stop(self.left); motor.stop(self.right)
        await _sleep_ms(15)
        elapsed_global = 0
        while True:
            y_rel = self._heading_rel()
            err = signed_target - y_rel
            if not crossed and err <= 0: crossed = True
            dyaw_dt = 0.0 if last_rel is None else (y_rel - last_rel) / (TICK_MS / 1000.0)
            last_rel = y_rel
            rate_ema = (1.0 - YAW_D_ALPHA) * rate_ema + YAW_D_ALPHA * dyaw_dt
            if abs(err) <= max(tol_deg, TURN_DEADBAND_DEG):
                in_hold = True
                _run_or_stop(self.left, 0); _run_or_stop(self.right, 0)
                settled_ms += TICK_MS
                if settled_ms >= TURN_SETTLE_MS: break
                await _sleep_ms(TICK_MS); elapsed_global += TICK_MS
                if elapsed_global >= GLOBAL_MOVE_TIMEOUT_MS: print("[_turn_to_heading_dir] TIMEOUT hold"); break
                continue
            if in_hold and abs(err) <= TURN_TOL_REENTRY_DEG:
                _run_or_stop(self.left, 0); _run_or_stop(self.right, 0)
                await _sleep_ms(TICK_MS); elapsed_global += TICK_MS
                if elapsed_global >= GLOBAL_MOVE_TIMEOUT_MS: print("[_turn_to_heading_dir] TIMEOUT reentry"); break
                continue
            in_hold = False; settled_ms = 0
            cmd = YAW_KP * err - YAW_KD * rate_ema
            scale = min(1.0, abs(err) / 45.0)
            local_cap = max(135.0, base * scale)
            target_mag = min(local_cap, abs(cmd))
            floor = 72.0 + 6.0 * min(5.0, abs(err))
            if abs(rate_ema) < 1.5 and abs(err) > 3.0:
                target_mag = max(target_mag, float(TURN_MIN_VEL))
            mag = _ramp_mag(mag, target_mag, TICK_MS, SLEW_UP_DPS2, TURN_SLEW_DOWN_DPS2)
            sign = 1 if cmd > 0 else -1
            p = sign * mag
            _run_or_stop(self.left,  self._L(-p))
            _run_or_stop(self.right, self._R( p))
            await _sleep_ms(TICK_MS); elapsed_global += TICK_MS
            if elapsed_global >= GLOBAL_MOVE_TIMEOUT_MS:
                print("[_turn_to_heading_dir] TIMEOUT turning")
                break
        await self.stop_hard()

    async def turn(self, direction, degrees, turn_velocity=None, tol_deg=1.0, report=False):
        d = float(degrees)
        if d < 0:
            print("[turn] degrees must be >= 0; got:", degrees); return
        if d > 180.0:
            print("[turn] degrees > 180 not supported; using 180."); d = 180.0
        s = _normalize_dir(direction)
        if s == 0: return
        await self._turn_to_heading_dir(d, s, turn_velocity=turn_velocity, tol_deg=tol_deg)
        if report or DEBUG_ON:
            _report('turn', dir=direction, deg=degrees,
                    turned_rel=self._heading_rel(), world=self._heading_world())

    # ====== Arcs (kept; arc_until_line removed by request) ======
    async def arc(self, direction, degrees, radius_cm, base_velocity=None, use_gyro_trim=True, report=False):
        await self._prep_drive_move(reset_enc=True); self._yaw0 = _yaw_deg_raw()
        R = float(radius_cm); W = self.track_width_cm
        if R <= 0 or R <= W / 2.0:
            print("[arc] invalid radius"); return False
        d = float(degrees)
        if d < 0:
            print("[arc] degrees must be >= 0; got:", degrees); return False
        if d > 180.0: d = 180.0
        s = _normalize_dir(direction)
        if s == 0: return False
        ccw = (s > 0)
        R_left, R_right = (R - W/2.0, R + W/2.0) if ccw else (R + W/2.0, R - W/2.0)
        target_ratio = R_left / R_right
        base = BASE_VEL if base_velocity is None else float(base_velocity)
        creep = CREEP_VEL; mag = 0.0; target_abs = abs(d); elapsed = 0
        while True:
            y_rel = self._heading_rel()
            done = (abs(y_rel) >= target_abs) and ((y_rel > 0) == ccw or abs(y_rel) >= target_abs + 1.0)
            if done: break
            remaining = max(0.0, target_abs - abs(y_rel))
            target_mag = _decel_target_mag(remaining, base, creep, DECEL_WINDOW_DEG)
            mag = _ramp_mag(mag, target_mag, TICK_MS, SLEW_UP_DPS2, SLEW_DOWN_DPS2)
            pR = _clip(mag); pL = _clip(mag * target_ratio)
            if use_gyro_trim:
                yaw_err = (target_abs if ccw else -target_abs) - y_rel
                trim = _clip(ARC_YAW_KP * yaw_err, lo=-0.3*mag, hi=0.3*mag)
                if ccw: pL = _clip(pL - trim); pR = _clip(pR + trim)
                else:   pL = _clip(pL + trim); pR = _clip(pR - trim)
            _run_or_stop(self.left,  self._L(pL))
            _run_or_stop(self.right, self._R(pR))
            await _sleep_ms(TICK_MS); elapsed += TICK_MS
            if elapsed >= GLOBAL_MOVE_TIMEOUT_MS: print("[arc] TIMEOUT"); break
        await self.stop_hard()
        ok = True
        if report or DEBUG_ON:
            _report('arc', ok=ok, dir=direction, deg=degrees,
                    turned_rel=self._heading_rel(), world=self._heading_world())
        return ok

    # ====== Core: straight → creep detect (black-only) → hard-stop → long very-slow back-off → independent ULTRA-SLOW alignment → optional center ======
    async def align(self, target_rotations, base_velocity=None, creep_velocity=None,
                                        Kp=4.0, center_after_square=False, report=False):
        if target_rotations <= 0:
            if report or DEBUG_ON:
                _report('align', found=False, first_hit=None, rotations_to_line=0.0)
            print("[align] target must be > 0 rotations; got:", target_rotations)
            return {'found': False, 'first_hit': None, 'rotations_to_line': 0.0}

        target = abs(target_rotations)
        min_rot = max(0.0, target - 0.30)
        max_rot = target + 0.20

        await self._prep_drive_move(reset_enc=True)
        if DBG_PRINT_THRESHOLDS_AT_START and DEBUG_ON:
            l, r, avg, lraw, rraw = self._refs_now()
            print("[align.start] thresholds abs=", ABS_BLACK_FIRE_MAX, "/", ABS_BLACK_CLEAR,
                  " deriv_drop>=", DERIV_EDGE_MIN_DROP if USE_DERIV_EDGE else "off",
                  " sample_ms=", SAMPLE_MS_CREEP, " creep_dbg_ms=", DBG_CREEP_SAMPLE_MS,
                  " final_align_thr≤", FINAL_ALIGN_BLACK_THRESH)
            print("[align.start] first reads L=", round(l,2), "(", lraw, ") R=", round(r,2), "(", rraw, ")")

        print("[align] target=", round(target,3), " window=[", round(min_rot,3), ",", round(max_rot,3), "]")
        await self.straight(min_rot, base_velocity=base_velocity, Kp=Kp)

        motor.reset_relative_position(self.left, 0)
        motor.reset_relative_position(self.right, 0)

        creep_cmd = float(CREEP_VEL if creep_velocity is None else float(creep_velocity))
        if creep_velocity is None:
            creep_cmd = min(creep_cmd, 160.0)
        _run_or_stop(self.left,  self._L(creep_cmd))
        _run_or_stop(self.right, self._R(creep_cmd))

        overshoot_guard_rot = max_rot - min_rot
        rot_per_sec = max(0.01, creep_cmd / 360.0)
        expected_ms = int(1000.0 * overshoot_guard_rot / rot_per_sec)
        max_creep_ms = min(6000, max(1200, int(3.0 * expected_ms)))

        first = None
        elapsed = 0
        still_ms = 0
        last_pos = self._fwd_rot_signed()
        L_black_cnt = 0; R_black_cnt = 0

        ring_cap = max(3, int(DERIV_EDGE_WINDOW_MS // max(1, SAMPLE_MS_CREEP)))
        self._ring_init(ring_cap)

        last_dbg_ms = -9999
        hit_latched = False

        while True:
            pos = self._fwd_rot_signed()
            if pos >= overshoot_guard_rot:
                if DEBUG_ON and DBG_PRINT_ON_MISS:
                    print("[creep] OVERSHOOT_GUARD pos=", round(pos,3), " guard=", round(overshoot_guard_rot,3))
                break

            l, r, avg, lraw, rraw = self._refs_now()
            self._ring_push(min(l, r))

            fire_side = None
            if l <= ABS_BLACK_FIRE_MAX: L_black_cnt += 1
            elif l > ABS_BLACK_CLEAR:   L_black_cnt = 0
            if r <= ABS_BLACK_FIRE_MAX: R_black_cnt += 1
            elif r > ABS_WHITE_CLEAR if False else ABS_BLACK_CLEAR:  # keep structure, no change in behavior
                R_black_cnt = 0
            if L_black_cnt >= MIN_BLACK_SAMPLES: fire_side = 'L'
            elif R_black_cnt >= MIN_BLACK_SAMPLES: fire_side = 'R'

            if USE_DERIV_EDGE and fire_side is None:
                drop = self._ring_max_drop(min(l, r))
                if drop >= DERIV_EDGE_MIN_DROP:
                    fire_side = 'L' if l <= r else 'R'
            else:
                drop = 0.0

            if DEBUG_ON and (elapsed - last_dbg_ms) >= DBG_CREEP_SAMPLE_MS:
                if DBG_KEEP_SIMPLE:
                    print("[creep] t=", elapsed, "ms pos=", round(pos,3), " L=", round(l,2), " R=", round(r,2))
                else:
                    print("[creep] t=", elapsed, "ms pos=", round(pos,3),
                          " L=", round(l,3), "(", lraw, ") R=", round(r,3), "(", rraw, ")",
                          " Lcnt=", L_black_cnt, " Rcnt=", R_black_cnt,
                          " drop=", round(drop,3),
                          " thr_abs≤", ABS_BLACK_FIRE_MAX, "/>", ABS_BLACK_CLEAR)
                last_dbg_ms = elapsed

            if fire_side is not None:
                confirm_elapsed = 0
                while confirm_elapsed < BLACK_CONFIRM_MS:
                    l2, r2, _, _, _ = self._refs_now()
                    if (fire_side == 'L' and l2 > ABS_BLACK_CLEAR) or (fire_side == 'R' and r2 > ABS_BLACK_CLEAR):
                        fire_side = None
                        break
                    await _sleep_ms(SAMPLE_MS_CREEP)
                    confirm_elapsed += SAMPLE_MS_CREEP
                    elapsed += SAMPLE_MS_CREEP
                if fire_side is not None:
                    first = fire_side
                    motor.stop(self.left); motor.stop(self.right)
                    await _sleep_ms(30)  # small settle before back-off
                    hit_latched = True
                    if DEBUG_ON and DBG_PRINT_ON_HIT:
                        print("[HIT]", first, "t=", elapsed, "ms pos=", round(pos,3),
                              " L=", round(l,3), " R=", round(r,3))
                    break

            if abs(pos - last_pos) < 0.001:
                still_ms += SAMPLE_MS_CREEP
            else:
                still_ms = 0
            last_pos = pos

            await _sleep_ms(SAMPLE_MS_CREEP)
            elapsed += SAMPLE_MS_CREEP
            if still_ms >= STALL_WINDOW_MS:
                print("[creep] STALL watchdog")
                break
            if elapsed >= max_creep_ms:
                if DEBUG_ON and DBG_PRINT_ON_MISS:
                    print("[creep] TIMEOUT window elapsed_ms=", elapsed, " expected_ms≈", expected_ms)
                break

        if USE_BRAKE_TAP_HIT and hit_latched:
            _run_or_stop(self.left,  self._L(-BRAKE_TAP_DPS))
            _run_or_stop(self.right, self._R(-BRAKE_TAP_DPS))
            await _sleep_ms(BRAKE_TAP_MS)
        await self.stop_hard()

        rot_to_line = max(0.0, self._fwd_rot_signed())
        if first is None:
            if report or DEBUG_ON:
                _report('align', found=False, first_hit=None, rotations_to_line=rot_to_line)
            print("[align] NO LINE FOUND within window. traveled=", round(rot_to_line,3), " rot")
            return {'found': False, 'first_hit': None, 'rotations_to_line': rot_to_line}

        # Long, very-slow back-off to fully clear edge before alignment; constant capped reverse
        await self._drive_delta_rot(-BACKOFF_ROT, velocity=BACKOFF_VEL_DPS)

        # Independent ULTRA-SLOW creep, no hysteresis: stop a wheel as soon as it sees black (<= FINAL_ALIGN_BLACK_THRESH)
        ok_sq = await self._square_independent_creep(creep_dps=ALIGN_CREEP_DPS)
        if not ok_sq:
            print("[align] WARNING: independent squaring failed (kept closest).")

        if not center_after_square:
            if report or DEBUG_ON:
                _report('align', found=ok_sq, first_hit=first, rotations_to_line=rot_to_line)
            return {'found': ok_sq, 'first_hit': first, 'rotations_to_line': rot_to_line}

        ok_center, delta = await self._edge_center_from_black(back_limit=0.30, fwd_limit=0.20, speed=150)
        if ok_center:
            print("[align] EDGE-CENTERED. Δrot≈", round(delta,4))
        else:
            print("[align] WARNING: edge-centering fell back.")
        if report or DEBUG_ON:
            _report('align', found=ok_center, first_hit=first, rotations_to_line=rot_to_line)
        return {'found': ok_center, 'first_hit': first, 'rotations_to_line': rot_to_line}

# ====== Single-axis Mechanism ======
class Mechanism:
    def __init__(self, p, hold_at_rest=True, decel_window_deg=90.0, creep_min_vel=120):
        self.p = _resolve_port(p)
        self.decel_window_deg = float(decel_window_deg)
        self.creep_min_vel = float(creep_min_vel)
        self.hold_at_rest = bool(hold_at_rest)
        self._ready = False

    async def _ensure_ready(self):
        if self._ready: return
        for _ in range(10):
            try:
                motor.stop(self.p)
                motor.reset_relative_position(self.p, 0)
                self._ready = True
                return
            except OSError:
                await _sleep_ms(50)
        print("[Mechanism] WARNING: motor init retries exhausted; continuing anyway.")
        self._ready = True

    async def _stop_end(self):
        await self._ensure_ready()
        motor.stop(self.p)
        await _sleep_ms(10)

    def position_deg(self):
        return float(motor.relative_position(self.p))

    async def rotate(self, direction, degrees, velocity=None, report=False):
        await self._ensure_ready()
        d = float(degrees)
        if d < 0:
            print("[Mechanism.rotate] degrees must be >= 0; got:", degrees); return
        s = _normalize_dir(direction)
        if s == 0:
            print("[Mechanism.rotate] expected direction 'left' or 'right'; got:", direction)
            return
        target = d
        base_vel = BASE_VEL if velocity is None else float(velocity)
        creep = self.creep_min_vel
        start_deg = self.position_deg()
        mag = 0.0
        elapsed = 0
        last = start_deg
        while True:
            cur_deg = self.position_deg()
            traveled = abs(cur_deg - start_deg)
            if traveled >= target: break
            remaining = target - traveled
            target_mag = _decel_target_mag(remaining, base_vel, creep, self.decel_window_deg)
            mag = _ramp_mag(mag, target_mag, TICK_MS, SLEW_UP_DPS2, SLEW_DOWN_DPS2)
            v = _clip(s * mag)
            _run_or_stop(self.p, v)
            await _sleep_ms(TICK_MS)
            elapsed += TICK_MS
            if abs(cur_deg - last) < (STALL_POS_EPS_ROT * 360.0):
                if elapsed >= STALL_WINDOW_MS:
                    print("[Mechanism.rotate] STALL watchdog")
                    break
            else:
                elapsed = 0
            last = cur_deg
            if elapsed >= GLOBAL_MOVE_TIMEOUT_MS:
                print("[Mechanism.rotate] TIMEOUT")
                break
        await self._stop_end()
        if report or DEBUG_ON:
            end_deg = self.position_deg()
            _report('Mechanism.rotate', dir=direction, deg=degrees, start=start_deg, end=end_deg, moved=end_deg - start_deg)

    async def home_stall(self, direction=-1, velocity=350, min_delta_deg=1.0, window_ms=300,
                         timeout_ms=4000, zero_at_end=True, report=False):
        await self._ensure_ready()
        direction = 1 if direction >= 0 else -1
        vel = abs(float(velocity))
        elapsed = 0
        _run_or_stop(self.p, direction * vel, min_abs=0)
        win_start_deg = self.position_deg()
        win_elapsed = 0
        while elapsed < timeout_ms:
            await _sleep_ms(TICK_MS)
            elapsed += TICK_MS
            win_elapsed += TICK_MS
            cur_deg = self.position_deg()
            if win_elapsed >= window_ms:
                moved = abs(cur_deg - win_start_deg)
                if moved < float(min_delta_deg):
                    motor.stop(self.p)
                    await _sleep_ms(20)
                    if zero_at_end:
                        motor.reset_relative_position(self.p, 0)
                    await self._stop_end()
                    if DEBUG_ON:
                        print("[Mechanism.home_stall] homed after", elapsed, "ms; moved", round(moved, 2), "deg.")
                    if report or DEBUG_ON:
                        _report('Mechanism.home_stall', ok=True, elapsed_ms=elapsed, moved_deg=moved)
                    return True
                win_start_deg = cur_deg
                win_elapsed = 0
        motor.stop(self.p)
        await _sleep_ms(20)
        await self._stop_end()
        print("[Mechanism.home_stall] TIMEOUT — no stall detected.")
        if report or DEBUG_ON:
            _report('Mechanism.home_stall', ok=False, elapsed_ms=elapsed)
        return False