# =============================================================
# PROWL-R | raider_controller.py
# Role: Infiltrate enemy zone, capture red flag, return to base.
#       Two Raiders cooperate via JSON broadcast messages.
# Team: Oscar Chia, Blair Fenton, Lakshay
# =============================================================

from controller import Supervisor
from enum import Enum
import math, json, csv

# ── Constants ─────────────────────────────────────────────────
MAX_SPEED               = 6.28
OBSTACLE_THRESHOLD      = 80
STUCK_DISTANCE          = 0.02
STUCK_TIME_LIMIT        = 3.0
AVOIDANCE_LIMIT         = 3.0
ESCAPE_STEPS            = 20
RECOVERY_REVERSE_STEPS  = 30
RECOVERY_TURN_STEPS     = 45
WAYPOINT_TOLERANCE      = 0.08
COMM_TIMEOUT            = 5.0
WHEEL_RADIUS            = 0.0205
AXLE_LENGTH             = 0.052

FLAG_PIXEL_THRESHOLD    = 3     # red pixels to confirm flag in view
FLAG_CAPTURE_RADIUS     = 0.30  # must be within this distance of FLAG_POS to capture
BASE_ARRIVAL_RADIUS     = 0.20  # larger than WAYPOINT_TOLERANCE to absorb odometry drift on return
GUARD_PIXEL_THRESHOLD   = 20    # yellow pixels to trigger EVADE_GUARD
EVADE_GUARD_MIN_STEPS   = 15    # minimum timesteps to stay in EVADE_GUARD (anti-oscillation)
TAG_PIXEL_THRESHOLD     = 200   # yellow pixels to trigger respawn (~10% of 52×39 frame)
GUARD_SEEN_RATE         = 0.5   # min seconds between GUARD_SEEN broadcasts
HEARTBEAT_RATE          = 0.5   # seconds between HEARTBEAT broadcasts
GUARD_SIGHTING_TTL      = 5.0   # seconds before a sighting expires
GUARD_SIGHTING_RADIUS   = 0.3   # metres — waypoint cost-penalty radius
YIELD_DISTANCE          = 0.2   # metres — close-approach threshold
YIELD_STEPS             = 50    # timesteps lower-ID raider halts (~1 s at 32 ms step)
BID_TIMEOUT             = 1.0   # seconds to wait for teammate BID at round start

# World-coordinate landmarks (shared by both Raiders)
NORTH_APPROACH  = (0.6,  0.25)
SOUTH_APPROACH  = (0.6, -0.25)
FLAG_POS        = (0.85, 0.0)

PS_ANGLE = [
    math.radians(-10),  math.radians(-45),  math.radians(-90),  math.radians(-150),
    math.radians(150),  math.radians(90),   math.radians(45),   math.radians(10),
]


class State(Enum):
    AVOID_OBSTACLE = 1
    RECOVERY       = 2
    EVADE_GUARD    = 3
    RETURN_TO_BASE = 4
    SEEK_FLAG      = 5


# ── Robot & Sensor Initialisation ────────────────────────────
robot    = Supervisor()
timestep = int(robot.getBasicTimeStep())
dt       = timestep / 1000.0

left_motor  = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

proximity_sensors = []
for i in range(8):
    ps = robot.getDevice(f'ps{i}')
    ps.enable(timestep)
    proximity_sensors.append(ps)

left_encoder  = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
left_encoder.enable(timestep)
right_encoder.enable(timestep)

camera = robot.getDevice('camera')
camera.enable(timestep)
compass = robot.getDevice('compass')
compass.enable(timestep)
emitter  = robot.getDevice('emitter')
receiver = robot.getDevice('receiver')
receiver.enable(timestep)

camera_pixels = camera.getWidth() * camera.getHeight()  # computed at runtime

# ── Identity & Spawn ──────────────────────────────────────────
robot_name = robot.getName()
if 'raider_a' in robot_name:
    robot_id    = 0
    SPAWN_X, SPAWN_Y = -0.7,  0.2
else:
    robot_id    = 1
    SPAWN_X, SPAWN_Y = -0.7, -0.2

emitter.setChannel(1)
receiver.setChannel(1)

HOME_BASE = (SPAWN_X, SPAWN_Y)

# ── Agent State Variables ─────────────────────────────────────
current_state = State.SEEK_FLAG
prev_state    = None
has_flag      = False
solo_mode     = False
seek_phase    = 'approach'     # 'approach' | 'flag'
approach_wp   = NORTH_APPROACH if robot_id == 0 else SOUTH_APPROACH  # overwritten by auction

# Odometry
pose_x         = SPAWN_X
pose_y         = SPAWN_Y
pose_theta     = 0.0
prev_left_enc  = 0.0
prev_right_enc = 0.0

# Stuck / recovery
last_check_x    = pose_x
last_check_y    = pose_y
stuck_timer     = 0.0
avoidance_timer = 0.0
escape_counter  = 0
escape_turn_left = True
recovery_phase   = 'reverse'
recovery_counter = 0

sim_time             = 0.0
last_heartbeat_time  = 0.0
last_hb_broadcast    = 0.0
last_guard_broadcast = 0.0
yield_counter        = 0
evade_guard_steps    = 0   # counts down after EVADE_GUARD triggers; holds state until 0
escort_mode          = False  # True when teammate has grabbed the flag — return to base
mission_done         = False  # True after arriving at base in escort or flag-carry mode

# Teammate state (updated via HEARTBEAT)
teammate_pos      = (0.0, 0.0)
teammate_has_flag = False

# Guard sightings: list of {"pos": (x,y), "t": float}
guard_sightings = []

# ── CSV Logging ───────────────────────────────────────────────
_log_path = f'raider_{robot_id}_mission.csv'
_log_file = open(_log_path, 'w', newline='')
_log      = csv.writer(_log_file)
_log.writerow(['sim_time', 'event', 'data'])


def log_csv(event, data=''):
    _log.writerow([f'{sim_time:.2f}', event, str(data)])
    _log_file.flush()


# ── Odometry ─────────────────────────────────────────────────
def update_odometry():
    global pose_x, pose_y, pose_theta, prev_left_enc, prev_right_enc
    l = left_encoder.getValue()
    r = right_encoder.getValue()
    dl = (l - prev_left_enc)  * WHEEL_RADIUS
    dr = (r - prev_right_enc) * WHEEL_RADIUS
    dd = (dl + dr) / 2.0
    h         = get_heading()
    pose_theta = math.atan2(math.sin(h), math.cos(h))
    pose_x    += dd * math.cos(pose_theta)
    pose_y    += dd * math.sin(pose_theta)
    prev_left_enc  = l
    prev_right_enc = r


def reset_pose_to(x, y, theta):
    global pose_x, pose_y, pose_theta
    pose_x, pose_y, pose_theta = x, y, theta


# ── Navigation ────────────────────────────────────────────────
def steer_toward(tx, ty):
    angle = math.atan2(ty - pose_y, tx - pose_x)
    h     = math.atan2(math.sin(get_heading()), math.cos(get_heading()))
    err   = math.atan2(math.sin(angle - h), math.cos(angle - h))
    ls    = max(-MAX_SPEED, min(MAX_SPEED, MAX_SPEED - err * 2.0))
    rs    = max(-MAX_SPEED, min(MAX_SPEED, MAX_SPEED + err * 2.0))
    left_motor.setVelocity(ls)
    right_motor.setVelocity(rs)


def distance_to(tx, ty):
    return math.sqrt((tx - pose_x)**2 + (ty - pose_y)**2)


def set_wheel_speeds(l, r):
    left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, l)))
    right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, r)))


# ── Compass ───────────────────────────────────────────────────
def get_heading():
    v = compass.getValues()
    h = math.atan2(v[0], v[1])
    return h if h >= 0 else h + 2 * math.pi


def turn_180(speed=2.0, tol=0.02):
    target = (get_heading() + math.pi) % (2 * math.pi)
    left_motor.setVelocity(-speed)
    right_motor.setVelocity(speed)
    while robot.step(timestep) != -1:
        err = math.atan2(math.sin(target - get_heading()), math.cos(target - get_heading()))
        if abs(err) < tol:
            break
        if abs(err) < 0.3:
            s = max(0.3, speed * abs(err) / 0.3)
            left_motor.setVelocity(-s)
            right_motor.setVelocity(s)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)


def do_bounce():
    global pose_theta, prev_left_enc, prev_right_enc
    steps = int(0.3 / dt)
    for _ in range(steps):
        left_motor.setVelocity(-MAX_SPEED * 0.5)
        right_motor.setVelocity(-MAX_SPEED * 0.5)
        if robot.step(timestep) == -1:
            return
    update_odometry()
    turn_180()
    h = get_heading()
    pose_theta     = math.atan2(math.sin(h), math.cos(h))
    prev_left_enc  = left_encoder.getValue()
    prev_right_enc = right_encoder.getValue()


# ── Perception ────────────────────────────────────────────────
def read_proximity():
    return [s.getValue() for s in proximity_sensors]


def obstacle_detected(readings):
    return any(v > OBSTACLE_THRESHOLD for v in [readings[1], readings[2], readings[5], readings[6]])


def _scan_camera(r_min, r_max, g_min, g_max, b_min, b_max):
    img = camera.getImage()
    w, h = camera.getWidth(), camera.getHeight()
    count, x_sum = 0, 0
    for row in range(h):
        for col in range(w):
            r = camera.imageGetRed(img, w, col, row)
            g = camera.imageGetGreen(img, w, col, row)
            b = camera.imageGetBlue(img, w, col, row)
            if r_min <= r <= r_max and g_min <= g <= g_max and b_min <= b <= b_max:
                count += 1
                x_sum += col
    offset = ((x_sum / count) - w / 2) / (w / 2) if count > 0 else 0.0
    return count, offset


def detect_flag_in_camera():
    count, _ = _scan_camera(150, 255, 0, 80, 0, 80)  # red flag
    return count >= FLAG_PIXEL_THRESHOLD


def detect_guard_in_camera():
    count, offset = _scan_camera(150, 255, 120, 255, 0, 80)  # yellow guard
    return count, offset


# ── Guard Sightings ───────────────────────────────────────────
def prune_guard_sightings():
    global guard_sightings
    guard_sightings = [s for s in guard_sightings if sim_time - s['t'] < GUARD_SIGHTING_TTL]


def waypoint_near_sighting(wx, wy):
    return any(
        math.sqrt((wx - s['pos'][0])**2 + (wy - s['pos'][1])**2) < GUARD_SIGHTING_RADIUS
        for s in guard_sightings
    )


def add_guard_sighting(x, y):
    guard_sightings.append({'pos': (x, y), 't': sim_time})


# ── Communication ─────────────────────────────────────────────
def broadcast_heartbeat_if_due():
    global last_hb_broadcast
    if sim_time - last_hb_broadcast < HEARTBEAT_RATE:
        return
    msg = {'type': 'HEARTBEAT', 'id': robot_id,
           'pos': [round(pose_x, 3), round(pose_y, 3)],
           'has_flag': has_flag, 't': round(sim_time, 2)}
    emitter.send(json.dumps(msg).encode('utf-8'))
    last_hb_broadcast = sim_time


def broadcast_guard_seen_if_due(gx, gy):
    global last_guard_broadcast
    if sim_time - last_guard_broadcast < GUARD_SEEN_RATE:
        return
    msg = {'type': 'GUARD_SEEN', 'id': robot_id,
           'pos': [round(gx, 3), round(gy, 3)], 't': round(sim_time, 2)}
    emitter.send(json.dumps(msg).encode('utf-8'))
    last_guard_broadcast = sim_time
    log_csv('GUARD_SEEN', f'pos=({gx:.2f},{gy:.2f})')


def check_teammate_comms():
    global last_heartbeat_time, solo_mode, teammate_pos, teammate_has_flag, yield_counter, escort_mode

    while receiver.getQueueLength() > 0:
        raw = receiver.getString()
        receiver.nextPacket()
        try:
            msg = json.loads(raw)
        except (json.JSONDecodeError, ValueError):
            continue

        if msg.get('id') == robot_id:
            continue  # ignore own messages

        t = msg.get('type', '')

        if t == 'HEARTBEAT':
            last_heartbeat_time = sim_time
            teammate_pos      = tuple(msg.get('pos', [0.0, 0.0]))
            teammate_has_flag = msg.get('has_flag', False)
            if not solo_mode:
                d = math.sqrt((pose_x - teammate_pos[0])**2 + (pose_y - teammate_pos[1])**2)
                tm_id = msg.get('id', 1 - robot_id)
                if d < YIELD_DISTANCE and robot_id < tm_id:
                    yield_counter = YIELD_STEPS

        elif t == 'FLAG_CAPTURED':
            if not has_flag:
                escort_mode = True
                print(f'[{robot_name}] teammate has flag — returning to base')
                log_csv('ESCORT_MODE', f'triggered by raider {msg.get("id")}')

        elif t == 'GUARD_SEEN':
            pos = msg.get('pos', [0.0, 0.0])
            t_seen = msg.get('t', sim_time)
            if sim_time - t_seen < GUARD_SIGHTING_TTL:
                add_guard_sighting(pos[0], pos[1])

    solo_mode = (sim_time - last_heartbeat_time > COMM_TIMEOUT)


# ── Stuck Detection ───────────────────────────────────────────
def check_if_stuck():
    global last_check_x, last_check_y, stuck_timer
    stuck_timer += dt
    if stuck_timer >= STUCK_TIME_LIMIT:
        moved       = distance_to(last_check_x, last_check_y)
        stuck_timer  = 0.0
        last_check_x = pose_x
        last_check_y = pose_y
        if moved < STUCK_DISTANCE:
            return True
    return False


# ── Respawn ───────────────────────────────────────────────────
def check_if_tagged():
    count, _ = _scan_camera(150, 255, 120, 255, 0, 80)  # yellow guard
    return count > TAG_PIXEL_THRESHOLD


def respawn():
    global has_flag, pose_x, pose_y, pose_theta, guard_sightings
    global prev_left_enc, prev_right_enc, seek_phase
    node       = robot.getSelf()
    node.getField('translation').setSFVec3f([SPAWN_X, SPAWN_Y, 0.0])
    node.getField('rotation').setSFRotation([0, 0, 1, 0])
    has_flag       = False
    seek_phase     = 'approach'
    pose_x, pose_y = SPAWN_X, SPAWN_Y
    pose_theta     = 0.0
    guard_sightings.clear()
    prev_left_enc  = left_encoder.getValue()
    prev_right_enc = right_encoder.getValue()
    print(f'[{robot_name}] TAGGED — respawned at ({SPAWN_X:.2f}, {SPAWN_Y:.2f})')
    log_csv('RESPAWN', f'teleported to ({SPAWN_X:.2f},{SPAWN_Y:.2f})')


# ── BT State: AVOID_OBSTACLE ──────────────────────────────────
def run_avoid_obstacle(readings):
    global escape_counter, escape_turn_left
    rp = readings[0] + readings[1] + readings[2]
    lp = readings[5] + readings[6] + readings[7]
    if lp > rp:
        set_wheel_speeds( MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
        escape_turn_left = False
    else:
        set_wheel_speeds(-MAX_SPEED * 0.5,  MAX_SPEED * 0.5)
        escape_turn_left = True
    escape_counter = ESCAPE_STEPS


# ── BT State: RECOVERY ────────────────────────────────────────
def run_recovery():
    global recovery_phase, recovery_counter
    if recovery_counter <= 0:
        recovery_phase   = 'reverse'
        recovery_counter = RECOVERY_REVERSE_STEPS
    if recovery_phase == 'reverse':
        set_wheel_speeds(-MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
        recovery_counter -= 1
        if recovery_counter <= 0:
            recovery_phase   = 'turn'
            recovery_counter = RECOVERY_TURN_STEPS
    else:
        set_wheel_speeds(MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
        recovery_counter -= 1


# ── BT State: EVADE_GUARD ─────────────────────────────────────
def run_evade_guard():
    _, offset = detect_guard_in_camera()
    if offset < 0:  # guard left → turn right
        set_wheel_speeds(MAX_SPEED, MAX_SPEED * 0.2)
    else:           # guard right → turn left
        set_wheel_speeds(MAX_SPEED * 0.2, MAX_SPEED)
    broadcast_guard_seen_if_due(pose_x, pose_y)


# ── BT State: RETURN_TO_BASE ──────────────────────────────────
def run_return_to_base():
    global has_flag, seek_phase, mission_done
    if distance_to(*HOME_BASE) < BASE_ARRIVAL_RADIUS:
        if has_flag:
            print(f'[{robot_name}] flag delivered — mission complete!')
            log_csv('FLAG_DELIVERED', f'pos=({pose_x:.2f},{pose_y:.2f})')
        else:
            print(f'[{robot_name}] escort arrived at base — mission complete!')
            log_csv('ESCORT_ARRIVED', f'pos=({pose_x:.2f},{pose_y:.2f})')
        set_wheel_speeds(0, 0)
        mission_done = True
    else:
        steer_toward(*HOME_BASE)


# ── BT State: SEEK_FLAG ───────────────────────────────────────
def run_seek_flag():
    global seek_phase

    if seek_phase == 'approach':
        # Switch approach if it's near a guard sighting
        wp = approach_wp
        alt_wp = SOUTH_APPROACH if approach_wp == NORTH_APPROACH else NORTH_APPROACH
        if waypoint_near_sighting(*wp) and not waypoint_near_sighting(*alt_wp):
            wp = alt_wp

        if distance_to(*wp) < WAYPOINT_TOLERANCE:
            seek_phase = 'flag'
        else:
            steer_toward(*wp)

    else:  # 'flag'
        steer_toward(*FLAG_POS)


# ── Flag Capture ──────────────────────────────────────────────
def check_flag_capture():
    global has_flag
    if not has_flag and distance_to(*FLAG_POS) < FLAG_CAPTURE_RADIUS and detect_flag_in_camera():
        has_flag = True
        print(f'[{robot_name}] captured the flag!')
        log_csv('FLAG_CAPTURED', f'pos=({pose_x:.2f},{pose_y:.2f})')
        msg = {'type': 'FLAG_CAPTURED', 'id': robot_id, 't': round(sim_time, 2)}
        emitter.send(json.dumps(msg).encode('utf-8'))
        log_csv('BROADCAST', 'FLAG_CAPTURED')


# ── BT Priority Selector ──────────────────────────────────────
def select_state(readings):
    global avoidance_timer, evade_guard_steps

    if obstacle_detected(readings):
        avoidance_timer += dt
        return State.AVOID_OBSTACLE
    elif escape_counter == 0:
        avoidance_timer = 0.0

    if check_if_stuck() or avoidance_timer >= AVOIDANCE_LIMIT:
        return State.RECOVERY

    guard_count, _ = detect_guard_in_camera()
    if guard_count >= GUARD_PIXEL_THRESHOLD:
        evade_guard_steps = EVADE_GUARD_MIN_STEPS
    if evade_guard_steps > 0:
        evade_guard_steps -= 1
        return State.EVADE_GUARD

    if has_flag or escort_mode:
        return State.RETURN_TO_BASE

    return State.SEEK_FLAG


# ── Start-of-Round Approach Auction ──────────────────────────
def run_auction():
    """Broadcast BID, wait up to BID_TIMEOUT for teammate reply, assign approach waypoints."""
    global approach_wp, sim_time

    d_north = distance_to(*NORTH_APPROACH)
    d_south = distance_to(*SOUTH_APPROACH)
    bid     = {'type': 'BID', 'id': robot_id,
               'north_cost': round(d_north, 4), 'south_cost': round(d_south, 4)}
    emitter.send(json.dumps(bid).encode('utf-8'))
    log_csv('BID', f'north={d_north:.3f} south={d_south:.3f}')

    teammate_bid = None
    deadline     = sim_time + BID_TIMEOUT

    while robot.step(timestep) != -1:
        sim_time += dt
        while receiver.getQueueLength() > 0:
            raw = receiver.getString()
            receiver.nextPacket()
            try:
                m = json.loads(raw)
                if m.get('type') == 'BID' and m.get('id') != robot_id:
                    teammate_bid = m
            except (json.JSONDecodeError, ValueError):
                pass
        if teammate_bid or sim_time >= deadline:
            break

    if teammate_bid:
        tm_n = teammate_bid['north_cost']
        tm_s = teammate_bid['south_cost']
        # Compare both assignment combinations; lower-ID breaks ties
        cost_self_n = d_north + tm_s   # self→NORTH, teammate→SOUTH
        cost_self_s = d_south + tm_n   # self→SOUTH, teammate→NORTH
        if cost_self_n < cost_self_s or (cost_self_n == cost_self_s and robot_id == 0):
            approach_wp = NORTH_APPROACH
        else:
            approach_wp = SOUTH_APPROACH
        log_csv('AUCTION_RESULT', f'self={approach_wp} teammate_bid_received=True')
    else:
        approach_wp = NORTH_APPROACH if d_north <= d_south else SOUTH_APPROACH
        log_csv('AUCTION_RESULT', f'self={approach_wp} solo_timeout=True')

    print(f'[{robot_name}] approach assigned: {approach_wp}')


# ── Startup Calibration ───────────────────────────────────────
robot.step(timestep)
_h        = get_heading()
pose_theta = math.atan2(math.sin(_h), math.cos(_h))
prev_left_enc  = left_encoder.getValue()
prev_right_enc = right_encoder.getValue()
print(f'[INIT] {robot_name}  id={robot_id}  spawn=({SPAWN_X},{SPAWN_Y})  '
      f'compass={math.degrees(_h):.1f}°')

run_auction()


# ── Main Control Loop ─────────────────────────────────────────
while robot.step(timestep) != -1:
    sim_time += dt

    if mission_done:
        set_wheel_speeds(0, 0)
        continue

    update_odometry()

    if check_if_tagged():
        respawn()
        continue

    broadcast_heartbeat_if_due()
    check_teammate_comms()
    prune_guard_sightings()

    proximity_readings = read_proximity()
    check_flag_capture()

    # Yield to teammate (lower-ID halts)
    if yield_counter > 0:
        set_wheel_speeds(0, 0)
        yield_counter -= 1
        continue

    current_state = select_state(proximity_readings)

    if current_state != prev_state:
        print(f'[STATE] {robot_name}: '
              f'{prev_state.name if prev_state else "START"} → {current_state.name}'
              f'  x={pose_x:.3f}  y={pose_y:.3f}  solo={solo_mode}')
        log_csv('STATE', f'{current_state.name} x={pose_x:.3f} y={pose_y:.3f}')
        prev_state = current_state

    if current_state == State.AVOID_OBSTACLE:
        run_avoid_obstacle(proximity_readings)
    elif escape_counter > 0:
        if escape_turn_left:
            set_wheel_speeds(-MAX_SPEED * 0.5, MAX_SPEED * 0.5)
        else:
            set_wheel_speeds(MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
        escape_counter -= 1
    elif current_state == State.RECOVERY:
        run_recovery()
    elif current_state == State.EVADE_GUARD:
        run_evade_guard()
    elif current_state == State.RETURN_TO_BASE:
        run_return_to_base()
    elif current_state == State.SEEK_FLAG:
        run_seek_flag()
