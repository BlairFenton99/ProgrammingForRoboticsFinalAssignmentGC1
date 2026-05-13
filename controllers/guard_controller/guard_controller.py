# =============================================================
# PROWL-R | guard_controller.py
# Role: Scripted guard — compass north-south patrol, chase any
#       visible Raider, return to patrol when lost or timed out.
#       No comms, no learning.
# Team: Oscar Chia, Blair Fenton, Lakshay
# =============================================================

from controller import Robot
from enum import Enum
import math

# ── Constants ─────────────────────────────────────────────────
MAX_SPEED          = 6.28
OBSTACLE_THRESHOLD = 80
WHEEL_RADIUS       = 0.0205
CHASE_TIMEOUT      = 5.0   # seconds before giving up on a chase
RAIDER_PIXEL_THRESHOLD = 15  # blue pixels to trigger CHASE


class State(Enum):
    AVOID_OBSTACLE = 1
    PATROL         = 2
    CHASE          = 3


# ── Robot & Sensor Initialisation ────────────────────────────
robot    = Robot()
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

robot_name = robot.getName()

# ── Agent State ───────────────────────────────────────────────
current_state  = State.PATROL
prev_state     = None
patrol_heading = 1    # +1 = north, -1 = south
chase_elapsed  = 0.0

prev_left_enc  = 0.0
prev_right_enc = 0.0


# ── Helpers ───────────────────────────────────────────────────
def set_wheel_speeds(l, r):
    left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, l)))
    right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, r)))


def read_proximity():
    return [s.getValue() for s in proximity_sensors]


def obstacle_detected(readings):
    return any(v > OBSTACLE_THRESHOLD for v in [readings[1], readings[2], readings[5], readings[6]])


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
    global prev_left_enc, prev_right_enc
    steps = int(0.3 / dt)
    for _ in range(steps):
        left_motor.setVelocity(-MAX_SPEED * 0.5)
        right_motor.setVelocity(-MAX_SPEED * 0.5)
        if robot.step(timestep) == -1:
            return
    turn_180()
    prev_left_enc  = left_encoder.getValue()
    prev_right_enc = right_encoder.getValue()


def detect_raider_in_camera():
    """Returns (count, horizontal_offset) for blue raider pixels."""
    img = camera.getImage()
    w, h = camera.getWidth(), camera.getHeight()
    count, x_sum = 0, 0
    for row in range(h):
        for col in range(w):
            r = camera.imageGetRed(img, w, col, row)
            g = camera.imageGetGreen(img, w, col, row)
            b = camera.imageGetBlue(img, w, col, row)
            if b > 150 and r < 80 and g < 100:
                count += 1
                x_sum += col
    offset = ((x_sum / count) - w / 2) / (w / 2) if count > 0 else 0.0
    return count, offset


# ── BT State Handlers ─────────────────────────────────────────
def run_avoid_obstacle(readings):
    rp = readings[0] + readings[1] + readings[2]
    lp = readings[5] + readings[6] + readings[7]
    if lp > rp:
        set_wheel_speeds( MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
    else:
        set_wheel_speeds(-MAX_SPEED * 0.5,  MAX_SPEED * 0.5)


def run_patrol(readings):
    global patrol_heading
    if readings[7] > OBSTACLE_THRESHOLD or readings[0] > OBSTACLE_THRESHOLD:
        patrol_heading = -patrol_heading
        do_bounce()
        return
    target_h    = PATROL_NORTH if patrol_heading > 0 else PATROL_SOUTH
    compass_h   = get_heading()
    heading_err = math.atan2(math.sin(target_h - compass_h), math.cos(target_h - compass_h))
    if abs(heading_err) < 0.2:
        set_wheel_speeds(MAX_SPEED, MAX_SPEED)
    else:
        set_wheel_speeds(MAX_SPEED * 0.5 - heading_err * 2.0,
                         MAX_SPEED * 0.5 + heading_err * 2.0)


def run_chase(offset):
    set_wheel_speeds(MAX_SPEED - offset * 3.0, MAX_SPEED + offset * 3.0)


# ── Priority Selector ─────────────────────────────────────────
def select_state(readings, raider_count, raider_offset):
    global current_state, chase_elapsed

    if obstacle_detected(readings):
        return State.AVOID_OBSTACLE

    if current_state == State.CHASE:
        chase_elapsed += dt
        if raider_count < RAIDER_PIXEL_THRESHOLD or chase_elapsed > CHASE_TIMEOUT:
            chase_elapsed = 0.0
            return State.PATROL
        return State.CHASE

    if raider_count >= RAIDER_PIXEL_THRESHOLD:
        chase_elapsed = 0.0
        return State.CHASE

    return State.PATROL


# ── Startup Calibration ───────────────────────────────────────
robot.step(timestep)
_h           = get_heading()
PATROL_NORTH = _h
PATROL_SOUTH = (_h + math.pi) % (2 * math.pi)
prev_left_enc  = left_encoder.getValue()
prev_right_enc = right_encoder.getValue()
print(f'[INIT] {robot_name}  PATROL_NORTH={math.degrees(PATROL_NORTH):.1f}°  '
      f'PATROL_SOUTH={math.degrees(PATROL_SOUTH):.1f}°')


# ── Main Control Loop ─────────────────────────────────────────
while robot.step(timestep) != -1:
    proximity_readings          = read_proximity()
    raider_count, raider_offset = detect_raider_in_camera()

    current_state = select_state(proximity_readings, raider_count, raider_offset)

    if current_state != prev_state:
        print(f'[STATE] {robot_name}: '
              f'{prev_state.name if prev_state else "START"} → {current_state.name}')
        prev_state = current_state

    if current_state == State.AVOID_OBSTACLE:
        run_avoid_obstacle(proximity_readings)
    elif current_state == State.PATROL:
        run_patrol(proximity_readings)
    elif current_state == State.CHASE:
        run_chase(raider_offset)
