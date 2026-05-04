# =============================================================
# PROWL-R | attacker_controller.py
# Role: Seeks the enemy flag and returns it to home base.
#       Evades defenders detected via camera.
# Team: Oscar Chia, Blair Fenton, Lakshay
# =============================================================

from controller import Robot
from enum import Enum
import math


# ── Constants ─────────────────────────────────────────────────
MAX_SPEED          = 6.28   # rad/s — e-puck physical maximum
OBSTACLE_THRESHOLD = 80     # proximity reading that triggers avoidance (matches e-puck spec)
STUCK_DISTANCE     = 0.02   # metres — minimum movement to not be considered stuck
STUCK_TIME_LIMIT   = 3.0    # seconds of no movement before RECOVERY triggers
AVOIDANCE_LIMIT    = 3.0    # seconds of continuous avoidance before RECOVERY triggers
ESCAPE_STEPS          = 20  # timesteps to sustain turn direction after clearing obstacle
RECOVERY_REVERSE_STEPS = 30  # timesteps to reverse during recovery
RECOVERY_TURN_STEPS    = 45  # timesteps to turn after reversing
WAYPOINT_TOLERANCE  = 0.07  # metres — close enough to consider a waypoint reached
FLAG_PIXEL_THRESHOLD = 5    # minimum matching pixels in camera frame to confirm flag is visible
COMM_TIMEOUT        = 5.0   # seconds without teammate heartbeat before SOLO mode
WHEEL_RADIUS       = 0.0205 # metres (e-puck hardware spec)
AXLE_LENGTH        = 0.052  # metres (e-puck hardware spec)


# ── Behaviour Tree States ─────────────────────────────────────
class State(Enum):
    AVOID_OBSTACLE = 1  # highest priority — reactive collision avoidance
    RECOVERY       = 2  # escape stuck or avoidance-loop situations
    SEEK_FLAG      = 3  # default — navigate toward enemy flag via waypoints
    EVADE_DEFENDER = 4  # defender spotted in camera — take evasive action
    RETURN_TO_BASE = 5  # flag captured — navigate home


# ── Robot & Sensor Initialisation ────────────────────────────
robot    = Robot()
timestep = int(robot.getBasicTimeStep())
dt       = timestep / 1000.0  # seconds per simulation step

# Differential drive motors — set to velocity control mode
left_motor  = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# IR proximity ring: ps0 (front-right) → ps7 (front-left)
proximity_sensors = []
for i in range(8):
    ps = robot.getDevice(f'ps{i}')
    ps.enable(timestep)
    proximity_sensors.append(ps)

# Wheel encoders for dead-reckoning odometry
left_encoder  = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# RGB camera — used for flag and opponent colour detection
camera = robot.getDevice('camera')
camera.enable(timestep)

# Emitter/Receiver — inter-agent communication channel
emitter  = robot.getDevice('emitter')
receiver = robot.getDevice('receiver')
receiver.enable(timestep)


# ── Team Initialisation ───────────────────────────────────────
# Read the robot name set in the world file to determine which team this
# instance belongs to, then configure team-specific waypoints and comms channel.
robot_name = robot.getName()
team       = 'a' if 'team_a' in robot_name else 'b'

if team == 'a':
    # Team A spawns on the left — attacks rightward toward the green flag (right base)
    ENEMY_BASE_WAYPOINTS = [(0.5, 0.0), (0.85, 0.0)]
    HOME_BASE_WAYPOINTS  = [(-0.85, 0.0), (-0.5, 0.0)]
    ENEMY_FLAG_COLOUR = 'GREEN'
    emitter.setChannel(1)
    receiver.setChannel(1)
else:
    # Team B spawns on the right — attacks leftward toward the yellow flag (left base)
    ENEMY_BASE_WAYPOINTS = [(-0.5, 0.0), (-0.85, 0.0)]
    HOME_BASE_WAYPOINTS  = [(0.85, 0.0), (0.5, 0.0)]
    ENEMY_FLAG_COLOUR = 'YELLOW'
    emitter.setChannel(2)
    receiver.setChannel(2)


# ── Agent State Variables ─────────────────────────────────────
current_state = State.SEEK_FLAG
has_flag      = False
solo_mode     = False  # True when teammate comms have timed out

# Odometry — pose estimate updated every timestep
pose_x         = 0.0
pose_y         = 0.0
pose_theta     = 0.0
prev_left_enc  = 0.0
prev_right_enc = 0.0

# Stuck detection bookkeeping
last_check_x    = 0.0
last_check_y    = 0.0
stuck_timer     = 0.0
avoidance_timer = 0.0

# Escape counter — sustains the avoidance turn for ESCAPE_STEPS after obstacle clears,
# preventing the robot from immediately driving back into the same obstacle
escape_counter   = 0
escape_turn_left = True

# Recovery sequence — tracks which phase (reverse/turn) and how many steps remain
recovery_phase   = 'reverse'
recovery_counter = 0

# Waypoint navigation indices
seek_waypoint_index   = 0
return_waypoint_index = len(HOME_BASE_WAYPOINTS) - 1

# Simulation clock and comms tracking
sim_time            = 0.0
last_heartbeat_time = 0.0
last_status_print   = 0.0  # tracks when the last search status line was printed

# Route memory — list of waypoint indices where the attacker was intercepted
blocked_routes = []


# ── Odometry ─────────────────────────────────────────────────
def update_odometry():
    """Integrate encoder deltas into (x, y, θ) pose estimate."""
    global pose_x, pose_y, pose_theta, prev_left_enc, prev_right_enc

    left_enc  = left_encoder.getValue()
    right_enc = right_encoder.getValue()

    delta_left  = (left_enc  - prev_left_enc)  * WHEEL_RADIUS
    delta_right = (right_enc - prev_right_enc) * WHEEL_RADIUS
    delta_dist  = (delta_left + delta_right) / 2.0
    delta_theta = (delta_right - delta_left) / AXLE_LENGTH

    pose_theta += delta_theta
    # Wrap to [-π, π] so heading never accumulates past a full rotation
    pose_theta  = math.atan2(math.sin(pose_theta), math.cos(pose_theta))
    pose_x     += delta_dist * math.cos(pose_theta)
    pose_y     += delta_dist * math.sin(pose_theta)

    prev_left_enc  = left_enc
    prev_right_enc = right_enc


def reset_pose_to(known_x, known_y, known_theta):
    """Re-zero odometry at a known ground-truth position to bound drift."""
    global pose_x, pose_y, pose_theta
    pose_x     = known_x
    pose_y     = known_y
    pose_theta = known_theta


# ── Navigation ────────────────────────────────────────────────
def steer_toward(target_x, target_y):
    """Proportional heading-error steering toward a target position."""
    angle_to_target = math.atan2(target_y - pose_y, target_x - pose_x)
    heading_error   = angle_to_target - pose_theta
    # Wrap to [-π, π] so the robot always turns the short way
    heading_error   = math.atan2(math.sin(heading_error), math.cos(heading_error))

    left_speed  = MAX_SPEED - heading_error * 2.0
    right_speed = MAX_SPEED + heading_error * 2.0

    # Clamp to physical motor limits
    left_speed  = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
    right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


def distance_to(target_x, target_y):
    """Euclidean distance from current pose to target."""
    return math.sqrt((target_x - pose_x) ** 2 + (target_y - pose_y) ** 2)


def set_wheel_speeds(left, right):
    """Clamp and apply motor velocities."""
    left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, left)))
    right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, right)))


# ── Perception ────────────────────────────────────────────────
def read_proximity():
    """Return current readings from all 8 proximity sensors."""
    return [s.getValue() for s in proximity_sensors]


def obstacle_detected(readings):
    """True if any front-arc sensor exceeds the avoidance threshold."""
    # ps7, ps0 are front-left and front-right; ps1, ps6 are side-front
    front_arc = [readings[7], readings[0], readings[1], readings[6]]
    return any(v > OBSTACLE_THRESHOLD for v in front_arc)


def detect_flag_in_camera():
    """
    Scan every camera pixel for the enemy flag colour.
    Blue team looks for green pixels (enemy green flag at right base).
    Red team looks for yellow pixels (enemy yellow flag at left base).
    Returns True if at least FLAG_PIXEL_THRESHOLD matching pixels are found.
    """
    image   = camera.getImage()
    w       = camera.getWidth()
    h       = camera.getHeight()
    matches = 0

    for y in range(h):
        for x in range(w):
            r = camera.imageGetRed(image, w, x, y)
            g = camera.imageGetGreen(image, w, x, y)
            b = camera.imageGetBlue(image, w, x, y)

            if team == 'a':
                # Green flag: dominant green, low red and blue
                if g > 150 and r < 80 and b < 80:
                    matches += 1
            else:
                # Yellow flag: high red and green, low blue
                if r > 150 and g > 150 and b < 80:
                    matches += 1

            if matches >= FLAG_PIXEL_THRESHOLD:
                return True

    return False


def detect_defender_in_camera():
    """Scan camera image for blue robot pixels indicating an opposing defender."""
    # TODO: implement HSV segmentation — isolate blue hue band in camera.getImage()
    return False


# ── Communication ─────────────────────────────────────────────
def broadcast_status():
    """Send position, heading, time, and flag status to teammate as a CSV string."""
    message = f"{pose_x},{pose_y},{pose_theta},{sim_time},{int(has_flag)}"
    emitter.send(message.encode('utf-8'))


def check_teammate_comms():
    """Read incoming packets; switch to SOLO mode if heartbeat times out."""
    global last_heartbeat_time, solo_mode

    while receiver.getQueueLength() > 0:
        receiver.getString()  # packet contents used once coordination logic is implemented
        last_heartbeat_time = sim_time
        receiver.nextPacket()

    # No message for COMM_TIMEOUT seconds → assume teammate is offline
    solo_mode = (sim_time - last_heartbeat_time > COMM_TIMEOUT)


# ── Stuck Detection ───────────────────────────────────────────
def check_if_stuck():
    """Return True if pose has not changed enough over STUCK_TIME_LIMIT seconds."""
    global last_check_x, last_check_y, stuck_timer

    stuck_timer += dt
    if stuck_timer >= STUCK_TIME_LIMIT:
        moved        = distance_to(last_check_x, last_check_y)
        stuck_timer  = 0.0
        last_check_x = pose_x
        last_check_y = pose_y
        if moved < STUCK_DISTANCE:
            return True
    return False


# ── BT State: AVOID_OBSTACLE ──────────────────────────────────
def run_avoid_obstacle(readings):
    """Turn away from the side with the highest proximity reading."""
    global escape_counter, escape_turn_left

    # ps0,ps1,ps2 are on the RIGHT side; ps5,ps6,ps7 are on the LEFT side
    right_side_pressure = readings[0] + readings[1] + readings[2]
    left_side_pressure  = readings[5] + readings[6] + readings[7]

    if left_side_pressure > right_side_pressure:
        # Obstacle on left → turn right (left wheel forward, right wheel back)
        set_wheel_speeds( MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
        escape_turn_left = False
    else:
        # Obstacle on right (or straight ahead) → turn left (left wheel back, right wheel forward)
        set_wheel_speeds(-MAX_SPEED * 0.5,  MAX_SPEED * 0.5)
        escape_turn_left = True

    # Reset escape counter so the turn is sustained after the obstacle clears
    escape_counter = ESCAPE_STEPS


# ── BT State: RECOVERY ────────────────────────────────────────
def run_recovery():
    """Two-phase escape: reverse away from the obstacle, then turn to face a new direction."""
    global recovery_phase, recovery_counter

    if recovery_counter <= 0:
        # Begin a fresh recovery sequence starting with the reverse phase
        recovery_phase   = 'reverse'
        recovery_counter = RECOVERY_REVERSE_STEPS

    if recovery_phase == 'reverse':
        set_wheel_speeds(-MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
        recovery_counter -= 1
        if recovery_counter <= 0:
            # Switch to turn phase once reverse is complete
            recovery_phase   = 'turn'
            recovery_counter = RECOVERY_TURN_STEPS
    else:
        # Turn right to face a new heading away from the blocked corridor
        set_wheel_speeds(MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
        recovery_counter -= 1


# ── Flag Capture ──────────────────────────────────────────────
def check_flag_capture():
    """
    Runs every main loop tick regardless of BT state — including during AVOID_OBSTACLE.
    Uses camera colour detection to confirm the enemy flag is visible.
    Only active once intermediate waypoints are cleared so the robot is in the right zone.
    """
    global has_flag

    if has_flag:
        return

    # Only check once intermediate waypoints are cleared
    if seek_waypoint_index < len(ENEMY_BASE_WAYPOINTS) - 1:
        return

    if detect_flag_in_camera():
        has_flag = True
        print(f"[{robot_name}] captured the {ENEMY_FLAG_COLOUR} flag!")


# ── BT State: SEEK_FLAG ───────────────────────────────────────
def run_seek_flag():
    """Navigate through ENEMY_BASE_WAYPOINTS toward the enemy flag."""
    global seek_waypoint_index

    # Skip any waypoint index previously marked as blocked
    while seek_waypoint_index in blocked_routes and seek_waypoint_index < len(ENEMY_BASE_WAYPOINTS) - 1:
        seek_waypoint_index += 1

    target = ENEMY_BASE_WAYPOINTS[seek_waypoint_index]

    if distance_to(*target) < WAYPOINT_TOLERANCE:
        if seek_waypoint_index < len(ENEMY_BASE_WAYPOINTS) - 1:
            seek_waypoint_index += 1

    steer_toward(*target)


# ── BT State: EVADE_DEFENDER ──────────────────────────────────
def run_evade_defender():
    """Evasive manoeuvre based on defender's position in the camera frame."""
    # TODO: read camera image to determine defender's x-offset and steer away
    set_wheel_speeds(MAX_SPEED, MAX_SPEED * 0.3)


# ── BT State: RETURN_TO_BASE ──────────────────────────────────
def run_return_to_base():
    """Navigate back through HOME_BASE_WAYPOINTS to deliver the flag."""
    global return_waypoint_index, seek_waypoint_index, has_flag

    target = HOME_BASE_WAYPOINTS[return_waypoint_index]

    if distance_to(*target) < WAYPOINT_TOLERANCE:
        if return_waypoint_index > 0:
            return_waypoint_index -= 1
        else:
            # Reached home base — flag delivered, reset for next round
            has_flag              = False
            seek_waypoint_index   = 0
            return_waypoint_index = len(HOME_BASE_WAYPOINTS) - 1
            reset_pose_to(0.0, 0.0, 0.0)

    steer_toward(*target)


# ── BT Priority Selector ──────────────────────────────────────
def select_state(readings):
    """
    Evaluate conditions from highest to lowest priority.
    The first condition that is True wins — lower states only run
    when every higher-priority condition is inactive.
    """
    global avoidance_timer

    # Priority 1 — imminent collision always overrides everything
    if obstacle_detected(readings):
        avoidance_timer += dt
        return State.AVOID_OBSTACLE
    elif escape_counter == 0:
        # Only reset the timer once fully clear — not during the escape phase,
        # so corridor oscillation keeps accumulating toward RECOVERY
        avoidance_timer = 0.0

    # Priority 2 — escape if stuck or trapped in avoidance loop
    if check_if_stuck() or avoidance_timer >= AVOIDANCE_LIMIT:
        return State.RECOVERY

    # Priority 3 — deliver flag if already carrying it
    if has_flag:
        return State.RETURN_TO_BASE

    # Priority 4 — evade if a defender is visible
    if detect_defender_in_camera():
        return State.EVADE_DEFENDER

    # Default — seek the enemy flag
    return State.SEEK_FLAG


# ── Main Control Loop ─────────────────────────────────────────
while robot.step(timestep) != -1:
    sim_time += dt

    update_odometry()
    broadcast_status()
    check_teammate_comms()

    proximity_readings = read_proximity()
    check_flag_capture()

    # Print a search status line every 3 seconds until the flag is captured
    if not has_flag and sim_time - last_status_print >= 3.0:
        print(f"[{robot_name}] searching for the {ENEMY_FLAG_COLOUR} flag...")
        last_status_print = sim_time

    current_state      = select_state(proximity_readings)

    if current_state == State.AVOID_OBSTACLE:
        run_avoid_obstacle(proximity_readings)
    elif escape_counter > 0:
        # Obstacle just cleared — continue turning to fully leave the obstacle zone
        if escape_turn_left:
            set_wheel_speeds(-MAX_SPEED * 0.5, MAX_SPEED * 0.5)
        else:
            set_wheel_speeds(MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
        escape_counter -= 1
    elif current_state == State.RECOVERY:
        run_recovery()
    elif current_state == State.SEEK_FLAG:
        run_seek_flag()
    elif current_state == State.EVADE_DEFENDER:
        run_evade_defender()
    elif current_state == State.RETURN_TO_BASE:
        run_return_to_base()
