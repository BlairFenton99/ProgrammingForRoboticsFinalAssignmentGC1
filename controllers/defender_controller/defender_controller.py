# =============================================================
# PROWL-R | defender_controller.py
# Role: Patrols the home zone, detects intruders via camera,
#       chases and tags them, then returns to patrol.
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
WAYPOINT_TOLERANCE = 0.07   # metres — close enough to consider a waypoint reached
TAG_RANGE          = 0.1    # metres — attacker must be this close to be tagged
COMM_TIMEOUT       = 5.0    # seconds without teammate heartbeat before SOLO mode
WHEEL_RADIUS       = 0.0205 # metres (e-puck hardware spec)
AXLE_LENGTH        = 0.052  # metres (e-puck hardware spec)


# ── Behaviour Tree States ─────────────────────────────────────
class State(Enum):
    AVOID_OBSTACLE = 1  # highest priority — reactive collision avoidance
    RECOVERY       = 2  # escape stuck or avoidance-loop situations
    PATROL_ZONE    = 3  # default — sweep patrol waypoints and monitor camera
    CHASE_INTRUDER = 4  # intruder spotted — pursue and attempt to tag
    RETURN_TO_POST = 5  # intruder tagged or lost — return to patrol route


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

# RGB camera — used for intruder colour detection
camera = robot.getDevice('camera')
camera.enable(timestep)

# Emitter/Receiver — inter-agent communication channel
emitter  = robot.getDevice('emitter')
receiver = robot.getDevice('receiver')
receiver.enable(timestep)


# ── Team Initialisation ───────────────────────────────────────
# Read the robot name set in the world file to determine which team this
# instance belongs to, then configure team-specific patrol waypoints and comms channel.
robot_name = robot.getName()
team       = 'a' if 'team_a' in robot_name else 'b'

if team == 'a':
    # Team A defender patrols the left half (negative x)
    PATROL_WAYPOINTS = [(-0.3, 0.2), (-0.3, -0.2), (-0.6, -0.2), (-0.6, 0.2)]
    emitter.setChannel(1)
    receiver.setChannel(1)
else:
    # Team B defender patrols the right half (positive x)
    PATROL_WAYPOINTS = [(0.3, -0.2), (0.3, 0.2), (0.6, 0.2), (0.6, -0.2)]
    emitter.setChannel(2)
    receiver.setChannel(2)

# Return destination after a chase — first waypoint in the patrol loop
HOME_POST = PATROL_WAYPOINTS[0]


# ── Agent State Variables ─────────────────────────────────────
current_state   = State.PATROL_ZONE
intruder_tagged = False
solo_mode       = False  # True when teammate comms have timed out

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

# Patrol waypoint index — cycles through PATROL_WAYPOINTS
patrol_index = 0

# Simulation clock and comms tracking
sim_time            = 0.0
last_heartbeat_time = 0.0

# Adaptive patrol — tracks which side attackers have historically approached from
approach_history = {'left': 0, 'right': 0}


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


def detect_intruder_in_camera():
    """Scan camera image for red robot pixels indicating an opposing attacker."""
    # TODO: implement HSV segmentation — isolate red hue band in camera.getImage()
    return False


def get_intruder_camera_offset():
    """
    Return the horizontal pixel offset of the intruder from the camera centre.
    Negative = intruder is left of centre, positive = right of centre.
    Used to steer the chase toward the intruder's position.
    """
    # TODO: implement centroid calculation from HSV-segmented red pixels
    return 0.0


# ── Communication ─────────────────────────────────────────────
def broadcast_status():
    """Send position, heading, time, and tag status to teammate as a CSV string."""
    message = f"{pose_x},{pose_y},{pose_theta},{sim_time},{int(intruder_tagged)}"
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


# ── Adaptive Patrol Bias ──────────────────────────────────────
def biased_patrol_index():
    """
    Return the patrol waypoint index that biases coverage toward the side
    attackers have historically approached from most often.
    Defenders shift patrol weight to cut off favoured attacker routes.
    """
    # TODO: map approach_history counts to a preferred waypoint index
    return patrol_index


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


# ── BT State: PATROL_ZONE ─────────────────────────────────────
def run_patrol_zone():
    """Cycle through PATROL_WAYPOINTS; shift bias based on attacker history."""
    global patrol_index

    target = PATROL_WAYPOINTS[biased_patrol_index()]

    if distance_to(*target) < WAYPOINT_TOLERANCE:
        # Advance to next waypoint and re-zero pose if at a known position
        patrol_index = (patrol_index + 1) % len(PATROL_WAYPOINTS)
        reset_pose_to(*target, pose_theta)

    steer_toward(*target)


# ── BT State: CHASE_INTRUDER ──────────────────────────────────
def run_chase_intruder():
    """
    Steer toward the intruder using their camera pixel offset.
    Proximity sensors confirm a tag when the attacker is within TAG_RANGE.
    """
    global intruder_tagged, approach_history

    offset = get_intruder_camera_offset()

    # Positive offset → intruder is right of centre → turn right (reduce left speed)
    left_speed  = MAX_SPEED - offset * 3.0
    right_speed = MAX_SPEED + offset * 3.0
    set_wheel_speeds(left_speed, right_speed)

    # Tag confirmed by proximity — attacker must re-spawn
    front_readings = [proximity_sensors[i].getValue() for i in [0, 7]]
    if any(v > OBSTACLE_THRESHOLD for v in front_readings):
        intruder_tagged = True
        # Record which side the attacker approached from for adaptive patrol
        if offset < 0:
            approach_history['left'] += 1
        else:
            approach_history['right'] += 1


# ── BT State: RETURN_TO_POST ──────────────────────────────────
def run_return_to_post():
    """Navigate back to the patrol start point after chasing."""
    global intruder_tagged, patrol_index

    if distance_to(*HOME_POST) < WAYPOINT_TOLERANCE:
        # Back on post — reset chase state and resume patrol
        intruder_tagged = False
        patrol_index    = 0
    else:
        steer_toward(*HOME_POST)


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

    # Priority 3 — return to post after a completed or lost chase
    if intruder_tagged:
        return State.RETURN_TO_POST

    # Priority 4 — chase if an intruder is visible in camera
    if detect_intruder_in_camera():
        return State.CHASE_INTRUDER

    # Default — patrol the home zone
    return State.PATROL_ZONE


# ── Main Control Loop ─────────────────────────────────────────
while robot.step(timestep) != -1:
    sim_time += dt

    update_odometry()
    broadcast_status()
    check_teammate_comms()

    proximity_readings = read_proximity()
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
    elif current_state == State.PATROL_ZONE:
        run_patrol_zone()
    elif current_state == State.CHASE_INTRUDER:
        run_chase_intruder()
    elif current_state == State.RETURN_TO_POST:
        run_return_to_post()
