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

# RETURN_TO_PATROL constants — coarse odometry phase then camera fine-alignment
PATROL_LINE_X           = 0.0   # patrol line x in pose-space (spawn centre)
PATROL_LINE_NEAR_THRESH = 0.08  # metres — displacement that triggers RETURN_TO_PATROL
PATROL_LINE_CLOSE_THRESH= 0.05  # metres — switches coarse → seek_line phase
SEEK_LINE_TIMEOUT       = 5.0   # seconds — fallback if camera cannot find the line
SEEK_LINE_CONFIRM_STEPS = 5     # consecutive centred readings to confirm alignment


# ── Behaviour Tree States ─────────────────────────────────────
class State(Enum):
    AVOID_OBSTACLE   = 1  # highest priority — reactive collision avoidance
    RECOVERY         = 2  # escape stuck or avoidance-loop situations
    RETURN_TO_POST   = 3  # intruder tagged or lost — return to patrol route
    CHASE_INTRUDER   = 4  # intruder spotted — pursue and attempt to tag
    RETURN_TO_PATROL = 5  # displaced off the patrol line — re-align before patrolling
    PATROL_ZONE      = 6  # default — north-south sweep with line following


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

# Compass — provides absolute heading for clean 180° patrol turns
compass = robot.getDevice('compass')
compass.enable(timestep)

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
    emitter.setChannel(1)
    receiver.setChannel(1)
else:
    emitter.setChannel(2)
    receiver.setChannel(2)

# Return destination after a chase — spawn centre in pose space
HOME_POST = (0, 0)


# ── Agent State Variables ─────────────────────────────────────
current_state   = State.PATROL_ZONE
prev_state      = None   # used to detect and log state transitions
intruder_tagged = False
solo_mode       = False  # True when teammate comms have timed out

# Odometry — pose estimate updated every timestep
pose_x         = 0.0
pose_y         = 0.0
pose_theta     = math.pi if team == 'b' else 0.0  # Team B spawns facing -x (rotation π)
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

# Patrol heading — π/2 = north (+y), -π/2 = south (-y); flips on wall contact
patrol_heading = math.pi / 2

# Simulation clock and comms tracking
sim_time            = 0.0
last_heartbeat_time = 0.0

# Adaptive patrol — tracks which side attackers have historically approached from
approach_history = {'left': 0, 'right': 0}

# Last known position of the enemy attacker — broadcast to the friendly attacker
# so it can pick the safer route into enemy territory
last_intruder_x = 0.0
last_intruder_y = 0.0

# RETURN_TO_PATROL bookkeeping
displaced_from_patrol   = False   # set when off-line, cleared after re-alignment
return_to_patrol_phase  = 'coarse'  # 'coarse' | 'seek_line'
seek_line_timer         = 0.0
seek_line_confirm_count = 0


# ── Odometry ─────────────────────────────────────────────────
def update_odometry():
    """
    Integrate encoder distance into (x, y) using compass heading.
    Encoders give distance travelled; compass gives direction — combining
    them keeps x/y correct even when the encoder-integrated theta would drift.
    """
    global pose_x, pose_y, pose_theta, prev_left_enc, prev_right_enc

    left_enc  = left_encoder.getValue()
    right_enc = right_encoder.getValue()

    delta_left  = (left_enc  - prev_left_enc)  * WHEEL_RADIUS
    delta_right = (right_enc - prev_right_enc) * WHEEL_RADIUS
    delta_dist  = (delta_left + delta_right) / 2.0

    # Use compass for heading so encoder drift can't corrupt x/y integration
    h          = get_heading()
    pose_theta = math.atan2(math.sin(h), math.cos(h))
    pose_x    += delta_dist * math.cos(pose_theta)
    pose_y    += delta_dist * math.sin(pose_theta)

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
    # Use compass directly — pose_theta drifts and would give wrong direction
    compass_h   = math.atan2(math.sin(get_heading()), math.cos(get_heading()))
    heading_error = math.atan2(math.sin(angle_to_target - compass_h),
                               math.cos(angle_to_target - compass_h))

    left_speed  = MAX_SPEED - heading_error * 2.0
    right_speed = MAX_SPEED + heading_error * 2.0

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
    """
    True if any SIDE sensor exceeds the avoidance threshold.
    Front sensors are deliberately excluded — the patrol flip handles
    front wall detection so AVOID_OBSTACLE does not curve the robot off its
    north-south track when approaching the arena boundary.
    """
    side_arc = [readings[1], readings[2], readings[5], readings[6]]
    return any(v > OBSTACLE_THRESHOLD for v in side_arc)


def get_patrol_line_offset():
    """
    Scan every camera pixel for the red patrol line marker — same pixel API
    as the attacker flag detection (camera.imageGetRed/Green/Blue).
    Returns the horizontal centroid offset normalised to [-1, 1]:
      negative = line is left of centre → robot drifted right → steer left
      positive = line is right of centre → robot drifted left → steer right
    Returns 0.0 when no red pixels are found (line not in view).
    """
    image  = camera.getImage()
    w      = camera.getWidth()
    h      = camera.getHeight()
    x_sum  = 0
    count  = 0

    for y in range(h):
        for x in range(w):
            r = camera.imageGetRed(image, w, x, y)
            g = camera.imageGetGreen(image, w, x, y)
            b = camera.imageGetBlue(image, w, x, y)
            # Red patrol line: dominant red channel, low green and blue
            if r > 150 and g < 80 and b < 80:
                x_sum += x
                count += 1

    if count == 0:
        return 0.0
    centroid = x_sum / count
    return (centroid - w / 2) / (w / 2)


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
    """
    Send position, heading, time, tag status, and last known enemy position to teammate.
    The friendly attacker uses last_intruder_x/y to select the safer route into enemy territory.
    Format: pose_x, pose_y, pose_theta, sim_time, intruder_tagged, intruder_x, intruder_y
    """
    message = f"{pose_x},{pose_y},{pose_theta},{sim_time},{int(intruder_tagged)},{last_intruder_x:.3f},{last_intruder_y:.3f}"
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


# ── Compass Helpers ───────────────────────────────────────────
def get_heading():
    """Absolute heading from compass in [0, 2π]."""
    v = compass.getValues()
    h = math.atan2(v[0], v[1])
    return h if h >= 0 else h + 2 * math.pi


def turn_180(turn_speed=2.0, tolerance=0.02):
    """Spin 180° in place using P-control on compass heading. Blocking."""
    target = (get_heading() + math.pi) % (2 * math.pi)
    left_motor.setVelocity(-turn_speed)
    right_motor.setVelocity(turn_speed)
    while robot.step(timestep) != -1:
        error = math.atan2(math.sin(target - get_heading()),
                           math.cos(target - get_heading()))
        if abs(error) < tolerance:
            break
        if abs(error) < 0.3:
            speed = max(0.3, turn_speed * abs(error) / 0.3)
            left_motor.setVelocity(-speed)
            right_motor.setVelocity(speed)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)


def do_patrol_bounce():
    """
    Reverse briefly then spin 180° by compass.
    Resets pose_theta and encoder baseline after the manoeuvre so the
    main-loop odometry step does not double-count the turn movement.
    """
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


# ── BT State: PATROL_ZONE ─────────────────────────────────────
def run_patrol_zone(readings):
    """
    Drive straight north or south. When front sensors detect the arena wall,
    reverse and spin 180° using the compass for a clean direction flip.
    """
    global patrol_heading

    if readings[7] > OBSTACLE_THRESHOLD or readings[0] > OBSTACLE_THRESHOLD:
        patrol_heading = -patrol_heading
        do_patrol_bounce()
        return

    # Use compass directly — avoids encoder-drift poisoning the heading loop
    compass_h     = get_heading()
    target_h      = PATROL_NORTH if patrol_heading > 0 else PATROL_SOUTH
    heading_error = math.atan2(math.sin(target_h - compass_h),
                               math.cos(target_h - compass_h))

    if abs(heading_error) < 0.2:
        line_offset = get_patrol_line_offset()
        correction  = line_offset * 3.0
        set_wheel_speeds(MAX_SPEED + correction, MAX_SPEED - correction)
    else:
        set_wheel_speeds(MAX_SPEED * 0.5 - heading_error * 2.0,
                         MAX_SPEED * 0.5 + heading_error * 2.0)


# ── BT State: CHASE_INTRUDER ──────────────────────────────────
def run_chase_intruder():
    """
    Steer toward the intruder using their camera pixel offset.
    Proximity sensors confirm a tag when the attacker is within TAG_RANGE.
    """
    global intruder_tagged, approach_history, last_intruder_x, last_intruder_y

    offset = get_intruder_camera_offset()

    # Positive offset → intruder is right of centre → turn right (reduce left speed)
    left_speed  = MAX_SPEED - offset * 3.0
    right_speed = MAX_SPEED + offset * 3.0
    set_wheel_speeds(left_speed, right_speed)

    # Tag confirmed by proximity — attacker must re-spawn
    front_readings = [proximity_sensors[i].getValue() for i in [0, 7]]
    if any(v > OBSTACLE_THRESHOLD for v in front_readings):
        intruder_tagged = True
        # Record tag position so the friendly attacker can avoid this corridor next run
        last_intruder_x = pose_x
        last_intruder_y = pose_y
        # Record which side the attacker approached from for adaptive patrol
        if offset < 0:
            approach_history['left'] += 1
        else:
            approach_history['right'] += 1


# ── BT State: RETURN_TO_POST ──────────────────────────────────
def run_return_to_post():
    """Navigate back to spawn centre after chasing, then resume patrol."""
    global intruder_tagged

    if distance_to(*HOME_POST) < WAYPOINT_TOLERANCE:
        intruder_tagged = False
    else:
        steer_toward(*HOME_POST)


# ── BT State: RETURN_TO_PATROL ────────────────────────────────
def run_return_to_patrol():
    """
    Two-phase re-alignment after the robot has been displaced off the patrol line.

    Phase 1 — coarse (odometry):
        Steer toward x=PATROL_LINE_X at the robot's current y until within
        PATROL_LINE_CLOSE_THRESH.  Dead-reckoning gets us into the right
        neighbourhood without needing the camera.

    Phase 2 — seek_line (camera):
        Align heading to patrol_heading, then centre the red patrol-line marker
        in the camera frame.  Once SEEK_LINE_CONFIRM_STEPS consecutive centred
        readings are seen, re-zero odometry and clear the displaced flag.
        A SEEK_LINE_TIMEOUT fallback accepts the position and resets anyway so
        the robot cannot get stuck in this state if the line is not visible.
    """
    global displaced_from_patrol, return_to_patrol_phase
    global seek_line_timer, seek_line_confirm_count

    if return_to_patrol_phase == 'coarse':
        if abs(pose_x - PATROL_LINE_X) < PATROL_LINE_CLOSE_THRESH:
            # Close enough — hand off to camera fine-alignment
            return_to_patrol_phase  = 'seek_line'
            seek_line_timer         = 0.0
            seek_line_confirm_count = 0
            print(f"[RTP]   coarse done  x={pose_x:.3f} → seek_line")
        else:
            # Steer laterally toward the patrol line at current y
            steer_toward(PATROL_LINE_X, pose_y)

    else:  # 'seek_line'
        seek_line_timer += dt

        # Timeout safety net — accept position and resume patrol
        if seek_line_timer > SEEK_LINE_TIMEOUT:
            print(f"[RTP]   seek_line timeout — accepting x={pose_x:.3f}")
            reset_pose_to(PATROL_LINE_X, pose_y, patrol_heading)
            displaced_from_patrol  = False
            return_to_patrol_phase = 'coarse'
            return

        compass_h     = get_heading()
        target_h      = PATROL_NORTH if patrol_heading > 0 else PATROL_SOUTH
        heading_error = math.atan2(math.sin(target_h - compass_h),
                                   math.cos(target_h - compass_h))

        if abs(heading_error) > 0.2:
            # Align heading first so the camera view is meaningful
            set_wheel_speeds(MAX_SPEED * 0.4 - heading_error * 2.0,
                             MAX_SPEED * 0.4 + heading_error * 2.0)
            seek_line_confirm_count = 0
        else:
            # Heading OK — use camera to centre the patrol line
            offset = get_patrol_line_offset()
            if abs(offset) < 0.15:
                seek_line_confirm_count += 1
                set_wheel_speeds(MAX_SPEED * 0.3, MAX_SPEED * 0.3)
                if seek_line_confirm_count >= SEEK_LINE_CONFIRM_STEPS:
                    # Line confirmed centred — re-zero and return to patrol
                    print(f"[RTP]   line aligned  x={pose_x:.3f} — resuming patrol")
                    reset_pose_to(PATROL_LINE_X, pose_y, patrol_heading)
                    displaced_from_patrol  = False
                    return_to_patrol_phase = 'coarse'
            else:
                seek_line_confirm_count = 0
                correction = offset * 3.0
                set_wheel_speeds(MAX_SPEED * 0.3 + correction,
                                 MAX_SPEED * 0.3 - correction)


# ── BT Priority Selector ──────────────────────────────────────
def select_state(readings):
    """
    Evaluate conditions from highest to lowest priority.
    The first condition that is True wins — lower states only run
    when every higher-priority condition is inactive.
    """
    global avoidance_timer, displaced_from_patrol, return_to_patrol_phase

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

    # Priority 5 — re-align with the patrol line if displaced
    # Flag is set here so displacement is detected once then owned by RETURN_TO_PATROL
    # until it explicitly clears it (prevents oscillation around the threshold).
    if not displaced_from_patrol and abs(pose_x - PATROL_LINE_X) > PATROL_LINE_NEAR_THRESH:
        displaced_from_patrol  = True
        return_to_patrol_phase = 'coarse'  # always start fresh from coarse phase
    if displaced_from_patrol:
        return State.RETURN_TO_PATROL

    # Default — patrol the home zone
    return State.PATROL_ZONE


# ── Startup Calibration ───────────────────────────────────────
# Step once to populate sensor buffers, then lock patrol targets in compass
# frame and seed pose_theta so x/y odometry integrates in the right direction.
robot.step(timestep)
_h          = get_heading()
pose_theta  = math.atan2(math.sin(_h), math.cos(_h))   # seed odometry heading
PATROL_NORTH = _h                                        # compass frame north target
PATROL_SOUTH = (_h + math.pi) % (2 * math.pi)           # compass frame south target
print(f"[INIT] compass={math.degrees(_h):.1f}°  "
      f"PATROL_NORTH={math.degrees(PATROL_NORTH):.1f}°  "
      f"PATROL_SOUTH={math.degrees(PATROL_SOUTH):.1f}°")

# ── Main Control Loop ─────────────────────────────────────────

while robot.step(timestep) != -1:
    sim_time += dt

    update_odometry()
    broadcast_status()
    check_teammate_comms()

    proximity_readings = read_proximity()
    current_state      = select_state(proximity_readings)

    if current_state != prev_state:
        phase = f"  phase={return_to_patrol_phase}" if current_state == State.RETURN_TO_PATROL else ""
        print(f"[STATE] {prev_state.name if prev_state else 'START'} -> {current_state.name}"
              f"  x={pose_x:.3f}  y={pose_y:.3f}  θ={math.degrees(pose_theta):.1f}°{phase}")
        prev_state = current_state

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
        run_patrol_zone(proximity_readings)
    elif current_state == State.CHASE_INTRUDER:
        run_chase_intruder()
    elif current_state == State.RETURN_TO_POST:
        run_return_to_post()
    elif current_state == State.RETURN_TO_PATROL:
        run_return_to_patrol()
