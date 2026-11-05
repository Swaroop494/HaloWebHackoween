# Tunable constants
ROOM_SIZE = 6.0
WALL_HEIGHT = 1.0
WALL_THICK = 0.1

# Original colors
FLOOR_COLOR = [0.914, 0.863, 0.765, 1.0]  # sand
WALL_COLOR = [0.969, 0.953, 0.918, 1.0]   # cream
DROP_ZONE_COLOR = [0.561, 0.682, 0.545, 1.0]  # original green
DROP_ZONE_BORDER = [0.369, 0.498, 0.357, 1.0]
ROBOT_COLOR = [0.227, 0.247, 0.267, 1.0]
ROBOT_MARKER = [1.0, 0.973, 0.906, 1.0]

# Premium environment colors (for upgraded look)
PREMIUM_FLOOR_COLOR = [0.20, 0.20, 0.22, 1.0]     # Deep graphite
FLOOR_GRID_COLOR = [0.25, 0.25, 0.27, 0.3]        # Subtle grid lines
PREMIUM_WALL_COLOR = [0.94, 0.93, 0.91, 1.0]      # Warm off-white
WALL_STRUCTURAL_COLOR = [0.18, 0.18, 0.18, 1.0]   # Charcoal ribs
PREMIUM_DROP_ZONE_COLOR = [0.08, 0.55, 0.52, 1.0] # Cyan/teal
DROP_ZONE_BORDER_PREMIUM = [0.08, 0.55, 0.52, 1.0] # Cyan perimeter
PREMIUM_DROP_ZONE_INDICATOR_COLOR = [0.12, 0.88, 0.82, 1.0]
LED_CYAN = [0.0, 0.80, 0.80, 0.5]                 # Cyan LED accent
SAFETY_YELLOW = [0.95, 0.75, 0.0, 1.0]            # Industrial amber
SAFETY_BLACK = [0.08, 0.08, 0.08, 1.0]            # Black safety
METALLIC_ACCENT = [0.50, 0.52, 0.54, 1.0]         # Brushed aluminium

PALETTE = [
    ([0.878, 0.478, 0.373, 1.0], "coral"),
    ([0.239, 0.545, 0.545, 1.0], "teal"),
    ([0.914, 0.706, 0.298, 1.0], "mustard"),
    ([0.490, 0.353, 0.498, 1.0], "plum"),
    ([0.435, 0.659, 0.863, 1.0], "sky blue"),
    ([0.247, 0.306, 0.549, 1.0], "indigo"),
]

OBSTACLE_BOX_COLOR = [0.847, 0.784, 0.659, 1.0]
OBSTACLE_VASE_COLOR = [0.624, 0.706, 0.780, 1.0]

ROBOT_RADIUS = 0.15
ROBOT_HEIGHT = 0.4
CAMERA_HEIGHT = 0.5
CAMERA_FORWARD = 0.15
CAMERA_TILT_DEG = 10.0
CAMERA_FOV_H = 90.0
CAMERA_WIDTH = 640
CAMERA_HEIGHT_PIX = 480

MOVE_SPEED = 1.0
TURN_SPEED_DEG = 90.0
PHYSICS_STEP = 1.0/240.0

PICKUP_REACH = 0.5
PICKUP_ANGLE_DEG = 30.0
CARRY_OFFSET_X = 0.45
CARRY_OFFSET_Z = 0.45
DROP_ZONE_SIZE = 1.0
PREMIUM_DROP_ZONE_SIZE = 2.6

MIN_OBJ_DIST = 0.8
MIN_OBJ_ROBOT_DIST = 1.0
MIN_ROBOT_WALL_DIST = 1.5
MIN_ROBOT_DROP_DIST = 1.5

MAX_HOLD = 1
SOFT_STEPS = 25
HARD_STEPS = 60
BLOCKED_RETRIES = 3
IDLE_TIMEOUT_S = 2.0

# Fetch task settings; scene definitions stay engine-private.
FETCH_OBJECT_COUNT = 6
FETCH_SHAPES = ("box", "sphere", "cylinder", "cone", "box", "sphere")
FETCH_OBJECT_MIN_SIZE = 0.22
FETCH_OBJECT_MAX_SIZE = 0.34
FETCH_OBJECT_MIN_HEIGHT = 0.22
FETCH_OBJECT_MAX_HEIGHT = 0.55
FETCH_MOVE_SPEED = 0.5
ROBOT_BASE_Z = 0.1
CAMERA_BASE_OFFSET_Z = CAMERA_HEIGHT - ROBOT_BASE_Z
