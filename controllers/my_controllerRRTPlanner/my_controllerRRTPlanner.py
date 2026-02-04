# rrt_supervisor_controller.py
import math
import random
import csv
import matplotlib.pyplot as plt
from controller import Supervisor

# ==============================
# -------- PARAMETERS ----------
# ==============================

MAP_FILE = "hospital_occupancy_inflated.csv"
MAP_RESOLUTION = 0.5

MAX_RRT_ITER = 6000
STEP_SIZE = 1

WHEEL_RADIUS = 0.05
AXLE_LENGTH = 0.32
MAX_SPEED = 6.0

GOAL_X = -8.0
GOAL_Y = -4.0

WORLD_MIN = -10.0
WORLD_MAX = 10.0

# ==============================
# -------- RRT NODE ------------
# ==============================

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

# ==============================
# -------- MAP UTILS -----------
# ==============================

def load_map(filename):
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        return [[int(c) for c in row] for row in reader]

def world_to_grid(x, y):
    gx = int((x - WORLD_MIN) / MAP_RESOLUTION)
    gy = int((y - WORLD_MIN) / MAP_RESOLUTION)
    return gx, gy

def grid_to_world(gx, gy):
    x = WORLD_MIN + gx * MAP_RESOLUTION
    y = WORLD_MIN + gy * MAP_RESOLUTION
    return x, y

def is_free(x, y):
    if x < 0 or y < 0 or x >= map_w or y >= map_h:
        return False
    return grid[y][x] == 0

# ==============================
# -------- RRT -----------------
# ==============================

def rrt(start, goal):
    nodes = [Node(start[0], start[1])]

    for _ in range(MAX_RRT_ITER):
        rand = Node(
            random.randint(0, map_w - 1),
            random.randint(0, map_h - 1)
        )

        nearest = min(nodes, key=lambda n: math.hypot(n.x - rand.x, n.y - rand.y))
        theta = math.atan2(rand.y - nearest.y, rand.x - nearest.x)

        new_x = int(nearest.x + STEP_SIZE * math.cos(theta))
        new_y = int(nearest.y + STEP_SIZE * math.sin(theta))

        if not is_free(new_x, new_y):
            continue

        new_node = Node(new_x, new_y, nearest)
        nodes.append(new_node)

        if math.hypot(new_x - goal[0], new_y - goal[1]) < 2:
            return extract_path(Node(goal[0], goal[1], new_node))

    return None

def extract_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

# ==============================
# -------- MOTION --------------
# ==============================

def normalize(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def goto_waypoint(wx, wy):
    while supervisor.step(timestep) != -1:
        pos = robot_node.getPosition()
        rot = robot_node.getOrientation()

        rx, ry = pos[0], pos[1]
        heading = math.atan2(rot[3], rot[0])

        dx, dy = wx - rx, wy - ry
        dist = math.hypot(dx, dy)

        if dist < 0.15:
            l_motor.setVelocity(0)
            r_motor.setVelocity(0)
            return

        target = math.atan2(dy, dx)
        err = normalize(target - heading)

        v = 2.0
        w = 4.0 * err

        left = (v - w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        right = (v + w * AXLE_LENGTH / 2) / WHEEL_RADIUS

        l_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, left)))
        r_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, right)))

        # ---- LIVE WORLD UPDATE ----
        robot_dot.set_data([rx], [ry])
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

# ==============================
# -------- MAIN ----------------
# ==============================

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

l_motor = supervisor.getDevice("l_wheel_joint")
r_motor = supervisor.getDevice("r_wheel_joint")
l_motor.setPosition(float('inf'))
r_motor.setPosition(float('inf'))

robot_node = supervisor.getSelf()

grid = load_map(MAP_FILE)
map_h = len(grid)
map_w = len(grid[0])

start = world_to_grid(
    robot_node.getPosition()[0],
    robot_node.getPosition()[1]
)
goal = world_to_grid(GOAL_X, GOAL_Y)

print("🔍 Planning RRT...")
path = rrt(start, goal)

if not path:
    print("❌ RRT failed")
    exit()

print(f"✅ Path found with {len(path)} points")

# ==============================
# -------- MATPLOTLIB ----------
# ==============================

plt.ion()
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_title("RRT Path Planning (World Coordinates)")
ax.set_aspect("equal")

extent = [WORLD_MIN, WORLD_MAX, WORLD_MIN, WORLD_MAX]
ax.imshow(grid, cmap="gray_r", origin="lower", extent=extent)

# Path in WORLD coordinates
wx_path, wy_path = zip(*[grid_to_world(x, y) for x, y in path])
ax.plot(wx_path, wy_path, 'b-', linewidth=2, label="RRT Path")

ax.plot(*grid_to_world(*start), 'co', markersize=8, label="Start")
ax.plot(GOAL_X, GOAL_Y, 'go', markersize=10, label="Goal")

robot_dot, = ax.plot(
    [robot_node.getPosition()[0]],
    [robot_node.getPosition()[1]],
    'ro', markersize=6, label="Robot"
)

ax.set_xlim(WORLD_MIN, WORLD_MAX)
ax.set_ylim(WORLD_MIN, WORLD_MAX)
ax.legend()
plt.show(block=False)

# ==============================
# -------- EXECUTE PATH --------
# ==============================

for gx, gy in path:
    wx, wy = grid_to_world(gx, gy)
    goto_waypoint(wx, wy)

l_motor.setVelocity(0)
r_motor.setVelocity(0)
print("🎯 Goal reached")

plt.ioff()
plt.show()
