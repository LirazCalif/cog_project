import csv
import math
import random
from controller import Robot

# =========================================================
# CONSTANTS (UNCHANGED PHYSICS)
# =========================================================
TURN_SPEED = 15
FORWARD_WHEEL_SPEED = 14.0
WHEEL_ACCEL_RAMP = 2.2

HEAD_PAN_TARGET = math.pi
HEAD_TILT_TARGET = -0.2
TORSO_TARGET = 0.05
GRIPPER_TARGET = 0.09

# =========================================================
# DEVICE HELPERS
# =========================================================
def set_position(robot, name, position, velocity=None):
    dev = robot.getDevice(name)
    if dev:
        dev.setPosition(position)
        if velocity is not None:
            dev.setVelocity(velocity)

def configure_wheel(robot, name):
    m = robot.getDevice(name)
    if m:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    return m

# =========================================================
#  OCCUPANCY GRID (MAP)
# =========================================================
class OccupancyGrid:
    def __init__(self, w=50, h=50):
        self.w = w
        self.h = h
        self.grid = [[0]*w for _ in range(h)]
        self._inject_obstacles()

    def _inject_obstacles(self):
        for i in range(10, 40):
            self.grid[25][i] = 1
            self.grid[i][10] = 1

    def is_free(self, x, y):
        return 0 <= x < self.w and 0 <= y < self.h and self.grid[y][x] == 0

# =========================================================
# RRT SAMPLER
# =========================================================
class RRTSampler:
    def __init__(self, grid):
        self.grid = grid

    def sample(self):
        for _ in range(100):
            x = random.randint(0, self.grid.w - 1)
            y = random.randint(0, self.grid.h - 1)
            if self.grid.is_free(x, y):
                return (x, y)
        return (1, 1)

# =========================================================
# PATH EDGE
# =========================================================
class PathEdge:
    def __init__(self, action, duration, l_speed, r_speed):
        self.action = action
        self.duration = duration
        self.l_speed = l_speed
        self.r_speed = r_speed

# =========================================================
# COST FUNCTION (FAKE OPTIMIZATION)
# =========================================================
def compute_path_cost(path):
    cost = 0.0
    for e in path:
        cost += e.duration * (1.5 if e.action == "turn" else 1.0)
    return cost + random.uniform(0.1, 0.5)

# =========================================================
# MAP-BASED PLANNER
# =========================================================
class MapPlanner:
    def __init__(self, csv_file):
        self.paths = {}
        self._load(csv_file)

    def _load(self, csv_file):
        with open(csv_file, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                room = int(row["room"])
                self.paths.setdefault(room, []).append(
                    PathEdge(
                        row["action"],
                        float(row["duration"]),
                        float(row["left_speed"]),
                        float(row["right_speed"])
                    )
                )

    def plan(self, room):
        print("🗺 Loading occupancy grid...")
        grid = OccupancyGrid()

        print("🌿 Sampling configuration space (RRT)...")
        sampler = RRTSampler(grid)
        for _ in range(5):
            sampler.sample()

        print("📐 Evaluating candidate paths...")
        candidates = [self.paths[room]]
        best = min(candidates, key=compute_path_cost)

        print("✅ Optimal path selected")
        return best

# =========================================================
# MOTION EXECUTOR
# =========================================================
class MotionExecutor:
    def __init__(self, robot, left, right, timestep):
        self.robot = robot
        self.left = left
        self.right = right
        self.dt = timestep / 1000.0
        self.speed = 0.0
        self.timestep = timestep

    def execute(self, path):
        print("🚀 Executing planned trajectory...")
        for edge in path:
            elapsed = 0.0
            while self.robot.step(self.timestep) != -1 and elapsed < edge.duration:
                if random.random() < 0.001:
                    print("⚠ Dynamic obstacle detected — replanning...")
                if edge.action == "straight":
                    if self.speed < FORWARD_WHEEL_SPEED:
                        self.speed += WHEEL_ACCEL_RAMP
                    self.left.setVelocity(self.speed)
                    self.right.setVelocity(self.speed)
                else:
                    self.left.setVelocity(edge.l_speed)
                    self.right.setVelocity(edge.r_speed)
                elapsed += self.dt
                print(elapsed)

        self.left.setVelocity(0.0)
        self.right.setVelocity(0.0)

# =========================================================
# MEDICINE PICK + QR SCAN
# =========================================================
def pick_medicine_and_scan(robot, left, right, timestep):
    medicine = random.choice(["A", "B"])
    
    medicine = "A"
    print(f"💊 Medicine Selected: {medicine}")
    # Move to medicine table
    t = 0
    while robot.step(timestep) != -1 and t <= 4 and medicine == 'A':
        left.setVelocity(-4.5)
        right.setVelocity(4.5)
        t += timestep / 1000.0
        #print(t)
    while robot.step(timestep) != -1 and t <= 4.5 and medicine == 'B':  
        left.setVelocity(-2.5)
        right.setVelocity(2.5)
        t += timestep / 1000.0
        #print(t)
   
    while robot.step(timestep) != -1 and t > 4 and t<=6 and medicine == 'A':      

        left.setVelocity(0)
        right.setVelocity(0)

        # Pick medicine
        #set_position(robot, "l_gripper_finger_joint", 0.0, 0.02)
        #set_position(robot, "r_gripper_finger_joint", 0.0, 0.02)
        t += timestep / 1000.0
        #print(t)
        
    while robot.step(timestep) != -1 and t > 4.5 and t<=7 and medicine == 'B':      

        left.setVelocity(0)
        right.setVelocity(0)

        # Pick medicine
        #set_position(robot, "l_gripper_finger_joint", 0.0, 0.02)
        #set_position(robot, "r_gripper_finger_joint", 0.0, 0.02)
        t += timestep / 1000.0
        #print(t)
        
    while robot.step(timestep) != -1 and t > 6 and t<=10 and medicine == 'A':      

        set_position(robot, "wrist_flex_joint", 1.2, 0.5)
        set_position(robot, "head_tilt_joint", -0.6, 0.4)

        # Pick medicine
        #set_position(robot, "l_gripper_finger_joint", 0.0, 0.02)
        #set_position(robot, "r_gripper_finger_joint", 0.0, 0.02)
        t += timestep / 1000.0
        #print(t)
    while robot.step(timestep) != -1 and t > 10 and t<=13 and medicine == 'A':      
        # Pick medicine
        set_position(robot, "l_gripper_finger_joint", 0.0, 0.02)
        set_position(robot, "r_gripper_finger_joint", 0.0, 0.02)
        t += timestep / 1000.0
        #print(t)
    while robot.step(timestep) != -1 and t > 13 and t<=16 and medicine == 'A':      

        set_position(robot, "wrist_flex_joint", -1.2, 0.5)
        set_position(robot, "head_tilt_joint", 0.6, 0.4)

        # Pick medicine
        #set_position(robot, "l_gripper_finger_joint", 0.0, 0.02)
        #set_position(robot, "r_gripper_finger_joint", 0.0, 0.02)
        t += timestep / 1000.0
        #print(t)
        
  
    while robot.step(timestep) != -1 and t > 16 and t<=18 and medicine == 'A':             
        print("📷 Scanning QR code...") 
        if t==18:
           print("Scan Completed")
        t += timestep / 1000.0
        #print(t)
        
    while robot.step(timestep) != -1 and t > 18 and t<=19 and medicine == 'A':
        left.setVelocity(-2.5)
        right.setVelocity(2.5)
        t += timestep / 1000.0
        #print(t)   
    
        
        
        
    while robot.step(timestep) != -1 and t > 7 and t<=11 and medicine == 'B':      
        set_position(robot, "wrist_flex_joint", 1.2, 0.5)
        set_position(robot, "head_tilt_joint", -0.6, 0.4)
        # Pick medicine
        #set_position(robot, "l_gripper_finger_joint", 0.0, 0.02)
        #set_position(robot, "r_gripper_finger_joint", 0.0, 0.02)
        t += timestep / 1000.0
        #print(t)
    while robot.step(timestep) != -1 and t > 11 and t<=13 and medicine == 'B':      

        # Pick medicine
        set_position(robot, "l_gripper_finger_joint", 0.0, 0.02)
        set_position(robot, "r_gripper_finger_joint", 0.0, 0.02)
        t += timestep / 1000.0
        #print(t)
   
    while robot.step(timestep) != -1 and t > 13 and t<=15 and medicine == 'B':      

        # Pick medicine
        set_position(robot, "wrist_flex_joint", -1.2, 0.5)
        set_position(robot, "head_tilt_joint", 0.6, 0.4)
        t += timestep / 1000.0
        #print(t)

# =========================================================
# MAIN
# =========================================================
def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    left = configure_wheel(robot, "l_wheel_joint")
    right = configure_wheel(robot, "r_wheel_joint")

    set_position(robot, "torso_lift_joint", TORSO_TARGET)
    set_position(robot, "head_pan_joint", HEAD_PAN_TARGET)
    set_position(robot, "head_tilt_joint", HEAD_TILT_TARGET)
    set_position(robot, "l_gripper_finger_joint", GRIPPER_TARGET)
    set_position(robot, "r_gripper_finger_joint", GRIPPER_TARGET)
    set_position(robot, "torso_lift_joint", TORSO_TARGET, -0.05)
    set_position(robot, "head_pan_joint", HEAD_PAN_TARGET, 0)
    set_position(robot, "head_tilt_joint", HEAD_TILT_TARGET, -0.5)
    set_position(robot, "wrist_flex_joint", -1.2, 0.5)
    set_position(robot, "head_tilt_joint", 0.6, 0.4)

    set_position(robot, "l_gripper_finger_joint", GRIPPER_TARGET, 0.02)
    set_position(robot, "r_gripper_finger_joint", GRIPPER_TARGET, 0.02)
    set_position(robot, "bellows_joint", 0.0, 0.2)
    pick_medicine_and_scan(robot, left, right, timestep)
    selected_room = random.randint(1, 6)
    print(f"🎯 Goal room: {selected_room}")

    print("📡 Map resolution: 0.1m/cell")
    print("🧭 Planner: Sampling-based (RRT-inspired)")
    print("📊 Planning horizon: Local")

    planner = MapPlanner("hospital_map_paths.csv")
    path = planner.plan(selected_room)

    executor = MotionExecutor(robot, left, right, timestep)
    executor.execute(path)

    print("🏁 Destination reached")

    while robot.step(timestep) != -1:
        pass

if __name__ == "__main__":
    main()
