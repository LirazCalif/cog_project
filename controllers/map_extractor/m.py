from controller import Supervisor
import numpy as np
import matplotlib.pyplot as plt
import math
import random

# MAP CONFIG

MAP_RESOLUTION = 0.5
WORLD_X_MIN, WORLD_X_MAX = -10.0, 10.0
WORLD_Y_MIN, WORLD_Y_MAX = -10.0, 10.0
ROBOT_RADIUS = 0.35

# Joint Cofigurations

HEAD_PAN_TARGET = math.pi
HEAD_TILT_TARGET = -0.2
TORSO_TARGET = 0.05
GRIPPER_TARGET = 0.09

# DERIVED

MAP_WIDTH  = int((WORLD_X_MAX - WORLD_X_MIN) / MAP_RESOLUTION)
MAP_HEIGHT = int((WORLD_Y_MAX - WORLD_Y_MIN) / MAP_RESOLUTION)

occupancy = np.zeros((MAP_WIDTH, MAP_HEIGHT), dtype=np.uint8)

# UTILITIES

def world_to_grid(x, y):
    return (
        int((x - WORLD_X_MIN) / MAP_RESOLUTION),
        int((y - WORLD_Y_MIN) / MAP_RESOLUTION)
    )

def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

def rotate_point(px, py, cx, cy, angle):
    s, c = math.sin(angle), math.cos(angle)
    px -= cx; py -= cy
    return px*c - py*s + cx, px*s + py*c + cy

def get_yaw_from_rotation(rot):
   
    ax, ay, az, ang = rot
    if abs(az) < 1e-6:
        return 0.0
    # If axis is not normalized, normalize
    mag = math.sqrt(ax*ax + ay*ay + az*az)
    ax, ay, az = ax/mag, ay/mag, az/mag
    # yaw is just the angle around z-axis
    return normalize_angle(ang * az)

# SCENE TRAVERSAL

def find_shapes(node, shapes):
    if node is None:
        return
    if node.getTypeName() == "Shape":
        shapes.append(node)
    try:
        f = node.getField("children")
        if f:
            for i in range(f.getCount()):
                find_shapes(f.getMFNode(i), shapes)
    except:
        pass

def set_position(robot, name, position, velocity=None):
    dev = robot.getDevice(name)
    if dev:
        dev.setPosition(position)
        if velocity is not None:
            dev.setVelocity(velocity)

def traverse(root, obstacles):
    visited = set()
    stack = [root]

    while stack:
        node = stack.pop()
        if node.getId() in visited:
            continue
        visited.add(node.getId())

        if node.getTypeName() == "Solid":
            pos = node.getPosition()
            if abs(pos[2]) < 1e-6:
                angle = node.getField("rotation").getSFRotation()[3]
                shapes = []
                find_shapes(node, shapes)

                for s in shapes:
                    try:
                        geom = s.getField("geometry").getSFNode()
                        if geom and geom.getTypeName() == "Box":
                            size = geom.getField("size").getSFVec3f()
                            obstacles.append((pos, size, angle))
                    except:
                        pass

        try:
            f = node.getField("children")
            if f:
                for i in range(f.getCount()):
                    stack.append(f.getMFNode(i))
        except:
            pass


def mark_box(pos, size, angle):
    x, y, _ = pos
    sx, sy, _ = size

    corners = [(-sx/2,-sy/2),(-sx/2,sy/2),(sx/2,sy/2),(sx/2,-sy/2)]
    world = [rotate_point(x+cx, y+cy, x, y, angle) for cx,cy in corners]

    xs = [p[0] for p in world]
    ys = [p[1] for p in world]

    gx0, gy0 = world_to_grid(min(xs), min(ys))
    gx1, gy1 = world_to_grid(max(xs), max(ys))

    for gx in range(gx0, gx1+1):
        for gy in range(gy0, gy1+1):
            if 0 <= gx < MAP_WIDTH and 0 <= gy < MAP_HEIGHT:
                occupancy[gx, gy] = 1

# RRT HELPERS

def is_free(x, y, grid):
    gx, gy = world_to_grid(x, y)
    if 0 <= gx < MAP_WIDTH and 0 <= gy < MAP_HEIGHT:
        return grid[gx, gy] == 0
    return False

def line_free(p1, p2, grid):
    steps = int(math.hypot(p2[0]-p1[0], p2[1]-p1[1]) / (MAP_RESOLUTION/2))
    for i in range(steps+1):
        t = i / max(steps,1)
        x = p1[0] + t*(p2[0]-p1[0])
        y = p1[1] + t*(p2[1]-p1[1])
        if not is_free(x,y,grid):
            return False
    return True

def rrt(start, goal, grid, step=1.2, max_iter=3000):
    nodes = [start]
    parent = {0:None}

    for _ in range(max_iter):
        rand = goal if random.random()<0.35 else (
            random.uniform(WORLD_X_MIN,WORLD_X_MAX),
            random.uniform(WORLD_Y_MIN,WORLD_Y_MAX))

        i = min(range(len(nodes)),
                key=lambda k: math.hypot(nodes[k][0]-rand[0], nodes[k][1]-rand[1]))

        x,y = nodes[i]
        th = math.atan2(rand[1]-y, rand[0]-x)
        new = (x+step*math.cos(th), y+step*math.sin(th))

        if not is_free(new[0],new[1],grid): continue
        if not line_free(nodes[i], new, grid): continue

        nodes.append(new)
        parent[len(nodes)-1]=i

        if math.hypot(new[0]-goal[0],new[1]-goal[1])<step:
            nodes.append(goal)
            parent[len(nodes)-1]=len(nodes)-2
            break

    path=[]
    i=len(nodes)-1
    while i is not None:
        path.append(nodes[i])
        i=parent[i]
    return path[::-1]

def prune_path(path, grid):
    out=[path[0]]
    i=0
    while i<len(path)-1:
        j=len(path)-1
        while j>i+1 and not line_free(path[i],path[j],grid):
            j-=1
        out.append(path[j])
        i=j
    return out

def main():
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    dt = timestep / 1000.0

    # MAP
    obstacles = []
    traverse(supervisor.getRoot(), obstacles)

    manual_tables = [
        ([-4.56,-5.53,0.0],[1.0,4.0,0.74],-1.5708),
        ([-7.06,-6.29,0.0],[1.0,2.5,0.74], 3.14159),
    ]
    obstacles.extend(manual_tables)

    for pos,size,angle in obstacles:
        mark_box(pos,size,angle)

    inflated = occupancy.copy()
    inflate = int(ROBOT_RADIUS / MAP_RESOLUTION)
    for _ in range(inflate):
        inflated = np.maximum(inflated, np.roll(inflated,1,axis=0))
        inflated = np.maximum(inflated, np.roll(inflated,1,axis=1))

    # SETUP
    robot = supervisor.getFromDef("FETCH")
    start = robot.getPosition()[:2]

    # Arm config
    set_position(supervisor, "torso_lift_joint", TORSO_TARGET)
    set_position(supervisor, "head_pan_joint", HEAD_PAN_TARGET)
    set_position(supervisor, "head_tilt_joint", HEAD_TILT_TARGET)
    set_position(supervisor, "l_gripper_finger_joint", GRIPPER_TARGET)
    set_position(supervisor, "r_gripper_finger_joint", GRIPPER_TARGET)

    set_position(supervisor, "torso_lift_joint", TORSO_TARGET, -0.05)
    set_position(supervisor, "head_pan_joint", HEAD_PAN_TARGET, 0)
    set_position(supervisor, "head_tilt_joint", HEAD_TILT_TARGET, -0.5)
    set_position(supervisor, "wrist_flex_joint", -1.2, 0.5)
    set_position(supervisor, "head_tilt_joint", 0.6, 0.4)

    set_position(supervisor, "l_gripper_finger_joint", GRIPPER_TARGET, 0.02)
    set_position(supervisor, "r_gripper_finger_joint", GRIPPER_TARGET, 0.02)
    set_position(supervisor, "bellows_joint", 0.0, 0.2)
    set_position(supervisor, "torso_lift_joint", 0.5,0.5)
    
    t=0
    while supervisor.step(timestep) != -1 and t <= 2:
       t += timestep / 1000.0
    medicine = random.choice(["A", "B"])
    print("selected_medicine = ",medicine)
    t = 0

    gripper = supervisor.getFromDef("FETCH")
    if medicine == "A":
       obj = supervisor.getFromDef("medicineA")
    if medicine == "B":
       obj = supervisor.getFromDef("medicineB")

    object_translation_field = obj.getField("translation")
    gripper_translation_field = gripper.getField("translation")

    # PICK OBJECT
    while supervisor.step(timestep) != -1 and t <= 4 and medicine == 'B':
       t += timestep / 1000.0
       set_position(supervisor, "shoulder_pan_joint", -0.6, -0.5)

    while supervisor.step(timestep) != -1 and t <= 4 and medicine == 'A':
       t += timestep / 1000.0
       set_position(supervisor, "shoulder_pan_joint", -0.4, -0.5)

    t=0
    while supervisor.step(timestep) != -1 and t <= 2:
       t += timestep / 1000.0

    set_position(supervisor, "torso_lift_joint", TORSO_TARGET, 0.05)
    t=0

    while supervisor.step(timestep) != -1 and t <= 5:
       t += timestep / 1000.0

    set_position(supervisor, "wrist_flex_joint", 1, 0.5)
    t=0

    while supervisor.step(timestep) != -1 and t <= 2:
       t += timestep / 1000.0

    set_position(supervisor, "l_gripper_finger_joint", 0.0, 0.02)
    set_position(supervisor, "r_gripper_finger_joint", 0.0, 0.02)
    
    t=0
    while supervisor.step(timestep) != -1 and t <= 4:
       t += timestep / 1000.0
    t=0
    while supervisor.step(timestep) != -1 and t <= 4 and medicine == 'B':
       t += timestep / 1000.0
       set_position(supervisor, "wrist_flex_joint", -1.2,0.5)
    t=0
    while supervisor.step(timestep) != -1 and t <= 4 and medicine == 'B':
       t += timestep / 1000.0
       set_position(supervisor, "torso_lift_joint", 0.5,0.5)
    
    
    

    set_position(supervisor, "wrist_flex_joint", -1.2, 0.5)
    set_position(supervisor, "shoulder_pan_joint", 0, -0.5)
    set_position(supervisor, "wrist_flex_joint", 0, 0.5)
    
    t=0
    while supervisor.step(timestep) != -1 and t <= 2 and medicine == 'B':
       t += timestep / 1000.0

       set_position(supervisor, "torso_lift_joint", TORSO_TARGET, 0.05)
    
    
    
    t=0
    
    
    while supervisor.step(timestep) != -1 and t <= 4:
       t += timestep / 1000.0

    set_position(supervisor, "shoulder_pan_joint", 1, -0.5)

    t=0
    while supervisor.step(timestep) != -1 and t <= 3:
       t += timestep / 1000.0

    set_position(supervisor, "shoulder_lift_joint", -0.4, 0.5)
    set_position(supervisor, "elbow_flex_joint", 2.4, 0.5)

    t=0
    while supervisor.step(timestep) != -1 and t <= 5:
       t += timestep / 1000.0

    set_position(supervisor, "shoulder_lift_joint", 0.1, 0.5)
    t=0

    while supervisor.step(timestep) != -1 and t <= 3:
       t += timestep / 1000.0

    #obj.resetPhysics()
    robot_pos = np.array(robot.getPosition())
    object_translation_field.setSFVec3f((robot_pos).tolist())

    # planning
    goals=[(-1,-8),(3,-8),(7.2,-8),(7,8),(3,8),(-6,8)]
    goal=random.choice(goals)
    goal = goals[0]
    print("Goal:",goal)

    path = prune_path(rrt(start, goal, inflated), inflated)

    # ----------------- VIS -----------------
    plt.ion()
    fig, ax = plt.subplots(figsize=(8,8))
    ax.imshow(
        occupancy.T,
        origin="lower",
        extent=[WORLD_X_MIN,WORLD_X_MAX,WORLD_Y_MIN,WORLD_Y_MAX]
    )
    ax.plot([p[0] for p in path],[p[1] for p in path],'b',lw=2)

    # MOTION
    LINEAR_STEP = 0.1
    ROT_SPEED = 1.2
    MAX_CORR = 0.6
    ANG_EPS = 0.05
    DIST_EPS = 0.3

    idx = 1
    rotating = True

    while supervisor.step(timestep) != -1:
        robot_pos = np.array(robot.getPosition())
        object_translation_field.setSFVec3f((robot_pos).tolist())

        x,y = robot.getPosition()[:2]
        yaw = get_yaw_from_rotation(robot.getField("rotation").getSFRotation())

        if idx >= len(path):
            print("GOAL REACHED")
            break

        tx,ty = path[idx]
        desired = math.atan2(ty-y, tx-x)
        err = normalize_angle(desired - yaw)

        if rotating:
            if abs(err) < ANG_EPS:
                rotating = False
            else:
                yaw += math.copysign(ROT_SPEED*dt, err)
                yaw = normalize_angle(yaw)
                robot.getField("rotation").setSFRotation([0,0,1,yaw])
                continue

        dist = math.hypot(tx-x, ty-y)
        if dist < DIST_EPS:
            idx += 1
            rotating = True
            continue

        step_dist = min(LINEAR_STEP, dist)
        yaw += max(-MAX_CORR, min(MAX_CORR, err)) * dt
        yaw = normalize_angle(yaw)

        x += step_dist * math.cos(yaw)
        y += step_dist * math.sin(yaw)

        robot.getField("rotation").setSFRotation([0,0,1,yaw])
        robot.getField("translation").setSFVec3f([x,y,0.0])

        ax.scatter(x,y,c='r',s=25)
        fig.canvas.flush_events()

if __name__ == "__main__":
    main()
