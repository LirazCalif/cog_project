"""
generate_map.py
----------------
Pure occupancy map generator from a Webots .wbt world file
(no sensors, no simulation).

World: hospital.wbt
Map plane: X–Y
"""

import re
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation

# =========================================================
# MAP CONFIGURATION
# =========================================================

MAP_RESOLUTION = 0.5   # meters per cell (5 cm)

WORLD_X_MIN = -10
WORLD_X_MAX = 10
WORLD_Y_MIN = -10
WORLD_Y_MAX = 10

ROBOT_RADIUS = 0.35     # Fetch robot base radius (meters)

WORLD_FILE = "hospital.wbt"
OUTPUT_MAP = "hospital_occupancy_xy.csv"
OUTPUT_MAP_INFLATED = "hospital_occupancy_xy_inflated.csv"

# =========================================================
# DERIVED PARAMETERS
# =========================================================

MAP_WIDTH = int((WORLD_X_MAX - WORLD_X_MIN) / MAP_RESOLUTION)
MAP_HEIGHT = int((WORLD_Y_MAX - WORLD_Y_MIN) / MAP_RESOLUTION)

print(f"Map size: {MAP_WIDTH} x {MAP_HEIGHT}")

# =========================================================
# GRID INITIALIZATION
# =========================================================

occupancy_grid = np.zeros((MAP_WIDTH, MAP_HEIGHT), dtype=np.uint8)

# =========================================================
# COORDINATE CONVERSION
# =========================================================

def world_to_grid(x, y):
    gx = int((x - WORLD_X_MIN) / MAP_RESOLUTION)
    gy = int((y - WORLD_Y_MIN) / MAP_RESOLUTION)
    return gx, gy

# =========================================================
# WBT PARSER (BOX GEOMETRY)
# =========================================================

def extract_box_solids(wbt_path):
    """
    Extracts all Solid nodes with Box geometry.
    Uses X–Y plane (Z ignored).
    Returns: [(tx, ty, sx, sy), ...]
    """
    with open(wbt_path, "r", encoding="utf-8") as f:
        text = f.read()

    boxes = []

    pattern = re.compile(
        r"Solid\s*{[^}]*?"
        r"translation\s+([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+)[^}]*?"
        r"Box\s*{\s*size\s+([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+)",
        re.DOTALL
    )

    for m in pattern.finditer(text):
        tx, ty, tz, sx, sy, sz = map(float, m.groups())

        # We use X and Y only
        boxes.append((tx, ty, sx, sy))

    print(f"Detected {len(boxes)} box obstacles")
    return boxes

# =========================================================
# RASTERIZATION
# =========================================================

def mark_box(tx, ty, sx, sy):
    x_min = tx - sx / 2
    x_max = tx + sx / 2
    y_min = ty - sy / 2
    y_max = ty + sy / 2

    gx_min, gy_min = world_to_grid(x_min, y_min)
    gx_max, gy_max = world_to_grid(x_max, y_max)

    for gx in range(gx_min, gx_max + 1):
        for gy in range(gy_min, gy_max + 1):
            if 0 <= gx < MAP_WIDTH and 0 <= gy < MAP_HEIGHT:
                occupancy_grid[gx, gy] = 1

# =========================================================
# MAIN PIPELINE
# =========================================================

def main():
    boxes = extract_box_solids(WORLD_FILE)

    for box in boxes:
        mark_box(*box)

    # Save raw map
    np.savetxt(OUTPUT_MAP, occupancy_grid, delimiter=",", fmt="%d")
    print(f"Saved raw occupancy map: {OUTPUT_MAP}")

    # Inflate obstacles
    inflate_cells = int(ROBOT_RADIUS / MAP_RESOLUTION)
    inflated = binary_dilation(
        occupancy_grid,
        iterations=inflate_cells
    ).astype(np.uint8)

    np.savetxt(OUTPUT_MAP_INFLATED, inflated, delimiter=",", fmt="%d")
    print(f"Saved inflated map: {OUTPUT_MAP_INFLATED}")

    # Visualization
    plt.figure(figsize=(7, 7))
    plt.imshow(inflated.T, origin="lower", cmap="gray")
    plt.title("Hospital Occupancy Grid (X–Y, Inflated)")
    plt.xlabel("X grid")
    plt.ylabel("Y grid")
    plt.tight_layout()
    plt.show()

# =========================================================
# ENTRY POINT
# =========================================================

if __name__ == "__main__":
    main()
