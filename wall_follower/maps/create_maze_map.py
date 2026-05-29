import math

resolution = 0.05
width_m = 12.0
height_m = 12.0
origin_x = -6.0
origin_y = -6.0

width = int(width_m / resolution)
height = int(height_m / resolution)

# 254 = free, 0 = occupied
grid = [[254 for _ in range(width)] for _ in range(height)]

def world_to_pixel(x, y):
    col = int((x - origin_x) / resolution)
    row = height - 1 - int((y - origin_y) / resolution)
    return row, col

def draw_wall(cx, cy, sx, sy):
    x_min = cx - sx / 2.0
    x_max = cx + sx / 2.0
    y_min = cy - sy / 2.0
    y_max = cy + sy / 2.0

    r1, c1 = world_to_pixel(x_min, y_max)
    r2, c2 = world_to_pixel(x_max, y_min)

    r_min = max(0, min(r1, r2))
    r_max = min(height - 1, max(r1, r2))
    c_min = max(0, min(c1, c2))
    c_max = min(width - 1, max(c1, c2))

    for r in range(r_min, r_max + 1):
        for c in range(c_min, c_max + 1):
            grid[r][c] = 0

# These match the simple_world.world walls
walls = [
    # outer walls
    (0, 6, 12, 0.2),
    (0, -6, 12, 0.2),
    (-6, 0, 0.2, 12),
    (6, 0, 0.2, 12),

    # internal walls
    (-3, -2, 0.2, 4),
    (-4.75, -1, 2.5, 0.2),
    (-1, 1, 0.2, 8),
    (0, -3, 4, 0.2),
    (3, -1, 0.2, 4),
    (4, 2, 2.5, 0.2),
    (4.75, 3.25, 0.2, 2.5),
]

for wall in walls:
    draw_wall(*wall)

with open('/work/wall_follower/maps/maze_map.pgm', 'wb') as f:
    f.write(f'P5\n{width} {height}\n255\n'.encode())
    for row in grid:
        f.write(bytes(row))

with open('/work/wall_follower/maps/maze_map.yaml', 'w') as f:
    f.write("""image: maze_map.pgm
mode: trinary
resolution: 0.05
origin: [-6.0, -6.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
""")

print("Created maze_map.pgm and maze_map.yaml")
