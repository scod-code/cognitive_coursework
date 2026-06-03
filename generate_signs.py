#!/usr/bin/env python3
"""
Generate realistic traffic sign textures for Gazebo simulation.
Run this inside WSL from your ros2_coursework_ws directory.

Usage:
    python3 generate_signs.py
"""

from PIL import Image, ImageDraw, ImageFont
import os
import math
import sys

SIZE = 512  # texture resolution

FONT_PATHS = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-B.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
]


def get_font(size):
    for path in FONT_PATHS:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    print("WARNING: No bold TTF font found, using PIL default (low quality)")
    return ImageFont.load_default()


def centered_text(draw, y, text, font, fill, image_width=SIZE):
    bbox = draw.textbbox((0, 0), text, font=font)
    w = bbox[2] - bbox[0]
    x = (image_width - w) // 2
    draw.text((x, y), text, fill=fill, font=font)


def draw_octagon(draw, cx, cy, radius, fill, outline=None, outline_width=8):
    points = []
    for i in range(8):
        # offset by pi/8 so octagon has flat top/bottom (standard stop sign orientation)
        angle = math.pi / 8 + i * math.pi / 4
        points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    if outline:
        draw.polygon(points, fill=fill, outline=outline)
        # PIL outline on polygon is thin; draw thicker border manually
        for _ in range(outline_width):
            inner = [(cx + (radius - _) * math.cos(math.pi/8 + i*math.pi/4),
                      cy + (radius - _) * math.sin(math.pi/8 + i*math.pi/4))
                     for i in range(8)]
        draw.polygon(points, fill=fill)
        draw.line(points + [points[0]], fill=outline, width=outline_width)
    else:
        draw.polygon(points, fill=fill)


# ---------------------------------------------------------------------------
# STOP SIGN  ——  red octagon, white border, white "STOP" text
# Poster background is white so it contrasts against the red arena wall
# ---------------------------------------------------------------------------
def create_stop_sign(filepath):
    img = Image.new("RGB", (SIZE, SIZE), (255, 255, 255))   # white poster background
    draw = ImageDraw.Draw(img)
    cx, cy = SIZE // 2, SIZE // 2

    # Outer white ring (creates the white octagon border)
    draw_octagon(draw, cx, cy, 238, fill=(255, 255, 255))
    # Red filled octagon inside
    draw_octagon(draw, cx, cy, 210, fill=(196, 2, 2))
    # Thin white inner ring for depth
    draw_octagon(draw, cx, cy, 210, fill=(196, 2, 2), outline=(255, 255, 255), outline_width=6)

    # "STOP" text centered in octagon
    font = get_font(115)
    bbox = draw.textbbox((0, 0), "STOP", font=font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    draw.text(((SIZE - tw) // 2, (SIZE - th) // 2 - 8), "STOP", fill=(255, 255, 255), font=font)

    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    img.save(filepath)
    print(f"  ✅  stop_sign.png  →  {filepath}")


# ---------------------------------------------------------------------------
# FAST SIGN  ——  circular green sign, solid upward arrow + "FAST"
# ---------------------------------------------------------------------------
def create_fast_sign(filepath):
    img = Image.new("RGB", (SIZE, SIZE), (30, 130, 30))
    draw = ImageDraw.Draw(img)
    cx, cy = SIZE // 2, SIZE // 2

    draw.ellipse([20, 20, SIZE-20, SIZE-20], outline=(255, 255, 255), width=18)

    arrow_pts = [
        (cx,       70),
        (cx - 85,  210),
        (cx - 36,  210),
        (cx - 36,  350),
        (cx + 36,  350),
        (cx + 36,  210),
        (cx + 85,  210),
    ]
    draw.polygon(arrow_pts, fill=(255, 255, 255))

    font = get_font(80)
    centered_text(draw, 362, "FAST", font, (255, 255, 255))

    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    img.save(filepath)
    print(f"  ✅  fast_sign.png  →  {filepath}")


# ---------------------------------------------------------------------------
# SLOW SIGN  ——  diamond warning sign (amber), text inside diamond
# ---------------------------------------------------------------------------
def create_slow_sign(filepath):
    img = Image.new("RGB", (SIZE, SIZE), (255, 200, 0))
    draw = ImageDraw.Draw(img)
    cx, cy = SIZE // 2, SIZE // 2

    margin = 28
    diamond = [
        (cx,          margin),
        (SIZE-margin, cy),
        (cx,          SIZE-margin),
        (margin,      cy),
    ]
    draw.polygon(diamond, fill=(255, 200, 0))
    draw.line(diamond + [diamond[0]], fill=(0, 0, 0), width=16)

    font_exc = get_font(140)
    bbox = draw.textbbox((0, 0), "!", font=font_exc)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    draw.text(((SIZE - tw) // 2, cy - th - 30), "!", fill=(0, 0, 0), font=font_exc)

    font = get_font(88)
    bbox = draw.textbbox((0, 0), "SLOW", font=font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    draw.text(((SIZE - tw) // 2, cy + 18), "SLOW", fill=(0, 0, 0), font=font)

    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    img.save(filepath)
    print(f"  ✅  slow_sign.png  →  {filepath}")


# ---------------------------------------------------------------------------
# Main — resolve output paths relative to this script or a given base
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    if len(sys.argv) > 1:
        base = sys.argv[1]
    else:
        base = os.path.join(
            os.path.expanduser("~"),
            "ros2_coursework_ws",
            "simple_robot_description",
            "models",
        )

    if not os.path.isdir(base):
        print(f"ERROR: models directory not found at '{base}'")
        print("Usage: python3 generate_signs.py /path/to/models/")
        sys.exit(1)

    print(f"\nGenerating sign textures into: {base}\n")

    create_stop_sign(os.path.join(base, "stop_sign_poster/materials/textures/stop_sign.png"))
    create_fast_sign(os.path.join(base, "fast_sign_poster/materials/textures/fast_sign.png"))
    create_slow_sign(os.path.join(base, "slow_sign_poster/materials/textures/slow_sign.png"))

    print("\nDone. Rebuild the package to apply textures:")
    print("  cd ~/ros2_coursework_ws")
    print("  colcon build --packages-select simple_robot_description")
    print("  source install/setup.bash")
