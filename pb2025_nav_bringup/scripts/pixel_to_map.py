#!/usr/bin/env python3

import argparse
import math
import os
import re
from typing import Tuple, Optional, Dict, Any


def parse_map_yaml(yaml_path: str) -> Dict[str, Any]:
    """Parse a standard ROS map YAML without external dependencies.

    Expects keys at top-level like:
      image: <path>
      resolution: <float>
      origin: [x, y, yaw]
      negate: 0|1
      occupied_thresh: <float>
      free_thresh: <float>
    """
    content = {}
    list_pattern = re.compile(r"\[(.*?)\]")

    with open(yaml_path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if ":" not in line:
                continue
            key, val = line.split(":", 1)
            key = key.strip()
            val = val.strip()

            # Remove inline comments after value
            if "#" in val:
                val = val.split("#", 1)[0].strip()

            if key == "image":
                # strip quotes if present
                val = val.strip("'\"")
                content["image"] = val
            elif key == "resolution":
                content["resolution"] = float(val)
            elif key == "origin":
                m = list_pattern.search(val)
                if not m:
                    raise ValueError(f"Invalid origin format in {yaml_path}: {val}")
                parts = [p.strip() for p in m.group(1).split(",")]
                floats = [float(p) for p in parts]
                if len(floats) < 2:
                    raise ValueError("origin must have at least x, y")
                # pad yaw if missing
                if len(floats) == 2:
                    floats.append(0.0)
                content["origin"] = floats[:3]
            elif key == "negate":
                try:
                    content["negate"] = int(val)
                except Exception:
                    content["negate"] = 0
            elif key in ("occupied_thresh", "free_thresh"):
                try:
                    content[key] = float(val)
                except Exception:
                    pass
            elif key == "mode":
                content["mode"] = val

    # Basic validation
    for required in ("image", "resolution", "origin"):
        if required not in content:
            raise ValueError(f"Missing '{required}' in YAML: {yaml_path}")

    return content


def resolve_image_path(yaml_path: str, image_field: str) -> str:
    # If image_field is absolute, return it; else resolve relative to YAML directory
    if os.path.isabs(image_field):
        return image_field
    return os.path.normpath(os.path.join(os.path.dirname(yaml_path), image_field))


def resolve_yaml_path(yaml_arg: str) -> str:
    """Resolve YAML path allowing shorthand names.

    Resolution order:
    1) As-is (absolute or relative to CWD)
    2) Add .yaml / .yml extension if missing
    3) Search in package map directories:
       - <pkg_root>/map/simulation/
       - <pkg_root>/map/reality/
    where <pkg_root> is the parent directory of this script's folder.
    """
    candidates = []

    def with_exts(path_noext: str):
        if path_noext.lower().endswith(('.yaml', '.yml')):
            return [path_noext]
        return [path_noext + '.yaml', path_noext + '.yml']

    # 1) direct
    candidates.append(yaml_arg)
    # 2) try with extensions if user omitted
    candidates += with_exts(yaml_arg)

    # 3) search in map dirs relative to package root
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_root = os.path.dirname(script_dir)  # pb2025_nav_bringup
    map_dirs = [
        os.path.join(pkg_root, 'map', 'simulation'),
        os.path.join(pkg_root, 'map', 'reality'),
    ]

    for d in map_dirs:
        for p in with_exts(os.path.join(d, yaml_arg)):
            candidates.append(p)

    # Check existence
    for c in candidates:
        if os.path.isfile(c):
            return os.path.abspath(c)

    raise FileNotFoundError(
        f"Cannot find YAML '{yaml_arg}'. Tried: " + ", ".join(candidates)
    )


def read_pgm_header(img_path: str) -> Tuple[str, int, int, int, int]:
    """Read PGM header, return (magic, width, height, maxval, header_end_offset).

    Supports P2 (ASCII) and P5 (binary)."""
    with open(img_path, "rb") as f:
        def _next_token(fp) -> str:
            # skip whitespace and comments
            token = b""
            c = fp.read(1)
            while c:
                if c.isspace():
                    c = fp.read(1)
                    continue
                if c == b"#":
                    # comment until end of line
                    while c and c != b"\n":
                        c = fp.read(1)
                    c = fp.read(1)
                    continue
                break
            # now c is start of token
            while c and (not c.isspace() and c != b"#"):
                token += c
                c = fp.read(1)
            return token.decode("ascii")

        start_pos = f.tell()
        magic = _next_token(f)
        if magic not in ("P2", "P5"):
            raise ValueError(f"Unsupported PGM magic: {magic}")
        width = int(_next_token(f))
        height = int(_next_token(f))
        maxval = int(_next_token(f))
        header_end_offset = f.tell()
        return magic, width, height, maxval, header_end_offset


def read_pgm_pixel(img_path: str, u: int, v: int) -> Optional[int]:
    """Read a single pixel value from PGM at (u, v) with v from top.
    Returns None if ASCII P2 (not implemented pixel random access)."""
    magic, width, height, maxval, header_end = read_pgm_header(img_path)
    if not (0 <= u < width and 0 <= v < height):
        raise IndexError(f"Pixel ({u}, {v}) out of bounds [0..{width-1}]x[0..{height-1}]")
    if magic == "P2":
        # ASCII PGM random access is non-trivial without scanning; skip.
        return None
    # P5: binary
    bytes_per_sample = 1 if maxval < 256 else 2
    offset = header_end + (v * width + u) * bytes_per_sample
    with open(img_path, "rb") as f:
        f.seek(offset)
        if bytes_per_sample == 1:
            b = f.read(1)
            if not b:
                raise IOError("Unexpected EOF in PGM data")
            return b[0]
        else:
            b = f.read(2)
            if len(b) != 2:
                raise IOError("Unexpected EOF in PGM data")
            return (b[0] << 8) | b[1]  # big-endian per PGM spec


def pixel_to_map(
    u: int,
    v: int,
    height: int,
    resolution: float,
    origin: Tuple[float, float, float],
    use_center: bool = True,
) -> Tuple[float, float]:
    """Convert image pixel (u,v) to map (x,y) using ROS map YAML semantics.

    - (u,v): pixel coordinate, u from left, v from top, 0-based.
    - height: image height in pixels (needed for vertical flip).
    - resolution: meters per pixel.
    - origin: (ox, oy, yaw) defines the world pose of the lower-left pixel (grid origin).
    - use_center: if True, maps the center of the pixel; else upper-left corner.
    """
    ox, oy, yaw = origin
    # Position in grid frame before rotation (origin at lower-left)
    # Horizontal: +x to the right; Vertical: +y upward (flip from image coordinates)
    half = 0.5 if use_center else 0.0
    x_local = (u + half) * resolution
    y_local = (height - v - half) * resolution

    if abs(yaw) < 1e-12:
        return ox + x_local, oy + y_local

    cy = math.cos(yaw)
    sy = math.sin(yaw)
    xr = cy * x_local - sy * y_local
    yr = sy * x_local + cy * y_local
    return ox + xr, oy + yr


def map_to_pixel(
    x: float,
    y: float,
    height: int,
    resolution: float,
    origin: Tuple[float, float, float],
    use_center: bool = True,
) -> Tuple[float, float]:
    """Convert map (x,y) to image pixel (u,v) floats using ROS map YAML semantics.

    Returns subpixel floats; caller may round/floor/ceil as needed.
    """
    ox, oy, yaw = origin
    dx = x - ox
    dy = y - oy
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    # rotate back by -yaw: [x_l;y_l] = R(-yaw) * [dx;dy]
    x_local =  cy * dx + sy * dy
    y_local = -sy * dx + cy * dy

    if use_center:
        u = (x_local / resolution) - 0.5
        v = height - (y_local / resolution) - 0.5
    else:
        u = (x_local / resolution)
        v = height - (y_local / resolution)
    return u, v


def occupancy_from_pixel_value(
    pixel_value: int,
    maxval: int,
    negate: int,
) -> float:
    """Compute occupancy probability [0,1] from pixel value using map_server formula."""
    # Normalize to 0..1
    occ = float(pixel_value) / float(maxval)
    if negate:
        occ = 1.0 - occ
    return occ


def main():
    parser = argparse.ArgumentParser(description="Convert PGM pixel to map coordinates using ROS map YAML.")
    parser.add_argument("--yaml", required=True, help="Path to map YAML file")
    parser.add_argument("--pixel", nargs=2, type=int, metavar=("U", "V"),
                        help="Pixel coordinate (u v), 0-based, origin at top-left")
    # Optional inverse mapping
    parser.add_argument("--map-to-pixel", nargs=2, type=float, metavar=("X", "Y"),
                        help="Convert map (x y) to pixel (u v)")
    parser.add_argument("--map-origin", action="store_true", help="Shortcut for --map-to-pixel 0 0")
    parser.add_argument("--one-based", action="store_true", help="Interpret input pixel as 1-based indices")
    parser.add_argument("--corner", action="store_true", help="Use pixel's upper-left corner instead of center")
    parser.add_argument("--check-occupancy", action="store_true", help="Read PGM to compute occupancy classification")
    parser.add_argument("--rounding", choices=["round", "floor", "ceil", "none"], default="round",
                        help="Rounding mode for inverse mapping output (default: round)")
    args = parser.parse_args()

    yaml_path = resolve_yaml_path(args.yaml)
    yaml_info = parse_map_yaml(yaml_path)
    image_path = resolve_image_path(yaml_path, yaml_info["image"]) 

    magic, width, height, maxval, header_end = read_pgm_header(image_path)

    # Pixel -> Map conversion path
    if args.pixel:
        u, v = args.pixel
        if args.one_based:
            u -= 1
            v -= 1
        if not (0 <= u < width and 0 <= v < height):
            raise SystemExit(f"Pixel ({u}, {v}) out of bounds [0..{width-1}]x[0..{height-1}]")

        x, y = pixel_to_map(
            u=u,
            v=v,
            height=height,
            resolution=float(yaml_info["resolution"]),
            origin=tuple(yaml_info["origin"]),
            use_center=not args.corner,
        )

        print(f"yaml: {yaml_path}")
        print(f"image: {image_path}")
        print(f"size: {width}x{height}, resolution: {yaml_info['resolution']} m/px, origin: {yaml_info['origin']}")
        print(f"pixel (u,v): ({u},{v}) -> map (x,y): ({x:.6f}, {y:.6f})")

        if args.check_occupancy:
            pv = read_pgm_pixel(image_path, u, v)
            if pv is None:
                print("occupancy: N/A (ASCII PGM random access unsupported)")
            else:
                negate = int(yaml_info.get("negate", 0))
                occ_prob = occupancy_from_pixel_value(pv, maxval, negate)
                occ_th = float(yaml_info.get("occupied_thresh", 0.65))
                free_th = float(yaml_info.get("free_thresh", 0.196))  # default from map_server
                if occ_prob >= occ_th:
                    label = "occupied"
                elif occ_prob <= free_th:
                    label = "free"
                else:
                    label = "unknown"
                print(f"pixel_value: {pv}/{maxval}, negate: {negate}, occ_prob: {occ_prob:.3f} -> {label}")

    # Map -> Pixel conversion path
    if args.map_origin or args.map_to_pixel:
        if args.map_origin:
            x_m, y_m = 0.0, 0.0
        else:
            x_m = float(args.map_to_pixel[0])
            y_m = float(args.map_to_pixel[1])

        uf, vf = map_to_pixel(
            x=x_m,
            y=y_m,
            height=height,
            resolution=float(yaml_info["resolution"]),
            origin=tuple(yaml_info["origin"]),
            use_center=not args.corner,
        )

        # rounding
        if args.rounding == "round":
            u_i, v_i = int(round(uf)), int(round(vf))
        elif args.rounding == "floor":
            u_i, v_i = int(math.floor(uf)), int(math.floor(vf))
        elif args.rounding == "ceil":
            u_i, v_i = int(math.ceil(uf)), int(math.ceil(vf))
        else:
            u_i, v_i = None, None

        print(f"yaml: {yaml_path}")
        print(f"image: {image_path}")
        print(f"size: {width}x{height}, resolution: {yaml_info['resolution']} m/px, origin: {yaml_info['origin']}")
        print(f"map (x,y): ({x_m:.6f}, {y_m:.6f}) -> pixel (u,v): ({uf:.3f}, {vf:.3f})")
        if u_i is not None:
            if 0 <= u_i < width and 0 <= v_i < height:
                print(f"pixel (rounded): ({u_i}, {v_i}) within image bounds {width}x{height}")
                if args.check_occupancy:
                    pv = read_pgm_pixel(image_path, u_i, v_i)
                    if pv is None:
                        print("occupancy: N/A (ASCII PGM random access unsupported)")
                    else:
                        negate = int(yaml_info.get("negate", 0))
                        occ_prob = occupancy_from_pixel_value(pv, maxval, negate)
                        occ_th = float(yaml_info.get("occupied_thresh", 0.65))
                        free_th = float(yaml_info.get("free_thresh", 0.196))
                        if occ_prob >= occ_th:
                            label = "occupied"
                        elif occ_prob <= free_th:
                            label = "free"
                        else:
                            label = "unknown"
                        print(f"pixel_value: {pv}/{maxval}, negate: {negate}, occ_prob: {occ_prob:.3f} -> {label}")
            else:
                print(f"pixel (rounded): ({u_i}, {v_i}) is OUT of bounds {width}x{height}")


if __name__ == "__main__":
    main()
