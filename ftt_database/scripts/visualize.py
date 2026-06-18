#!/usr/bin/env python3
"""
visualize.py
Animates GPS trajectories from multiple shifts simultaneously on a
satellite tile map.

Usage:
    python trajectory_player.py 46 47 48 49 50 51 52
    python trajectory_player.py 46 47 --zoom 17 --speed 2.0 --fps 15
"""

import argparse
import math
import os

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import psycopg2
import requests
from PIL import Image

# User-configurable defaults
DB_HOST      = "localhost"
DB_PORT      = "5432"
DB_NAME      = "ftt"
DB_USER      = "postgres"
DB_PASSWORD  = "postgres"

TILE_SERVER  = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
TILE_ZOOM    = 17
TILE_SIZE    = 256          # pixels per tile (standard OSM size)
TILE_CACHE   = "/tmp/ftt_map_tiles"

FPS          = 10           # animation frames per second
SPEED        = 1.0          # data-seconds consumed per wall-clock second
TILE_PADDING = 1            # extra tiles around the bounding box

COLORS = [
    "#1f77b4",   # blue
    "#d62728",   # red
    "#2ca02c",   # green
    "#9467bd",   # purple
    "#ff7f0e",   # orange
    "#bcbd22",   # yellow-green
    "#17becf",   # cyan
    "#e377c2",   # pink
]


# Database helpers

def get_connection(host: str, port: str) -> psycopg2.extensions.connection:
    return psycopg2.connect(
        host=host, port=port,
        dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD,
    )


def fetch_shift_data(conn, shift_ids: list[int]) -> dict:
    """
    Returns
    -------
    {
      shift_id: {
        "performer": str,
        "poses": [(lat, lon, orig_secs), ...]
      }
    }
    """
    cur = conn.cursor()
    result = {}

    for sid in shift_ids:
        # Performer label (fall back to shift id when no performer assigned)
        cur.execute(
            """
            SELECT COALESCE(p.institution, 'Shift ' || s.id::text)
            FROM   shift s
            LEFT JOIN performer p ON s.performer_id = p.id
            WHERE  s.id = %s
            """,
            (sid,),
        )
        row = cur.fetchone()
        performer = row[0] if row else f"Shift {sid}"

        # All GPS poses for this shift, traversing:
        #  shift → leg → segment (master only, parent_id IS NULL) → pose
        cur.execute(
            """
            SELECT ST_Y(po.position)  AS lat,
                   ST_X(po.position)  AS lon,
                   po.orig_secs
            FROM   pose    po
            JOIN   segment se ON po.segment_id = se.id
            JOIN   leg     l  ON se.leg_id     = l.id
            WHERE  l.shift_id   = %s
              AND  se.parent_id IS NULL
              AND  po.orig_secs IS NOT NULL
            ORDER  BY po.orig_secs
            """,
            (sid,),
        )
        poses = cur.fetchall()   # list of (lat, lon, orig_secs)

        result[sid] = {"performer": performer, "poses": poses}

    cur.close()
    return result


# Tile helpers 

def _latlon_to_tile_float(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    """Fractional tile coordinates for a lat/lon."""
    n = 2 ** zoom
    x = (lon + 180.0) / 360.0 * n
    lat_r = math.radians(lat)
    y = (1.0 - math.asinh(math.tan(lat_r)) / math.pi) / 2.0 * n
    return x, y


def _download_tile(server: str, zoom: int, tx: int, ty: int, cache_dir: str) -> Image.Image:
    os.makedirs(cache_dir, exist_ok=True)
    path = os.path.join(cache_dir, f"{zoom}_{tx}_{ty}.png")
    if not os.path.isfile(path):
        url = server.format(z=zoom, x=tx, y=ty)
        try:
            resp = requests.get(
                url,
                headers={"User-Agent": "FTT-TrajectoryPlayer/1.0"},
                timeout=15,
            )
            resp.raise_for_status()
            with open(path, "wb") as f:
                f.write(resp.content)
            print(f"  Downloaded {url}")
        except Exception as exc:
            print(f"  [WARN] Tile {zoom}/{tx}/{ty} unavailable: {exc}")
            return Image.new("RGB", (TILE_SIZE, TILE_SIZE), (180, 180, 180))
    return Image.open(path).convert("RGB")


def build_map_image(
    all_lats: list[float],
    all_lons: list[float],
    zoom: int,
    server: str,
    cache_dir: str,
    padding: int = TILE_PADDING,
) -> tuple[Image.Image, int, int]:
    """
    Download and merge tiles that cover all coordinates.

    Returns
    -------
    (merged_image, tx_min, ty_min)
    """
    txs = [int(_latlon_to_tile_float(la, lo, zoom)[0]) for la, lo in zip(all_lats, all_lons)]
    tys = [int(_latlon_to_tile_float(la, lo, zoom)[1]) for la, lo in zip(all_lats, all_lons)]

    tx_min, tx_max = min(txs) - padding, max(txs) + padding
    ty_min, ty_max = min(tys) - padding, max(tys) + padding

    cols = tx_max - tx_min + 1
    rows = ty_max - ty_min + 1

    print(f"  Merging {cols * rows} tiles ({cols}×{rows}) …")
    canvas = Image.new("RGB", (cols * TILE_SIZE, rows * TILE_SIZE))

    for tx in range(tx_min, tx_max + 1):
        for ty in range(ty_min, ty_max + 1):
            tile = _download_tile(server, zoom, tx, ty, cache_dir)
            tile = tile.resize((TILE_SIZE, TILE_SIZE))
            canvas.paste(tile, ((tx - tx_min) * TILE_SIZE, (ty - ty_min) * TILE_SIZE))

    return canvas, tx_min, ty_min


def latlon_to_pixel(
    lat: float, lon: float, zoom: int, tx_min: int, ty_min: int
) -> tuple[float, float]:
    """Convert lat/lon to pixel coordinates within the merged image."""
    xf, yf = _latlon_to_tile_float(lat, lon, zoom)
    px = (xf - tx_min) * TILE_SIZE
    py = (yf - ty_min) * TILE_SIZE
    return px, py


# Animation

def animate(
    shift_data: dict,
    tile_server: str = TILE_SERVER,
    zoom: int = TILE_ZOOM,
    cache_dir: str = TILE_CACHE,
    fps: int = FPS,
    speed: float = SPEED,
    pause_secs=3.0,
) -> None:
    """
    Animate all trajectories simultaneously on the satellite map.
    Each shift:
      - gets a unique colour
      - shows a moving dot + label "<performer>\\n+HH:MM:SS"
      - leaves a persistent coloured trail
    Time is shown relative to each shift's own first pose.
    """
    # 1. Pre-process poses
    shift_ids = list(shift_data.keys())

    all_lats, all_lons = [], []
    for info in shift_data.values():
        for lat, lon, _ in info["poses"]:
            all_lats.append(lat)
            all_lons.append(lon)

    if not all_lats:
        print("No pose data found for the requested shifts.")
        return

    # 2. Build satellite background
    print("Building satellite background …")
    map_img, tx_min, ty_min = build_map_image(
        all_lats, all_lons, zoom, tile_server, cache_dir
    )
    map_arr = np.array(map_img)

    # Convert poses to pixel coords and make timestamps relative
    for sid in shift_ids:
        raw = shift_data[sid]["poses"]
        if not raw:
            shift_data[sid]["px"] = []
            continue
        t0 = raw[0][2]                 # first orig_secs  →  t = 0
        shift_data[sid]["px"] = [
            (latlon_to_pixel(lat, lon, zoom, tx_min, ty_min), t - t0)
            for lat, lon, t in raw
        ]

    # Global animation duration = longest individual trajectory
    global_t_max = max(
        (px[-1][1] for sid in shift_ids for px in [shift_data[sid]["px"]] if px),
        default=1.0,
    )

    dt_per_frame = speed / fps                    # data-seconds per frame
    pause_frames = int(pause_secs * fps)
    n_frames     = pause_frames + max(1, math.ceil(global_t_max / dt_per_frame))

    # 3. Matplotlib figure
    fig, ax = plt.subplots(figsize=(14, 10), dpi=96)
    ax.imshow(map_arr, aspect="equal", interpolation="bilinear")
    ax.set_xlim(0, map_img.width)
    ax.set_ylim(map_img.height, 0)   # y-axis: 0 at top (image convention)
    ax.axis("off")
    ax.set_title("Shift Trajectory Playback", fontsize=14, pad=10)

    # Clock in top-left corner
    clock_txt = ax.text(
        0.01, 0.98, "",
        transform=ax.transAxes,
        fontsize=11, color="white", va="top",
        bbox=dict(boxstyle="round,pad=0.3", fc="black", alpha=0.65),
        zorder=10,
    )

    countdown_txt = ax.text(
        0.5, 0.5, "",
        transform=ax.transAxes,
        fontsize=36, color="white", va="center", ha="center", fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.5", fc="black", alpha=0.70),
        zorder=20,
    )

    # Per-shift artists
    arts = {}
    for i, sid in enumerate(shift_ids):
        c = COLORS[i % len(COLORS)]
        trail,  = ax.plot([], [], "-",  color=c, linewidth=2.0, alpha=0.85, zorder=3)
        dot,    = ax.plot([], [], "o",  color=c, markersize=11,
                          markeredgecolor="white", markeredgewidth=1.2, zorder=5)
        lbl = ax.text(
            0, 0, "",
            color="white", fontsize=8, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.25", fc=c, alpha=0.85, edgecolor="white", linewidth=0.6),
            zorder=6, visible=False,
        )
        arts[sid] = dict(trail=trail, dot=dot, lbl=lbl)

    # Legend
    legend_handles = []
    for i, sid in enumerate(shift_ids):
        c = COLORS[i % len(COLORS)]
        handle = plt.Line2D(
            [0], [0], marker="o", color="w", markerfacecolor=c,
            markersize=10, label=f"{shift_data[sid]['performer']}  (shift {sid})",
        )
        legend_handles.append(handle)
    ax.legend(
        handles=legend_handles,
        loc="lower right",
        fontsize=9,
        framealpha=0.75,
        edgecolor="white",
    )

    # Animation callbacks 

    def init():
        for sid in shift_ids:
            arts[sid]["trail"].set_data([], [])
            arts[sid]["dot"].set_data([], [])
            arts[sid]["lbl"].set_visible(False)
        clock_txt.set_text("")
        countdown_txt.set_text("")
        flat = [v for d in arts.values() for v in d.values()]
        return flat + [clock_txt, countdown_txt]

    def update(frame: int):
        # Pause phase 
        if frame < pause_frames:
            remaining = math.ceil(pause_secs - frame / fps)
            countdown_txt.set_text(f"Starting in {remaining}…")
            countdown_txt.set_visible(True)
            flat = [v for d in arts.values() for v in d.values()]
            return flat + [clock_txt, countdown_txt]

        # Animation phase 
        countdown_txt.set_visible(False)
        cur_t = (frame - pause_frames) * dt_per_frame

        for sid in shift_ids:
            px_poses = shift_data[sid]["px"]
            if not px_poses:
                continue

            # All points whose relative timestamp ≤ current time
            visible = [((px, py), rt) for (px, py), rt in px_poses if rt <= cur_t]
            if not visible:
                arts[sid]["trail"].set_data([], [])
                arts[sid]["dot"].set_data([], [])
                arts[sid]["lbl"].set_visible(False)
                continue

            xs = [p[0][0] for p in visible]
            ys = [p[0][1] for p in visible]

            arts[sid]["trail"].set_data(xs, ys)
            arts[sid]["dot"].set_data([xs[-1]], [ys[-1]])

            # Label: performer + elapsed HH:MM:SS (relative to this shift's t0)
            elapsed   = int(visible[-1][1])
            h, rem    = divmod(elapsed, 3600)
            m, s      = divmod(rem, 60)
            performer = shift_data[sid]["performer"]
            arts[sid]["lbl"].set_text(f"{performer}\n+{h:02d}:{m:02d}:{s:02d}")
            arts[sid]["lbl"].set_position((xs[-1] + 10, ys[-1] - 10))
            arts[sid]["lbl"].set_visible(True)

        # Global clock
        h, rem = divmod(int(cur_t), 3600)
        m, s   = divmod(rem, 60)
        clock_txt.set_text(f"elapsed  {h:02d}:{m:02d}:{s:02d}")

        flat = [v for d in arts.values() for v in d.values()]
        return flat + [clock_txt]

    ani = animation.FuncAnimation(
        fig, update,
        frames=n_frames,
        init_func=init,
        interval=1000 / fps,
        blit=True,
        repeat=False,
    )

    plt.tight_layout()
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()
    plt.show()


# Entry point 

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Animate shift trajectories on a satellite tile map."
    )
    parser.add_argument(
        "shift_ids", nargs="+", type=int,
        help="Shift IDs to animate  (e.g.  46 47 48 49 50 51 52)",
    )
    parser.add_argument("--db-host",     default=DB_HOST,    help="PostgreSQL host")
    parser.add_argument("--db-port",     default=DB_PORT,    help="PostgreSQL port")
    parser.add_argument("--tile-server", default=TILE_SERVER,
                        help="Tile URL template  {z}/{x}/{y}")
    parser.add_argument("--zoom",        default=TILE_ZOOM,  type=int,
                        help="Tile zoom level")
    parser.add_argument("--tile-dir",    default=TILE_CACHE,
                        help="Directory to cache downloaded tiles")
    parser.add_argument("--fps",         default=FPS,        type=int,
                        help="Frames per second")
    parser.add_argument("--speed",       default=SPEED,      type=float,
                        help="Playback speed multiplier  (1 = real-time, 2 = 2 times faster …)")
    parser.add_argument("--pause", default=3, type=float,
                    help="Seconds to wait before the animation starts (default: 3)")
    args = parser.parse_args()

    print(f"Connecting to {args.db_host}:{args.db_port} …")
    conn = get_connection(args.db_host, args.db_port)

    print(f"Fetching data for shifts: {args.shift_ids}")
    shift_data = fetch_shift_data(conn, args.shift_ids)
    conn.close()

    for sid, info in shift_data.items():
        print(f"  Shift {sid:>4d}  |  performer: {info['performer']:<20s}  |  {len(info['poses'])} poses")

    animate(
        shift_data,
        tile_server=args.tile_server,
        zoom=args.zoom,
        cache_dir=args.tile_dir,
        fps=args.fps,
        speed=args.speed,
        pause_secs=args.pause,
    )