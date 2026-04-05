import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Arrow, Rectangle

# ============================================================
# CONFIG - EDIT HERE
# ============================================================

# ADVANCED CONTROLS
GRID_WIDTH = 4
GRID_HEIGHT = 5
CELL_SIZE = 50.0
DENSE_SPACING = 3.0
BLEND_RADIUS = 25.0
DESIRED_TIME = 73
ROBOT_RADIUS = 8.0
ROBOT_SAMPLE_SPACING = 25.0
MIN_ROBOT_SAMPLES = 2
BOTTLE_RADIUS = 3.0

FORWARD = "FORWARD" #NOTE: ROBOT CAN NOT IMMEDIATELY TURN AT THE BEGINNING OF A SUBGRAPH MAKE SURE YOU MOVE AT LEAST ONE TILE BEFORE TURNING
REVERSE = "REVERSE"
DO_180 = "DO_180" #NOTE: 180's CAN CAUSE DRIFTING
START = "START"
STOP = "STOP"

FORWARD_COLOR = "tab:blue"
REVERSE_COLOR = "tab:orange"
TURN_COLOR = "tab:green"
CARRIED_BOTTLE_COLOR = "purple"
MISSION_MARKER_COLOR = "yellow"

# MAIN THING TO EDIT
MISSION_PROTOCOL = [
    (0, 2.5),
    START,
    (0, 2.5),
    (2.5, 2.5),
    (2.5, 4.5),
    (0.5, 4.5),
    REVERSE,
    (2.5, 4.5),
    (2.5, 2.5),
    FORWARD,
    (2.5, 3.5),
    (0.5, 3.5),
    DO_180,
    (3.5, 3.5),
    (3.5, 0.5),
    REVERSE,
    (3.5, 1.5),
    (2.5, 1.5),
    (2.5, 2.5),
    FORWARD,
    (2.5, 1.5),
    (0.5, 1.5),
    (0.5, 0.5),
    (1.5, 0.5),
    REVERSE,
    (0.5, 0.5),
    (0.5, 1.5),
    (3.5, 1.5),
    (3.5, 3.5),
    STOP
]

BOTTLE_SPOTS = [
    (1, 1.5),
    (1, 3.5),
    (2.5, 4)
]

ZONE_SPOTS = [
    (1, 5, "A"),
    (2, 1, "B"),
    (4, 3, "C"),
    (4, 1, "D")
]

WALL_SPOTS = [
    (0, 5, 1, 5),
    (1, 4, 2, 4),
    (3, 4, 3, 5),
    (3, 4, 4, 4),
    (0, 3, 1, 3),
    (3, 2, 3, 3),
    (1, 1, 2, 1),
    (2, 0, 2, 1),
]

# ============================================================
# GEOMETRY HELPERS - DO NOT EDIT
# ============================================================

def to_cm(points_grid, cell_size):
    return [(x * cell_size, y * cell_size) for x, y in points_grid]

def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1])

def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1])

def vec_mul(v, s):
    return (v[0] * s, v[1] * s)

def vec_len(v):
    return math.hypot(v[0], v[1])

def vec_norm(v):
    l = vec_len(v)
    if l < 1e-9:
        return (0.0, 0.0)
    return (v[0] / l, v[1] / l)

def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

def angle_of(v):
    return math.atan2(v[1], v[0])

def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def dedupe_points(points, eps=1e-6):
    if not points:
        return []
    out = [points[0]]
    for p in points[1:]:
        if distance(p, out[-1]) > eps:
            out.append(p)
    return out

def path_length(points):
    total = 0.0
    for i in range(len(points) - 1):
        total += distance(points[i], points[i + 1])
    return total

# ============================================================
# PATH GENERATION - DO NOT EDIT
# ============================================================

def interpolate_line(a, b, spacing):
    d = distance(a, b)
    if d < 1e-9:
        return [a]
    steps = max(1, int(math.ceil(d / spacing)))
    return [lerp(a, b, i / steps) for i in range(steps + 1)]

def interpolate_arc(center, radius, start_angle, end_angle, spacing, ccw=True):
    arc_angle = wrap_angle(end_angle - start_angle)

    if ccw and arc_angle < 0:
        arc_angle += 2 * math.pi
    elif (not ccw) and arc_angle > 0:
        arc_angle -= 2 * math.pi

    arc_len = abs(arc_angle) * radius
    steps = max(1, int(math.ceil(arc_len / spacing)))

    pts = []
    for i in range(steps + 1):
        t = i / steps
        a = start_angle + arc_angle * t
        pts.append((
            center[0] + radius * math.cos(a),
            center[1] + radius * math.sin(a)
        ))
    return pts

def signed_turn_angle(in_dir, out_dir):
    return wrap_angle(angle_of(out_dir) - angle_of(in_dir))

def build_fillet(a, b, c, requested_radius, spacing):
    v_in = vec_sub(b, a)
    v_out = vec_sub(c, b)

    len_in = vec_len(v_in)
    len_out = vec_len(v_out)

    if len_in < 1e-9 or len_out < 1e-9:
        return None

    in_dir = vec_norm(v_in)
    out_dir = vec_norm(v_out)

    turn = signed_turn_angle(in_dir, out_dir)
    theta = abs(turn)

    if theta < math.radians(3):
        return None
    if abs(math.pi - theta) < math.radians(3):
        return None

    tan_half = math.tan(theta / 2.0)
    if abs(tan_half) < 1e-9:
        return None

    max_radius_from_in = len_in / tan_half
    max_radius_from_out = len_out / tan_half
    r = min(requested_radius, max_radius_from_in * 0.99, max_radius_from_out * 0.99)

    if r < 1e-6:
        return None

    tangent_dist = r * tan_half

    entry = vec_add(b, vec_mul(in_dir, -tangent_dist))
    exit_ = vec_add(b, vec_mul(out_dir, tangent_dist))

    ccw = turn > 0
    left_normal_in = (-in_dir[1], in_dir[0])
    right_normal_in = (in_dir[1], -in_dir[0])
    n = left_normal_in if ccw else right_normal_in

    center = vec_add(entry, vec_mul(n, r))

    if abs(distance(center, exit_) - r) > 1e-4:
        return None

    start_angle = math.atan2(entry[1] - center[1], entry[0] - center[0])
    end_angle = math.atan2(exit_[1] - center[1], exit_[0] - center[0])

    arc_pts = interpolate_arc(center, r, start_angle, end_angle, spacing, ccw=ccw)
    return entry, arc_pts, exit_

def generate_dense_path(points, spacing, blend_radius):
    if len(points) < 2:
        return points[:]

    if len(points) == 2 or blend_radius <= 0.0:
        dense = []
        for i in range(len(points) - 1):
            seg = interpolate_line(points[i], points[i + 1], spacing)
            if dense and distance(dense[-1], seg[0]) < 1e-6:
                seg = seg[1:]
            dense.extend(seg)
        return dedupe_points(dense)

    dense = [points[0]]
    current_start = points[0]
    i = 0

    while i < len(points) - 1:
        if i >= len(points) - 2:
            seg = interpolate_line(current_start, points[i + 1], spacing)
            if seg and distance(dense[-1], seg[0]) < 1e-6:
                seg = seg[1:]
            dense.extend(seg)
            break

        a = points[i]
        b = points[i + 1]
        c = points[i + 2]
        fillet = build_fillet(a, b, c, blend_radius, spacing)

        if fillet is None:
            seg = interpolate_line(current_start, b, spacing)
            if seg and distance(dense[-1], seg[0]) < 1e-6:
                seg = seg[1:]
            dense.extend(seg)
            current_start = b
            i += 1
            continue

        entry, arc_pts, exit_ = fillet

        seg = interpolate_line(current_start, entry, spacing)
        if seg and distance(dense[-1], seg[0]) < 1e-6:
            seg = seg[1:]
        dense.extend(seg)

        if arc_pts and distance(dense[-1], arc_pts[0]) < 1e-6:
            arc_pts = arc_pts[1:]
        dense.extend(arc_pts)

        current_start = exit_
        i += 1

    return dedupe_points(dense)

# ============================================================
# PARSING - DO NOT EDIT
# ============================================================

def parse_mixed_segments(items, initial_mode=FORWARD):
    segments = []
    current_mode = initial_mode
    current_points = []

    for item in items:
        if item == FORWARD or item == REVERSE:
            if len(current_points) >= 2:
                segments.append({
                    "kind": "path",
                    "mode": current_mode,
                    "points": current_points[:]
                })
            current_mode = item
            if current_points:
                current_points = [current_points[-1]]
            else:
                current_points = []

        elif item == DO_180:
            if len(current_points) >= 2:
                segments.append({
                    "kind": "path",
                    "mode": current_mode,
                    "points": current_points[:]
                })

            if len(current_points) >= 1:
                segments.append({
                    "kind": "turn",
                    "mode": current_mode,
                    "point": current_points[-1]
                })

            if current_points:
                current_points = [current_points[-1]]

        elif item == START:
            if len(current_points) >= 2:
                segments.append({
                    "kind": "path",
                    "mode": current_mode,
                    "points": current_points[:]
                })

            if len(current_points) >= 1:
                segments.append({
                    "kind": "start",
                    "mode": current_mode,
                    "point": current_points[-1],
                    "label": "Mission begin"
                })

            current_points = []

        elif item == STOP:
            if len(current_points) >= 2:
                segments.append({
                    "kind": "path",
                    "mode": current_mode,
                    "points": current_points[:]
                })

            if len(current_points) >= 1:
                segments.append({
                    "kind": "stop",
                    "mode": current_mode,
                    "point": current_points[-1],
                    "label": "Mission complete"
                })

            current_points = []

        else:
            current_points.append(item)

    if len(current_points) >= 2:
        segments.append({
            "kind": "path",
            "mode": current_mode,
            "points": current_points[:]
        })

    return segments

# ============================================================
# ROBOT DRAWING - DO NOT EDIT
# ============================================================

def sample_robot_positions(path, spacing_cm, min_samples=2):
    if len(path) == 0:
        return []
    if len(path) == 1:
        return [0]

    cumulative = [0.0]
    for i in range(1, len(path)):
        cumulative.append(cumulative[-1] + distance(path[i - 1], path[i]))

    total_len = cumulative[-1]
    if total_len < 1e-9:
        return [0]

    count = max(min_samples, int(total_len // spacing_cm) + 1)
    count = min(count, len(path))

    target_distances = [
        total_len * i / (count - 1) if count > 1 else 0.0
        for i in range(count)
    ]

    idxs = []
    for td in target_distances:
        best_idx = min(range(len(cumulative)), key=lambda i: abs(cumulative[i] - td))
        idxs.append(best_idx)

    out = []
    seen = set()
    for idx in idxs:
        if idx not in seen:
            out.append(idx)
            seen.add(idx)

    if 0 not in seen:
        out.insert(0, 0)
    if (len(path) - 1) not in seen:
        out.append(len(path) - 1)

    return out

def heading_at(path, idx, reverse=False):
    if len(path) < 2:
        return 0.0

    if idx <= 0:
        heading = angle_of(vec_sub(path[1], path[0]))
    elif idx >= len(path) - 1:
        heading = angle_of(vec_sub(path[-1], path[-2]))
    else:
        heading = angle_of(vec_sub(path[idx + 1], path[idx - 1]))

    if reverse:
        heading = wrap_angle(heading + math.pi)

    return heading

def draw_robot(ax, x, y, heading, radius, color, label=None):
    body = Circle((x, y), radius, fill=False, linewidth=2.0, edgecolor=color, zorder=6)
    ax.add_patch(body)

    arrow_len = radius * 1.5
    arrow = Arrow(
        x, y,
        arrow_len * math.cos(heading),
        arrow_len * math.sin(heading),
        width=radius * 0.7,
        color=color,
        zorder=7
    )
    ax.add_patch(arrow)

    if label is not None:
        ax.text(x + radius + 2, y + radius + 2, label, fontsize=9, color=color, zorder=8)

# ============================================================
# FIELD DRAWING - DO NOT EDIT
# ============================================================

def draw_walls(ax, cell_size):
    for x1, y1, x2, y2 in WALL_SPOTS:
        ax.plot(
            [x1 * cell_size, x2 * cell_size],
            [y1 * cell_size, y2 * cell_size],
            linewidth=5,
            color="black",
            solid_capstyle="butt",
            zorder=8
        )

def draw_zones(ax, cell_size, show_labels=True):
    for col, row, label in ZONE_SPOTS:
        x0 = (col - 1) * cell_size
        y0 = (row - 1) * cell_size

        zone = Rectangle(
            (x0, y0),
            cell_size,
            cell_size,
            fill=True,
            alpha=0.15,
            linewidth=2.0,
            zorder=1
        )
        ax.add_patch(zone)

        ax.text(
            x0 + 0.5 * cell_size,
            y0 + 0.5 * cell_size,
            label,
            ha="center",
            va="center",
            fontsize=16 if show_labels else 12,
            fontweight="bold",
            zorder=2
        )

def draw_bottle(ax, x, y, color="black", label=None, zorder=9):
    bottle = Circle(
        (x, y),
        BOTTLE_RADIUS,
        facecolor=color,
        edgecolor=color,
        linewidth=1.0,
        zorder=zorder
    )
    ax.add_patch(bottle)

    if label is not None:
        ax.text(x + 4, y + 4, label, fontsize=9, color=color, zorder=zorder + 1)

def draw_bottles_state(ax, bottle_states, show_labels=True):
    for bottle in bottle_states:
        x, y = bottle["pos_cm"]
        label = f"B{bottle['id']}" if show_labels else None
        draw_bottle(ax, x, y, color="black", label=label, zorder=9)

# ============================================================
# BOTTLE SIMULATION - DO NOT EDIT
# ============================================================

def initial_bottle_states(cell_size):
    bottles_cm = to_cm(BOTTLE_SPOTS, cell_size)
    return [{"id": i, "pos_cm": bottles_cm[i]} for i in range(len(bottles_cm))]

def find_first_hit_bottle(path, bottle_states):
    hit_radius = ROBOT_RADIUS + BOTTLE_RADIUS

    for path_idx, robot_pos in enumerate(path):
        for bottle in bottle_states:
            if distance(robot_pos, bottle["pos_cm"]) <= hit_radius:
                return bottle["id"], path_idx
    return None, None

def make_turn_path_and_headings(point_cm, current_heading, count=7):
    headings = []
    for i in range(count):
        t = i / (count - 1) if count > 1 else 0.0
        headings.append(wrap_angle(current_heading + math.pi * t))
    path = [point_cm for _ in range(count)]
    return path, headings

def simulate_segments(segments, cell_size, spacing, blend_radius):
    bottle_states = initial_bottle_states(cell_size)
    attached_bottle_id = None
    current_heading = 0.0

    sim_results = []

    for seg_idx, segment in enumerate(segments):
        attach_event = None
        drop_event = None
        attached_path_positions = {}

        if segment["kind"] == "path":
            stop_points = to_cm(segment["points"], cell_size)
            dense_path = generate_dense_path(stop_points, spacing, blend_radius)
            mode = segment["mode"]

            if mode == REVERSE and attached_bottle_id is not None:
                drop_pos = dense_path[0]
                for bottle in bottle_states:
                    if bottle["id"] == attached_bottle_id:
                        bottle["pos_cm"] = drop_pos
                        break
                drop_event = {
                    "bottle_id": attached_bottle_id,
                    "path_idx": 0,
                    "pos_cm": drop_pos
                }
                attached_bottle_id = None

            bottle_states_before = [
                {"id": b["id"], "pos_cm": b["pos_cm"]}
                for b in bottle_states
            ]

            if mode == FORWARD:
                if attached_bottle_id is None:
                    hit_bottle_id, hit_idx = find_first_hit_bottle(dense_path, bottle_states)
                    if hit_bottle_id is not None:
                        attached_bottle_id = hit_bottle_id
                        attach_event = {
                            "bottle_id": hit_bottle_id,
                            "path_idx": hit_idx,
                            "pos_cm": dense_path[hit_idx]
                        }
                        for i in range(hit_idx, len(dense_path)):
                            attached_path_positions[i] = dense_path[i]
                else:
                    for i in range(len(dense_path)):
                        attached_path_positions[i] = dense_path[i]

            bottle_states_after = [
                {"id": b["id"], "pos_cm": b["pos_cm"]}
                for b in bottle_states
            ]

            if len(dense_path) >= 2:
                if mode == REVERSE:
                    current_heading = heading_at(dense_path, len(dense_path) - 1, reverse=True)
                else:
                    current_heading = heading_at(dense_path, len(dense_path) - 1, reverse=False)

            sim_results.append({
                "segment_index": seg_idx,
                "kind": "path",
                "mode": mode,
                "stop_points_cm": stop_points,
                "dense_path": dense_path,
                "turn_headings": None,
                "bottle_states_before": bottle_states_before,
                "bottle_states_after": bottle_states_after,
                "attached_path_positions": attached_path_positions,
                "attach_event": attach_event,
                "drop_event": drop_event,
                "attached_bottle_id": attached_bottle_id if mode == FORWARD and attached_path_positions else None
            })

        elif segment["kind"] == "turn":
            point_cm = to_cm([segment["point"]], cell_size)[0]
            dense_path, turn_headings = make_turn_path_and_headings(point_cm, current_heading, count=7)

            bottle_states_before = [
                {"id": b["id"], "pos_cm": b["pos_cm"]}
                for b in bottle_states
            ]

            if attached_bottle_id is not None:
                for i in range(len(dense_path)):
                    attached_path_positions[i] = point_cm

            bottle_states_after = [
                {"id": b["id"], "pos_cm": b["pos_cm"]}
                for b in bottle_states
            ]

            current_heading = wrap_angle(current_heading + math.pi)

            sim_results.append({
                "segment_index": seg_idx,
                "kind": "turn",
                "mode": segment["mode"],
                "stop_points_cm": [point_cm],
                "dense_path": dense_path,
                "turn_headings": turn_headings,
                "bottle_states_before": bottle_states_before,
                "bottle_states_after": bottle_states_after,
                "attached_path_positions": attached_path_positions,
                "attach_event": None,
                "drop_event": None,
                "attached_bottle_id": attached_bottle_id
            })

        elif segment["kind"] == "start":
            point_cm = to_cm([segment["point"]], cell_size)[0]

            bottle_states_before = [
                {"id": b["id"], "pos_cm": b["pos_cm"]}
                for b in bottle_states
            ]

            bottle_states_after = [
                {"id": b["id"], "pos_cm": b["pos_cm"]}
                for b in bottle_states
            ]

            sim_results.append({
                "segment_index": seg_idx,
                "kind": "start",
                "mode": segment["mode"],
                "stop_points_cm": [point_cm],
                "dense_path": [point_cm],
                "turn_headings": None,
                "bottle_states_before": bottle_states_before,
                "bottle_states_after": bottle_states_after,
                "attached_path_positions": {},
                "attach_event": None,
                "drop_event": None,
                "attached_bottle_id": attached_bottle_id,
                "label": segment.get("label", "Mission begin")
            })

        elif segment["kind"] == "stop":
            point_cm = to_cm([segment["point"]], cell_size)[0]

            bottle_states_before = [
                {"id": b["id"], "pos_cm": b["pos_cm"]}
                for b in bottle_states
            ]

            bottle_states_after = [
                {"id": b["id"], "pos_cm": b["pos_cm"]}
                for b in bottle_states
            ]

            sim_results.append({
                "segment_index": seg_idx,
                "kind": "stop",
                "mode": segment["mode"],
                "stop_points_cm": [point_cm],
                "dense_path": [point_cm],
                "turn_headings": None,
                "bottle_states_before": bottle_states_before,
                "bottle_states_after": bottle_states_after,
                "attached_path_positions": {},
                "attach_event": None,
                "drop_event": None,
                "attached_bottle_id": attached_bottle_id,
                "label": segment.get("label", "Mission complete")
            })

    return sim_results

# ============================================================
# SEGMENT DRAWING - DO NOT EDIT
# ============================================================

def setup_field(ax, cell_size, show_labels=True):
    ax.clear()
    field_width = GRID_WIDTH * cell_size
    field_height = GRID_HEIGHT * cell_size

    draw_zones(ax, cell_size, show_labels)

    for i in range(GRID_HEIGHT + 1):
        y = i * cell_size
        ax.plot([0, field_width], [y, y], linewidth=1, color="gray", zorder=3)

    for i in range(GRID_WIDTH + 1):
        x = i * cell_size
        ax.plot([x, x], [0, field_height], linewidth=1, color="gray", zorder=3)

    draw_walls(ax, cell_size)

    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_xlim(-10, field_width + 10)
    ax.set_ylim(-10, field_height + 10)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(False)

def draw_segment(ax, sim_result, cell_size, show_labels=True):
    setup_field(ax, cell_size, show_labels)

    kind = sim_result["kind"]
    mode = sim_result["mode"]

    draw_bottles_state(ax, sim_result["bottle_states_before"], show_labels=show_labels)

    if kind == "path":
        color = FORWARD_COLOR if mode == FORWARD else REVERSE_COLOR
        reverse_mode = (mode == REVERSE)

        stop_points = sim_result["stop_points_cm"]
        dense_path = sim_result["dense_path"]

        sx = [p[0] for p in stop_points]
        sy = [p[1] for p in stop_points]
        dx = [p[0] for p in dense_path]
        dy = [p[1] for p in dense_path]

        ax.plot(sx, sy, linestyle="--", linewidth=1.5, color=color, alpha=0.7, zorder=4)
        ax.plot(dx, dy, linewidth=2.5, color=color, zorder=5)
        ax.scatter(sx, sy, s=80, color=color, zorder=6)
        ax.scatter(dx, dy, s=12, color=color, alpha=0.7, zorder=5)

        ax.scatter([sx[0]], [sy[0]], s=160, color="green", zorder=10)
        ax.scatter([sx[-1]], [sy[-1]], s=180, color="red", marker="x", linewidths=3, zorder=10)

        if show_labels:
            ax.text(sx[0] + 5, sy[0] + 5, "START", color="green", fontsize=10, zorder=11)
            ax.text(sx[-1] + 5, sy[-1] + 5, "END", color="red", fontsize=10, zorder=11)

        sample_idxs = sample_robot_positions(
            dense_path,
            ROBOT_SAMPLE_SPACING,
            min_samples=MIN_ROBOT_SAMPLES
        )

        attached_path_positions = sim_result["attached_path_positions"]
        attached_bottle_id = sim_result["attached_bottle_id"]

        for n, idx in enumerate(sample_idxs):
            x, y = dense_path[idx]
            heading = heading_at(dense_path, idx, reverse=reverse_mode)
            label = f"R{n}" if show_labels else None
            draw_robot(ax, x, y, heading, ROBOT_RADIUS, color=color, label=label)

            if idx in attached_path_positions and attached_bottle_id is not None:
                bx, by = attached_path_positions[idx]
                draw_bottle(
                    ax,
                    bx,
                    by,
                    color=CARRIED_BOTTLE_COLOR,
                    label=(f"B{attached_bottle_id}*" if show_labels else None),
                    zorder=12
                )

        if sim_result["attach_event"] is not None:
            ev = sim_result["attach_event"]
            ax.scatter([ev["pos_cm"][0]], [ev["pos_cm"][1]], s=180, marker="s",
                       color=CARRIED_BOTTLE_COLOR, zorder=13)

        if sim_result["drop_event"] is not None:
            ev = sim_result["drop_event"]
            ax.scatter([ev["pos_cm"][0]], [ev["pos_cm"][1]], s=180, marker="D",
                       color=CARRIED_BOTTLE_COLOR, zorder=13)

        title = f"Segment {sim_result['segment_index'] + 1}: {mode}"
        if sim_result["attach_event"] is not None:
            title += f" | attach B{sim_result['attach_event']['bottle_id']}"
        if sim_result["drop_event"] is not None:
            title += f" | disengage B{sim_result['drop_event']['bottle_id']}"

        ax.set_title(title)

    elif kind == "turn":
        dense_path = sim_result["dense_path"]
        turn_headings = sim_result["turn_headings"]
        point = dense_path[0]

        ax.scatter([point[0]], [point[1]], s=160, color="green", zorder=10)

        for n, (p, heading) in enumerate(zip(dense_path, turn_headings)):
            label = f"T{n}" if show_labels else None
            draw_robot(ax, p[0], p[1], heading, ROBOT_RADIUS, color=TURN_COLOR, label=label)

        attached_bottle_id = sim_result["attached_bottle_id"]
        if attached_bottle_id is not None:
            draw_bottle(
                ax,
                point[0],
                point[1],
                color=CARRIED_BOTTLE_COLOR,
                label=(f"B{attached_bottle_id}*" if show_labels else None),
                zorder=12
            )

        ax.set_title(
            f"Segment {sim_result['segment_index'] + 1}: DO_180 | FORWARD-facing after turn | Risky"
        )

    elif kind == "start":
        point = sim_result["stop_points_cm"][0]
        label = sim_result.get("label", "Mission begin")

        ax.scatter(
            [point[0]],
            [point[1]],
            s=260,
            color=MISSION_MARKER_COLOR,
            marker="*",
            edgecolors="goldenrod",
            linewidths=1.5,
            zorder=12
        )

        if show_labels:
            ax.text(
                point[0] + 5,
                point[1] + 5,
                label,
                color="black",
                fontsize=11,
                fontweight="bold",
                zorder=13
            )

        ax.set_title(f"Segment {sim_result['segment_index'] + 1}: {label}")

    elif kind == "stop":
        point = sim_result["stop_points_cm"][0]
        label = sim_result.get("label", "Mission complete")

        ax.scatter(
            [point[0]],
            [point[1]],
            s=260,
            color=MISSION_MARKER_COLOR,
            marker="*",
            edgecolors="goldenrod",
            linewidths=1.5,
            zorder=12
        )

        if show_labels:
            ax.text(
                point[0] + 5,
                point[1] + 5,
                label,
                color="black",
                fontsize=11,
                fontweight="bold",
                zorder=13
            )

        attached_bottle_id = sim_result["attached_bottle_id"]
        if attached_bottle_id is not None:
            draw_bottle(
                ax,
                point[0],
                point[1],
                color=CARRIED_BOTTLE_COLOR,
                label=(f"B{attached_bottle_id}*" if show_labels else None),
                zorder=12
            )

        ax.set_title(f"Segment {sim_result['segment_index'] + 1}: {label}")

# ============================================================
# MAIN - DO NOT EDIT
# ============================================================

segments = parse_mixed_segments(MISSION_PROTOCOL, initial_mode=FORWARD)

if not segments:
    raise ValueError("No valid segments found.")

sim_results = simulate_segments(
    segments,
    CELL_SIZE,
    DENSE_SPACING,
    BLEND_RADIUS
)

all_dense_paths = [result["dense_path"] for result in sim_results if result["kind"] == "path"]
total_path_length = sum(path_length(path) for path in all_dense_paths)
desired_velocity = total_path_length / DESIRED_TIME if DESIRED_TIME > 1e-9 else 0.0

fig, axes = plt.subplots(len(sim_results), 1, figsize=(9, 7 * len(sim_results)))
if len(sim_results) == 1:
    axes = [axes]

for i, result in enumerate(sim_results):
    draw_segment(
        axes[i],
        result,
        CELL_SIZE,
        show_labels=True
    )

if desired_velocity > 30.0:
    speed_label = "Risky Speed"
else:
    speed_label = "Safe Speed"

fig.text(
    0.5, 0.02,
    f"Total Path Length = {total_path_length:.1f} cm | "
    f"Desired Time = {DESIRED_TIME:.0f} s | "
    f"Desired Velocity = {desired_velocity:.2f} cm/s | "
    f"{speed_label}",
    ha="center",
    fontsize=12
)

plt.tight_layout(rect=[0, 0.05, 1, 1])
plt.show()
