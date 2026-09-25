"""The live window: every pipeline stage side by side, updated every tick.

Panels: raw frame with tags; rectified arena; obstacle mask (red) with component outlines and areas;
inflated C-space (orange) over the dimmed arena; cost-to-go contours; path (cyan), waypoints (magenta),
lookahead, robot pose arrow and goal; and a text corner with state, command, loop rate, robot.z std,
reprojection error, tags detected and free-space percentage.
"""
import numpy as np
import cv2

from .geometry import CORNER_IDS, ROBOT_ID, ARM_ID

PANEL_H = 330
FONT = cv2.FONT_HERSHEY_SIMPLEX
CYAN, MAGENTA, RED, ORANGE, GREEN, WHITE, YELLOW = (255, 255, 0), (255, 0, 255), (0, 0, 255), (0, 140, 255), (0, 220, 0), (255, 255, 255), (0, 255, 255)


def fit(img, h=PANEL_H):
    s = h / img.shape[0]
    return cv2.resize(img, (int(img.shape[1] * s), h), interpolation=cv2.INTER_AREA), s


def label(img, text, y=22):
    cv2.putText(img, text, (8, y), FONT, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (8, y), FONT, 0.6, WHITE, 1, cv2.LINE_AA)


def grid_to_rect(planner, grid_img, rect_shape):
    inv = cv2.invertAffineTransform(planner.rect_to_grid)
    return cv2.warpAffine(grid_img, inv, (rect_shape[1], rect_shape[0]), flags=cv2.INTER_NEAREST)


def draw_robot(img, g, robot, colour=GREEN, scale=1.0):
    if robot is None:
        return
    u, v = g.world_to_rect([[robot["x"], robot["y"]]])[0] * scale
    a = np.radians(robot["heading"])
    tip = (u + 40 * np.cos(a), v - 40 * np.sin(a))              # rect rows go down, world y goes up
    cv2.circle(img, (int(u), int(v)), 6, colour, -1)
    cv2.arrowedLine(img, (int(u), int(v)), (int(tip[0]), int(tip[1])), colour, 2, tipLength=0.3)


def setup_panel(frame, tags, g, hint, tracker_state):
    """What the loop is waiting for during calibration: the tags it sees, the corners it needs, the
    tracker's own view of the robot if there is one."""
    panel = np.zeros((PANEL_H, 3 * PANEL_H // 2, 3), np.uint8)
    y = 26
    lines = ["CALIBRATING", "", f"corner tags seen: {[i for i in sorted(tags) if i in CORNER_IDS]}",
             f"corner tags needed: {[i for i in CORNER_IDS if i not in tags]}",
             f"robot tag: {'seen' if ROBOT_ID in tags else 'not seen'}   arm tag: {'seen' if ARM_ID in tags else 'not seen'}", ""]
    for chunk in (hint or "").split("; "):
        lines.append(chunk)
    if tracker_state:
        r = tracker_state.get("robot")
        lines += ["", f"Pi tracker: calibrated={tracker_state.get('calibrated')}  floor markers {tracker_state.get('floorMarkers')}  fps {tracker_state.get('fps')}",
                  (f"  robot at ({r['x']:.1f}, {r['y']:.1f}) cm heading {r['heading']:.0f}" if r and 'x' in r else "  robot not tracked")]
    lines += ["", "the tag panels fill in once the pose is frozen"]
    for line in lines:
        cv2.putText(panel, line, (10, y), FONT, 0.55, WHITE if line != "CALIBRATING" else YELLOW, 1, cv2.LINE_AA); y += 22
    return panel


def compose(frame, tags, g, rect, seg_out, planner, plan, robot, goal_xy, lookahead_xy, info, setup_hint="", tracker_state=None):
    """Returns the overlay image (BGR). The layout has the same size before and after calibration."""
    panels = []
    # 1. raw frame with tags
    raw = frame.copy()
    for i, q in tags.items():
        colour = GREEN if i in CORNER_IDS else (YELLOW if i == ROBOT_ID else ORANGE)
        cv2.polylines(raw, [q.astype(np.int32)], True, colour, 3)
        cv2.putText(raw, str(i), tuple(q[0].astype(int)), FONT, 1.4, colour, 3, cv2.LINE_AA)
    if g.frozen:
        for i in CORNER_IDS:
            cv2.polylines(raw, [g.project(g.corner_world[i]).astype(np.int32)], True, WHITE, 1)
    raw, _ = fit(raw); label(raw, "1 raw + tags"); panels.append(raw)

    if rect is None:
        panels.append(setup_panel(frame, tags, g, setup_hint, tracker_state))
        for i in range(2, 6):
            blank = np.zeros((PANEL_H, PANEL_H, 3), np.uint8); label(blank, f"{i + 1 if i > 2 else 3} (after calibration)"); panels.append(blank)
    else:
        r2, s = fit(rect); label(r2, "2 rectified"); panels.append(r2)
        # 3. obstacle mask
        m = rect.copy()
        if seg_out is not None:
            m[seg_out["persisted"]] = (0.4 * m[seg_out["persisted"]] + 0.6 * np.array(RED)).astype(np.uint8)
            m[seg_out["raw"] & ~seg_out["persisted"]] = (0.6 * m[seg_out["raw"] & ~seg_out["persisted"]] + 0.4 * np.array(YELLOW)).astype(np.uint8)
            for c in seg_out["components"]:
                if c["contour"] is not None:
                    cv2.drawContours(m, [c["contour"]], -1, WHITE, 2)
                x, y, w, h = c["bbox"]
                cv2.putText(m, f"{c['area_cm2']:.0f} cm2", (x, max(12, y - 4)), FONT, 0.5, WHITE, 1, cv2.LINE_AA)
            for o in seg_out.get("objects", []):                       # Sean's detector: outline in orange, name
                pts = np.round(g.world_to_rect(np.array(o["outline"], dtype=float))).astype(np.int32)
                cv2.polylines(m, [pts], True, ORANGE, 2)
                u, v = g.world_to_rect([[o["x"], o["y"]]])[0]
                cv2.putText(m, o["label"], (int(u) - 30, int(v) + 5), FONT, 0.5, ORANGE, 2, cv2.LINE_AA)
        m, _ = fit(m); label(m, "3 obstacles (red chroma, orange objects)"); panels.append(m)
        # 4. inflated C-space
        cs = (rect * 0.45).astype(np.uint8)
        if planner is not None and planner.free is not None:
            infl = grid_to_rect(planner, (planner.inflated & ~planner.occupied).astype(np.uint8), rect.shape) > 0
            occ = grid_to_rect(planner, planner.occupied.astype(np.uint8), rect.shape) > 0
            cs[infl] = (0.5 * cs[infl] + 0.5 * np.array(ORANGE)).astype(np.uint8)
            cs[occ] = (0.5 * cs[occ] + 0.5 * np.array(RED)).astype(np.uint8)
        cs, _ = fit(cs); label(cs, "4 inflated C-space"); panels.append(cs)
        # 5. field contours
        fc = (rect * 0.6).astype(np.uint8)
        if plan is not None and plan.get("field") is not None:
            field = plan["field"].copy()
            finite = np.isfinite(field)
            if finite.any():
                field[~finite] = np.nan
                fr = grid_to_rect(planner, field.astype(np.float32), rect.shape)
                fmax = np.nanmax(field)
                for level in np.arange(0, fmax, max(4.0, fmax / 14)):
                    band = np.abs(fr - level) < 0.6
                    fc[band] = CYAN
                fc[np.isnan(fr)] = (fc[np.isnan(fr)] * 0.5).astype(np.uint8)
        fc, _ = fit(fc); label(fc, "5 cost-to-go contours"); panels.append(fc)
        # 6+7. navigation
        nv = rect.copy()
        nv_s = PANEL_H / rect.shape[0]
        nv, _ = fit(nv)
        if plan is not None and len(plan.get("path_cm", [])) > 1:
            pts = (g.world_to_rect(plan["path_cm"]) * nv_s).astype(np.int32)
            cv2.polylines(nv, [pts], False, CYAN, 2)
            for wp in plan["waypoints"]:
                u, v = g.world_to_rect([wp])[0] * nv_s
                cv2.circle(nv, (int(u), int(v)), 4, MAGENTA, -1)
        if lookahead_xy is not None:
            u, v = g.world_to_rect([lookahead_xy])[0] * nv_s
            cv2.circle(nv, (int(u), int(v)), 7, YELLOW, 2)
        if goal_xy is not None:
            u, v = g.world_to_rect([goal_xy])[0] * nv_s
            cv2.drawMarker(nv, (int(u), int(v)), GREEN, cv2.MARKER_STAR, 22, 2)
        draw_robot(nv, g, robot, scale=nv_s)
        label(nv, "6+7 path, waypoints, lookahead, robot, goal"); panels.append(nv)
    # 8. text
    txt = np.zeros((PANEL_H, 400, 3), np.uint8)
    y = 26
    for line in info:
        cv2.putText(txt, line, (10, y), FONT, 0.55, WHITE, 1, cv2.LINE_AA); y += 24
    panels.append(txt)
    # arrange in two rows
    half = (len(panels) + 1) // 2
    rows = [np.hstack(panels[:half]), np.hstack(panels[half:])]
    w = max(r.shape[1] for r in rows)
    rows = [np.pad(r, ((0, 0), (0, w - r.shape[1]), (0, 0))) for r in rows]
    return np.vstack(rows)
