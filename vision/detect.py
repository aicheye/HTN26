"""Finds objects on the arena floor in top-down camera frames and returns oriented boxes in floor centimetres.

The tracker gives the camera pose for every frame, so each frame is first warped into a fixed top-down view of the
floor (1 mm per pixel). Two cues are combined over a set of frames from a moving camera:
 - colour: the board has one colour, and what is clearly not board-coloured in most frames is an object
 - parallax: the floor holds still in the warped views, and anything with height shifts as the camera moves, so
   its edges change brightness from frame to frame. This finds cardboard on a wooden board, which colour cannot,
   and it ignores shadows, which lie on the floor and do not shift.
Floor markers and the robot are left out.

The outlines themselves come from MobileSAM (vision/sam.py), prompted on a grid of points over the sharpest top
view. SAM draws a clean outline around whatever is under a point, including bare board, so the two cues above
decide which of its masks are objects: a mask is kept when its colour differs from the board, or when it has
moving edges along its border (it has height), or when it clearly differs from the ring of floor around it.

    python3 vision/detect.py recordings/rec-006            writes objects.json and objects.jpg into that folder
"""
import json
import sys
import time
import warnings
from pathlib import Path

import cv2
import numpy as np

warnings.filterwarnings("ignore", message="Degrees of freedom")  # numpy, for pixels no frame saw

PX_PER_CM = 10
MARGIN_CM = 6             # the view extends this far beyond the floor markers, the board is a little larger than them
FLOOR_MARKER_CM = 11      # cut square of a floor marker, plus slack
ROBOT_RADIUS_CM = 12      # legs included
MIN_AREA_CM2 = 3
ARENA_SLACK_CM = 2.5      # objects are looked for inside the floor-marker rectangle plus this much
VOTE_SHARE = 0.6          # a pixel is an object when it was one in this share of the frames that saw it


def floor_to_image(camera):
    """Homography taking floor points (x, y, 1) in cm to image pixels."""
    rotation = cv2.Rodrigues(np.array(camera["rvec"], dtype=float))[0]
    K = np.array([[camera["f"], 0, camera["cx"]], [0, camera["f"], camera["cy"]], [0, 0, 1.0]])
    return K @ np.column_stack([rotation[:, 0], rotation[:, 1], np.array(camera["tvec"], dtype=float)])


class TopView:
    """Fixed top-down view of the floor. Row 0 is the far side (largest y), so it looks as seen from above."""

    def __init__(self, floor_w, floor_h):
        self.w, self.h = floor_w, floor_h
        self.size = (int((floor_w + 2 * MARGIN_CM) * PX_PER_CM), int((floor_h + 2 * MARGIN_CM) * PX_PER_CM))
        # view pixel (col, row) -> floor cm: x = col / s - margin, y = (rows - row) / s - margin
        self.view_to_floor = np.array([[1 / PX_PER_CM, 0, -MARGIN_CM], [0, -1 / PX_PER_CM, self.size[1] / PX_PER_CM - MARGIN_CM], [0, 0, 1.0]])
        self.floor_to_view = np.linalg.inv(self.view_to_floor)

    def warp(self, image, camera, flags=cv2.INTER_LINEAR):
        return cv2.warpPerspective(image, floor_to_image(camera) @ self.view_to_floor, self.size, flags=flags | cv2.WARP_INVERSE_MAP)

    def to_view(self, x, y):
        p = self.floor_to_view @ np.array([x, y, 1.0])
        return int(round(p[0])), int(round(p[1]))

    def to_floor(self, points):
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        out = (self.view_to_floor @ np.column_stack([points, np.ones(len(points))]).T).T
        return out[:, :2]


def board_colour(lab, view):
    """Median colour of the middle of the view, where the board dominates."""
    s = PX_PER_CM
    middle = lab[(MARGIN_CM + 8) * s:-(MARGIN_CM + 8) * s, (MARGIN_CM + 8) * s:-(MARGIN_CM + 8) * s].reshape(-1, 3)
    return np.median(middle, axis=0)


def object_mask(top, view, state):
    """Pixels of the top view that are neither board, floor marker nor robot. Returns (mask, seen)."""
    lab = cv2.cvtColor(cv2.GaussianBlur(top, (0, 0), 1.5), cv2.COLOR_BGR2LAB).astype(np.float32)
    _, a0, b0 = board_colour(lab, view)
    chroma = np.hypot(lab[..., 1] - a0, lab[..., 2] - b0)
    # Brightness of the bare board around each pixel. The lighting is uneven (206 to 242 across the board), so one
    # number will not do. Closing fills in everything darker and smaller than 25 cm with its surroundings.
    small = cv2.resize(lab[..., 0], None, fx=0.125, fy=0.125, interpolation=cv2.INTER_AREA)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (int(25 * PX_PER_CM / 8),) * 2)
    local = cv2.resize(cv2.blur(cv2.morphologyEx(small, cv2.MORPH_CLOSE, kernel), (9, 9)), lab.shape[1::-1], interpolation=cv2.INTER_LINEAR)
    darker = local - lab[..., 0]
    # Only a clearly different colour counts here. Measured on rec-006: cardboard is just 9 to 12 away from the
    # board's colour and 64 to 70 darker, while a cast shadow is 4.5 to 9 away and 57 to 63 darker. Colour cannot
    # tell those two apart, so cardboard is left to the parallax cue in detect().
    mask = (chroma > 16) | (darker > 95) | (darker < -60)
    seen = top.sum(axis=2) > 0                      # outside the camera frame the warp leaves black
    seen = cv2.erode(seen.astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)

    ignore = np.zeros(mask.shape, np.uint8)
    half = int(FLOOR_MARKER_CM * PX_PER_CM / 2)
    for x, y in ((0, 0), (view.w, 0), (view.w, view.h), (0, view.h)):
        cx, cy = view.to_view(x, y)
        cv2.rectangle(ignore, (cx - half, cy - half), (cx + half, cy + half), 1, -1)
    robot = state.get("robot")
    if robot and state.get("camera"):
        # where the robot appears in the floor view: its marker's ray continued down to the floor
        p = np.linalg.inv(floor_to_image(state["camera"])) @ np.array([robot["px"][0], robot["px"][1], 1.0])
        cv2.circle(ignore, view.to_view(p[0] / p[2], p[1] / p[2]), ROBOT_RADIUS_CM * PX_PER_CM, 1, -1)
    x0, y1 = view.to_view(-ARENA_SLACK_CM, -ARENA_SLACK_CM)
    x1, y0 = view.to_view(view.w + ARENA_SLACK_CM, view.h + ARENA_SLACK_CM)
    arena = np.zeros(mask.shape, bool)
    arena[y0:y1, x0:x1] = True
    usable = seen & arena & (ignore == 0)
    return mask & usable, usable, lab[..., 0]


def white_balance(picture, view):
    """Corrects the camera's colour cast using the white paper border of the four floor markers as the reference.

    QNX's camera driver does not run auto white balance (the request fails), and everything comes out yellow-green.
    The marker borders are known to be white, which gives one gain per colour channel.
    """
    ring = np.zeros(picture.shape[:2], np.uint8)
    outer, inner = int(4.9 * PX_PER_CM), int(4.3 * PX_PER_CM)  # the black square is 8 cm, the paper about 10 cm
    for x, y in ((0, 0), (view.w, 0), (view.w, view.h), (0, view.h)):
        cx, cy = view.to_view(x, y)
        cv2.rectangle(ring, (cx - outer, cy - outer), (cx + outer, cy + outer), 1, -1)
        cv2.rectangle(ring, (cx - inner, cy - inner), (cx + inner, cy + inner), 0, -1)
    pixels = picture[(ring > 0) & (picture.sum(axis=2) > 0)].astype(np.float32)
    if len(pixels) < 500:
        return picture
    white = np.median(pixels[pixels.sum(axis=1) >= np.percentile(pixels.sum(axis=1), 50)], axis=0)  # brighter half: skips print bleed
    gains = white.max() / np.maximum(white, 1)
    return np.clip(picture.astype(np.float32) * gains, 0, 255).astype(np.uint8)


def rectangle_fill(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea)
    (_, _), (w, h), _ = cv2.minAreaRect(contour)
    return cv2.contourArea(contour) / max(w * h, 1.0)


def split_blob(blob, lab, depth=0):
    """Splits a blob that is not box-shaped (touching objects) into parts that are. Returns a list of masks.

    Two ways of cutting are tried: by colour (k-means inside the blob) and along brightness edges inside it, such as
    the gap between two cardboard boxes. A cut is kept only when every part is clearly more box-shaped than the
    whole and the parts cover it, so a single object with a busy print does not get shattered.
    """
    area = int(blob.sum())
    min_part = 8 * PX_PER_CM ** 2
    fill = rectangle_fill(blob)
    if depth >= 2 or fill >= 0.82 or area < 2 * min_part:
        return [blob]
    ys, xs = np.nonzero(blob)
    pixels = lab[ys, xs].astype(np.float32) * np.array([0.6, 1, 1], np.float32)
    candidates = []
    for k in (2, 3):
        _, labels, _ = cv2.kmeans(pixels, k, None, (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0), 2, cv2.KMEANS_PP_CENTERS)
        label_map = np.full(blob.shape, -1, np.int32)
        label_map[ys, xs] = labels.ravel()
        candidates.append([(label_map == i).astype(np.uint8) for i in range(k)])
    edges = cv2.Canny(cv2.GaussianBlur(lab[..., 0], (0, 0), 2).astype(np.uint8), 12, 36)
    candidates.append([blob & ~cv2.dilate(edges, np.ones((5, 5), np.uint8)).astype(bool)])

    # Objects with height show a side face and cast a shadow. Both are dark, irregular slivers attached to the
    # box-shaped top face (seen in rec-006: an upright green box next to a cardboard box). So a cut is judged by
    # its box-shaped parts only, and whatever is left over is dropped as side face or shadow.
    best, best_score = None, 0.0
    for groups in candidates:
        parts = []
        for group in groups:
            group = cv2.morphologyEx(group.astype(np.uint8), cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
            count, labels, stats, _ = cv2.connectedComponentsWithStats(group)
            parts += [(labels == i).astype(np.uint8) for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= min_part]
        boxes = [part for part in parts if rectangle_fill(part) >= 0.8]
        if not boxes:
            continue
        covered = sum(int(part.sum()) for part in boxes) / area
        score = covered * min(rectangle_fill(part) for part in boxes)
        if covered >= 0.45 and score > best_score:
            best, best_score, leftovers = boxes, score, [part for part in parts if rectangle_fill(part) < 0.8]
    if best is None:
        return [blob]
    # A leftover that completes a neighbouring box belongs to it: an upright box shows its top and one side in
    # different colours, and together they are one box-shaped silhouette. Leftovers that fit nowhere are dropped.
    for leftover in sorted(leftovers, key=lambda part: -int(part.sum())):
        near = cv2.dilate(leftover, np.ones((15, 15), np.uint8))
        touching = [(int((near & box).sum()), i) for i, box in enumerate(best) if (near & box).any()]
        if touching:
            _, i = max(touching)
            union = cv2.morphologyEx(best[i] | leftover, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
            if rectangle_fill(union) >= 0.78:
                best[i] = union
    # A small, clearly darker box-shaped part touching a larger one is that object's side face, not a second object.
    best.sort(key=lambda part: -int(part.sum()))
    brightness = [float(lab[..., 0][part > 0].mean()) for part in best]
    kept = []
    for i, part in enumerate(best):
        near = cv2.dilate(part, np.ones((15, 15), np.uint8))
        side_face = any((near & best[j]).any() and part.sum() < 0.4 * best[j].sum() and brightness[i] < brightness[j] - 25 for j in kept)
        if not side_face:
            kept.append(i)
    return [piece for i in kept for piece in split_blob(cv2.morphologyEx(best[i], cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8)), lab, depth + 1)]


GRID_CM = 3.5              # spacing of the prompt points
MIN_SCORE = 0.88


def sam_objects(picture, view, usable, colour_share, moving_edges, sam):
    """Prompts SAM on a grid and keeps the masks that the colour and parallax cues confirm. Returns a list of masks."""
    sam.set_image(picture)
    lab = cv2.cvtColor(cv2.GaussianBlur(picture, (0, 0), 1.5), cv2.COLOR_BGR2LAB).astype(np.float32)
    arena_area = float(usable.sum())
    step = int(GRID_CM * PX_PER_CM)
    # Points the cues already flag come first, so real objects claim their area before the bare-board points run.
    points = [(x, y) for y in range(step // 2, usable.shape[0], step) for x in range(step // 2, usable.shape[1], step) if usable[y, x]]
    points.sort(key=lambda p: -(colour_share[p[1], p[0]] + moving_edges[p[1], p[0]]))
    kept, scores = [], {}
    for x, y in points:
        if any(mask[y, x] for mask in kept):
            continue
        mask, score = sam.mask_at([(x, y)])
        area = float(mask.sum())
        if score < MIN_SCORE or area < MIN_AREA_CM2 * PX_PER_CM ** 2 or area > 0.25 * arena_area or not mask[y, x]:
            continue
        scores[id(mask)] = score
        if (mask & ~usable.astype(np.uint8)).sum() > 0.3 * area:      # mostly a floor marker, the robot, or outside
            continue
        # Evidence that this is an object and not a patch of board.
        inside = mask > 0
        border = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, np.ones((9, 9), np.uint8)) > 0
        ring = (cv2.dilate(mask, np.ones((31, 31), np.uint8)) > 0) & ~(cv2.dilate(mask, np.ones((9, 9), np.uint8)) > 0) & usable
        coloured = float(colour_share[inside].mean())
        raised = float(moving_edges[border].mean())
        contrast = float(np.linalg.norm(lab[inside].mean(axis=0) - lab[ring].mean(axis=0))) if ring.any() else 0.0
        if not (coloured > 0.35 or raised > 0.25 or contrast > 22):
            continue
        # One object, one mask: drop parts of a whole. When the new mask is the whole, it replaces its parts.
        if any((inside & (other > 0)).sum() > 0.8 * area for other in kept):
            continue
        kept = [other for other in kept if (inside & (other > 0)).sum() < 0.8 * other.sum()]
        kept.append(mask)
    return kept, [round(scores[id(mask)], 2) for mask in kept]


def detect(frames, floor=(63, 63), sam=None, return_masks=False):
    """frames: list of (BGR image, tracker state). Returns (objects, top view image, mask)."""
    view = TopView(*floor)
    votes = np.zeros(view.size[::-1], np.float32)
    views = np.zeros(view.size[::-1], np.float32)
    colour = np.zeros((*view.size[::-1], 3), np.float32)
    brightness = []
    for image, state in frames:
        top = view.warp(image, state["camera"])
        mask, seen, L = object_mask(top, view, state)
        votes += mask
        views += seen
        colour += top * seen[..., None]
        brightness.append(np.where(seen, L, np.nan).astype(np.float32))
    mean_top = (colour / np.maximum(views, 1)[..., None]).astype(np.uint8)
    # The mean is blurred, because tall objects sit in a slightly different place in every view. For display and
    # textures use one frame: the one taken from closest to straight above the middle, with all four markers.
    def off_centre(state):
        rotation = cv2.Rodrigues(np.array(state["camera"]["rvec"], dtype=float))[0]
        centre = -rotation.T @ np.array(state["camera"]["tvec"], dtype=float)
        return np.hypot(centre[0] - floor[0] / 2, centre[1] - floor[1] / 2) + (0 if state["floorMarkers"] == 4 else 1000)
    best_image, best_state = min(frames, key=lambda f: off_centre(f[1]))
    picture = view.warp(best_image, best_state["camera"])
    share = votes / np.maximum(views, 1)
    enough = views >= max(1, 0.3 * len(frames))
    objects_mask = (share >= VOTE_SHARE) & enough

    # Parallax cue: how much each floor pixel's brightness changes between views. Bare board changes by about 6
    # (camera pose jitter on the wood grain), edges of objects with height by 30 to 60.
    with np.errstate(all="ignore"):
        spread = np.nan_to_num(np.nanstd(np.array(brightness), axis=0))
    spread[~enough] = 0
    moving_edges = spread > max(18.0, 2.8 * float(np.median(spread[enough])))
    # Edges are outlines. Close small gaps in them, then fill every closed outline to get the object's area.
    outlines = cv2.morphologyEx((objects_mask | moving_edges).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    filled = np.zeros_like(outlines)
    cv2.drawContours(filled, cv2.findContours(outlines, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], -1, 1, -1)
    objects_mask = filled.astype(bool)

    # Opening removes outline fragments that never closed into an area (they are only a few mm wide).
    clean = cv2.morphologyEx(objects_mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((13, 13), np.uint8))
    if sam is not None:
        usable = enough & (views >= 0.9 * views.max())
        pieces, piece_scores = sam_objects(picture, view, usable, share * enough, moving_edges.astype(np.float32), sam)
    else:  # without the model: blobs from the two cues, cut apart where that yields box shapes
        mean_lab = cv2.cvtColor(cv2.GaussianBlur(mean_top, (0, 0), 1.5), cv2.COLOR_BGR2LAB).astype(np.float32)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(clean)
        pieces = []
        for i in range(1, count):
            if stats[i, cv2.CC_STAT_AREA] >= MIN_AREA_CM2 * PX_PER_CM ** 2:
                pieces += split_blob((labels == i).astype(np.uint8), mean_lab)
    if sam is None:
        piece_scores = [0.5] * len(pieces)
    picture = white_balance(picture, view)  # after detection, which works relative to the board colour anyway
    clean = np.zeros_like(clean)
    objects, object_masks = [], []
    for piece, piece_score in zip(pieces, piece_scores):
        found, _ = cv2.findContours(piece, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour = max(found, key=cv2.contourArea)
        clean |= piece
        area = cv2.contourArea(contour) / PX_PER_CM ** 2
        if area < MIN_AREA_CM2:
            continue
        (cx, cy), (w, h), _ = cv2.minAreaRect(contour)
        corners = view.to_floor(cv2.boxPoints(cv2.minAreaRect(contour)))
        edge = corners[1] - corners[0]
        x, y = view.to_floor([(cx, cy)])[0]
        inside = np.zeros(clean.shape, np.uint8)
        cv2.drawContours(inside, [contour], -1, 1, -1)
        b, g, r = cv2.mean(picture, mask=inside)[:3]
        outline = cv2.approxPolyDP(contour, 0.012 * cv2.arcLength(contour, True), True)
        objects.append({
            "x": round(float(x), 1), "y": round(float(y), 1),
            "width": round(float(np.linalg.norm(edge)), 1), "length": round(float(np.linalg.norm(corners[2] - corners[1])), 1),
            "yaw": round(float(np.arctan2(edge[1], edge[0])), 3),
            "area": round(float(area), 1), "fill": round(float(area / max(w * h / PX_PER_CM ** 2, 1e-6)), 2),
            "colour": [int(r), int(g), int(b)], "score": piece_score,
            "corners": [[round(float(px), 1), round(float(py), 1)] for px, py in corners],
            "outline": [[round(float(px), 1), round(float(py), 1)] for px, py in view.to_floor(outline.reshape(-1, 2))],
        })
        object_masks.append(piece)
    order = sorted(range(len(objects)), key=lambda i: -objects[i]["area"])
    objects, object_masks = [objects[i] for i in order], [object_masks[i] for i in order]
    if return_masks:
        return objects, picture, clean, view, object_masks
    return objects, picture, clean, view


def draw(objects, top, view):
    out = top.copy()
    for i, o in enumerate(objects):
        box = np.array([view.to_view(x, y) for x, y in o["corners"]], np.int32)
        outline = np.array([view.to_view(x, y) for x, y in o["outline"]], np.int32)
        cv2.polylines(out, [outline], True, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.polylines(out, [box], True, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(out, str(i + 1), tuple(box.mean(axis=0).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
    return out


def main():
    folder = Path(sys.argv[1])
    frames = []
    for line in (folder / "states.jsonl").read_text().splitlines():
        entry = json.loads(line)
        state = entry["state"]
        if state.get("camera") and state["floorMarkers"] >= 3:
            frames.append((cv2.imread(str(folder / entry["file"])), state))
    frames = frames[:: max(1, len(frames) // 25)]
    try:
        from sam import Sam
        sam = Sam()
    except Exception as error:  # model files or onnxruntime missing
        print(f"MobileSAM not available ({error}), using the colour and parallax cues alone")
        sam = None
    started = time.time()
    objects, top, mask, view = detect(frames, tuple(frames[0][1]["floor"]), sam)
    print(f"detection took {time.time() - started:.1f} s")
    (folder / "objects.json").write_text(json.dumps(objects, indent=1))
    cv2.imwrite(str(folder / "objects.jpg"), draw(objects, top, view))
    cv2.imwrite(str(folder / "objects-mask.png"), mask * 255)
    print(f"{len(frames)} frames -> {len(objects)} objects")
    for i, o in enumerate(objects):
        print(f"  {i + 1}: centre ({o['x']}, {o['y']}) cm, {o['width']} x {o['length']} cm, turned {np.degrees(o['yaw']):.0f} deg, box fill {o['fill']}, colour {o['colour']}")


if __name__ == "__main__":
    main()
