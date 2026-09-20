"""Stage 4: costmap and cost-to-go planner.

Grid of CELL_CM cells over the arena polygon, padded beyond the tag boundary so the boundary has outside
cells to inflate inward from. Obstacles (from the persisted segmentation mask) and everything outside the
polygon are occupied; occupancy is inflated by the robot radius plus one cell of slack so the robot is a
point. A soft cost near walls centres paths in corridors. Dijkstra runs from the goal over the free cells,
giving a cost-to-go field with no local minima: steepest descent from anywhere reaches the goal, and the
robot can be picked up and moved without replanning. Replanned from scratch every tick.
"""
import time

import numpy as np
import cv2
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

CELL_CM = 1.0
SLACK_CELLS = 1
SOFT_WEIGHT = 3.0          # extra cost right at the inflation edge, fading to 0 over SOFT_BAND
NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


class Planner:
    def __init__(self, geometry, robot_radius_cm, cell_cm=CELL_CM):
        self.g = geometry
        self.cell = cell_cm
        self.radius_cm = robot_radius_cm
        self.r_cells = int(np.ceil(robot_radius_cm / cell_cm)) + SLACK_CELLS
        pad = self.r_cells * cell_cm + 4 * cell_cm
        poly = geometry.polygon
        self.x0, self.y1 = float(poly[:, 0].min() - pad), float(poly[:, 1].max() + pad)
        self.cols = int(np.ceil((poly[:, 0].max() + pad - self.x0) / cell_cm))
        self.rows = int(np.ceil((self.y1 - (poly[:, 1].min() - pad)) / cell_cm))
        self.inside = np.zeros((self.rows, self.cols), np.uint8)
        cv2.fillPoly(self.inside, [np.round(self.world_to_cell(poly)).astype(np.int32)], 255)
        self.inside = self.inside > 0
        # rectified image -> grid: both are axis aligned with y down, only scale and offset differ
        r = geometry.rect
        s = r["cm_per_px"] / cell_cm
        self.rect_to_grid = np.array([[s, 0, (r["x0"] - self.x0) / cell_cm], [0, s, (self.y1 - r["y1"]) / cell_cm]])
        self._build_graph()
        self.free = self.cost = self.field = None
        # warm up: the first update and plan pay for allocations and scipy's first call, not for planning
        self.update(np.zeros((geometry.rect["h"], geometry.rect["w"]), bool))
        self.plan(self.cell_to_world([(self.rows // 2, self.cols // 2)])[0], (self.rows // 2, self.cols // 2))

    # --- coordinates ---------------------------------------------------------------------------------------
    def world_to_cell(self, xy):
        xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        return np.column_stack([(xy[:, 0] - self.x0) / self.cell, (self.y1 - xy[:, 1]) / self.cell])   # (col, row), float

    def cell_to_world(self, rc):
        """(row, col) cells -> world cm at the cell centres. Cells are (row, col) everywhere in this class."""
        rc = np.asarray(rc, dtype=np.float64).reshape(-1, 2)
        return np.column_stack([self.x0 + (rc[:, 1] + 0.5) * self.cell, self.y1 - (rc[:, 0] + 0.5) * self.cell])

    def index(self, xy):
        c, r = np.floor(self.world_to_cell(xy)[0]).astype(int)
        return int(np.clip(r, 0, self.rows - 1)), int(np.clip(c, 0, self.cols - 1))

    # --- graph ---------------------------------------------------------------------------------------------
    def _build_graph(self):
        """Edge lists of the 8-connected grid, built once; weights are refilled every tick."""
        rows, cols = self.rows, self.cols
        rr, cc = np.meshgrid(np.arange(rows), np.arange(cols), indexing="ij")
        src, dst, length, ortho_a, ortho_b = [], [], [], [], []
        for dr, dc in NEIGHBOURS:
            if (dr, dc) < (0, 0) or (dr == 0 and dc < 0):
                continue                                          # each undirected edge once
            ok = (rr + dr >= 0) & (rr + dr < rows) & (cc + dc >= 0) & (cc + dc < cols)
            a, b = (rr[ok] * cols + cc[ok]), ((rr[ok] + dr) * cols + (cc[ok] + dc))
            src.append(a); dst.append(b); length.append(np.full(len(a), np.hypot(dr, dc)))
            # the two cells a diagonal step passes between; for straight steps point at the endpoints
            if dr and dc:
                ortho_a.append(rr[ok] * cols + (cc[ok] + dc)); ortho_b.append((rr[ok] + dr) * cols + cc[ok])
            else:
                ortho_a.append(a); ortho_b.append(b)
        self.e_src, self.e_dst = np.concatenate(src), np.concatenate(dst)
        self.e_len = np.concatenate(length) * self.cell
        self.e_oa, self.e_ob = np.concatenate(ortho_a), np.concatenate(ortho_b)

    # --- costmap -------------------------------------------------------------------------------------------
    def update(self, persisted_rect_mask):
        """Occupancy from the persisted obstacle mask (rectified px) plus the boundary, then inflation and cost."""
        occ_rect = persisted_rect_mask.astype(np.uint8) * 255
        occ = cv2.warpAffine(occ_rect, self.rect_to_grid, (self.cols, self.rows), flags=cv2.INTER_AREA) > 0
        occ |= ~self.inside
        self.occupied = occ
        dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5)     # cells to the nearest occupied
        self.inflated = dist <= self.r_cells
        self.free = ~self.inflated
        band = max(1.0, self.radius_cm / self.cell)
        self.cost = 1.0 + SOFT_WEIGHT * np.clip(1.0 - (dist - self.r_cells) / band, 0.0, 1.0)
        self.dist = dist

    def nearest_free(self, rc, max_cells=None):
        """Nearest free cell to (row, col), for a start inside the inflation band. None if nothing is near."""
        r, c = rc
        if self.free[r, c]:
            return rc
        max_cells = max_cells or (self.r_cells + 4)
        r0, r1 = max(0, r - max_cells), min(self.rows, r + max_cells + 1)
        c0, c1 = max(0, c - max_cells), min(self.cols, c + max_cells + 1)
        ys, xs = np.nonzero(self.free[r0:r1, c0:c1])
        if len(ys) == 0:
            return None
        d = np.hypot(ys + r0 - r, xs + c0 - c)
        k = d.argmin()
        return int(ys[k] + r0), int(xs[k] + c0)

    def region_of(self, rc):
        """Label image of free connected regions and the label containing rc."""
        n, labels = cv2.connectedComponents(self.free.astype(np.uint8), connectivity=8)
        return labels, int(labels[rc])

    def pick_goal(self, start_rc, rng, min_fraction=0.6):
        """A random free cell in the start's connected region, among the farther ones. None if the region is tiny."""
        start = self.nearest_free(start_rc)
        if start is None:
            return None
        labels, lab = self.region_of(start)
        ys, xs = np.nonzero((labels == lab) & (self.dist > self.r_cells + 2))
        if len(ys) < 10:                                    # a cramped arena: any free cell in the region will do
            ys, xs = np.nonzero(labels == lab)
        if len(ys) < 4:
            return None
        d = np.hypot(ys - start[0], xs - start[1])
        far = d >= min_fraction * d.max()
        k = rng.choice(np.nonzero(far)[0])
        return int(ys[k]), int(xs[k])

    # --- planning ------------------------------------------------------------------------------------------
    def plan(self, start_xy, goal_rc):
        """Cost-to-go field from goal_rc and the steepest-descent path from start_xy. Returns a dict."""
        t0 = time.perf_counter()
        free = self.free.ravel()
        cost = self.cost.ravel()
        ok = free[self.e_src] & free[self.e_dst] & free[self.e_oa] & free[self.e_ob]   # no cutting occupied corners
        w = self.e_len[ok] * 0.5 * (cost[self.e_src[ok]] + cost[self.e_dst[ok]])
        n = self.rows * self.cols
        graph = csr_matrix((w, (self.e_src[ok], self.e_dst[ok])), shape=(n, n))
        goal = None if goal_rc is None else self.nearest_free(goal_rc)
        if goal is None:
            return {"field": None, "path": [], "blocked": True, "ms": 1000 * (time.perf_counter() - t0)}
        field = dijkstra(graph, directed=False, indices=goal[0] * self.cols + goal[1]).reshape(self.rows, self.cols)
        self.field = field
        start_rc = self.index(start_xy)
        start = self.nearest_free(start_rc)
        blocked = start is None or not np.isfinite(field[start])
        path = [] if blocked else self.descend(start, goal)
        return {"field": field, "path": path, "path_cm": self.cell_to_world(path) if path else np.zeros((0, 2)),
                "waypoints": self.waypoints(path), "start": start, "goal": goal, "start_rescued": start != start_rc,
                "blocked": blocked, "ms": 1000 * (time.perf_counter() - t0)}

    def descend(self, start, goal):
        """Steepest descent on the field from start to goal, as (row, col) cells."""
        path = [start]
        r, c = start
        for _ in range(self.rows * self.cols):
            if (r, c) == goal:
                break
            best, bv = None, self.field[r, c]
            for dr, dc in NEIGHBOURS:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols and self.field[nr, nc] < bv:
                    if dr and dc and not (self.free[r, nc] and self.free[nr, c]):
                        continue
                    best, bv = (nr, nc), self.field[nr, nc]
            if best is None:
                break
            r, c = best
            path.append(best)
        return path

    def waypoints(self, path):
        """World-cm corners of the path: keep cells where the step direction changes, plus both ends."""
        if len(path) < 2:
            return self.cell_to_world(path) if path else np.zeros((0, 2))
        keep = [path[0]]
        for a, b, c in zip(path, path[1:], path[2:]):
            if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
                keep.append(b)
        keep.append(path[-1])
        return self.cell_to_world(keep)
