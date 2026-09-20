import { useEffect, useRef } from "react";
import type { MapProps } from "./MapProps";
import { armGeometry, displayRobot, origin, projectedOutline, sesameTopView, type Solid } from "../robot/geometry";
import { cornerTags, markerImage, obstacleImage, tableBorder, woodCanvas } from "./sceneSurface";
import {
  EDGE_LIMIT,
  edgeLimit,
  DANGER_M,
  FLOOR_EDGE,
  GRID,
  ROBOT_FITTING,
  ROBOT_FITTING_INSET,
  isCarried,
  obstacleColor,
  obstacleOutline,
  OBSTACLE_DANGER,
  TILE_M,
  distanceTo,
} from "./mapShared";
import type { Obstacle, Point, Robot, WorldState } from "../types/world";

const PADDING = 20; // px around the arena

type View = { scale: number; offsetX: number; offsetY: number; height: number };

export function Map2D({
  state,
  showCameraLayer = false,
  compact = false,
  onPickGoal,
}: MapProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);
  const viewRef = useRef<View>({ scale: 1, offsetX: 0, offsetY: 0, height: 0 });

  useEffect(() => {
    const canvas = canvasRef.current;
    const wrap = wrapRef.current;
    if (!canvas || !wrap) return;

    const draw = () => {
      const ctx = canvas.getContext("2d");
      if (!ctx) return;
      const dpr = window.devicePixelRatio || 1;
      const w = wrap.clientWidth;
      const h = wrap.clientHeight;
      if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width = w * dpr;
        canvas.height = h * dpr;
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      viewRef.current = computeView(state, w, h, compact ? 8 : PADDING);
      render(ctx, w, h, state, viewRef.current, showCameraLayer, compact);
    };

    (state.arena.cornerTagIds ?? []).forEach((id) => markerImage(id, draw));
    state.robots.forEach((r) => markerImage(r.tagId, draw));
    state.obstacles.forEach((o) => o.textureUrl && obstacleImage(o.id, o.textureUrl, draw));
    draw();
    const ro = new ResizeObserver(draw);
    ro.observe(wrap);
    let frame = 0;
    const animate = () => { draw(); frame = requestAnimationFrame(animate); };
    if (state.robots.some((r) => r.mode === "moving" || r.mode === "turning")) frame = requestAnimationFrame(animate);
    return () => { ro.disconnect(); cancelAnimationFrame(frame); };
  }, [state, showCameraLayer, compact]);

  const handleClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!onPickGoal) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const { scale, offsetX, offsetY, height } = viewRef.current;
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const x = (px - offsetX) / scale;
    const y = (height - py - offsetY) / scale;
    if (x < 0 || y < 0 || x > state.arena.width || y > state.arena.length) return;
    onPickGoal({ x, y });
  };

  return (
    <div ref={wrapRef} className="relative h-full w-full">
      <canvas
        ref={canvasRef}
        onClick={handleClick}
        className="h-full w-full cursor-crosshair rounded-lg"
      />
    </div>
  );
}

function computeView(state: WorldState, w: number, h: number, pad = PADDING): View {
  const { width, length } = state.arena;
  const border = tableBorder(state.arena);
  const scale = Math.max(1, Math.min((w - pad * 2) / (width + border * 2), (h - pad * 2) / (length + border * 2)));
  return {
    scale,
    offsetX: (w - width * scale) / 2,
    offsetY: (h - length * scale) / 2,
    height: h,
  };
}

/** World meters -> canvas pixels (y flipped so +y points up the screen). */
function toPx(p: Point, v: View): [number, number] {
  return [v.offsetX + p.x * v.scale, v.height - (v.offsetY + p.y * v.scale)];
}

const FLOOR = "#e2e8f0";

function render(
  ctx: CanvasRenderingContext2D,
  w: number,
  h: number,
  state: WorldState,
  v: View,
  showCameraLayer: boolean,
  compact: boolean,
) {
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#f8fafc";
  ctx.fillRect(0, 0, w, h);

  const tracked = state.robots.find((r) => r.tracking);
  drawFloor(ctx, state, v);
  if (showCameraLayer) drawCameraPlaceholder(ctx, state, v);
  drawCornerTags(ctx, state, v);
  state.obstacles.forEach((o) =>
    drawObstacle(ctx, o, v, tracked ? distanceTo(o, tracked) < DANGER_M : false),
  );
  drawPath(ctx, state.path, v);
  if (state.goal) drawGoal(ctx, state.goal, v);
  state.robots.forEach((r) => drawRobot(ctx, r, v, isCarried(state.arm, r.id)));
  if (state.arm) drawArm(ctx, state, v);
  if (!compact) drawScaleBar(ctx, state, v);
}

function drawFloor(ctx: CanvasRenderingContext2D, state: WorldState, v: View) {
  const { width, length } = state.arena;
  const [x, y] = toPx({ x: 0, y: length }, v);
  const wpx = width * v.scale;
  const hpx = length * v.scale;

  ctx.save();
  const border = tableBorder(state.arena) * v.scale;
  roundRect(ctx, x - border, y - border, wpx + border * 2, hpx + border * 2, 8);
  ctx.clip();
  if (state.arena.surface !== "grid") ctx.drawImage(woodCanvas(), x - border, y - border, wpx + border * 2, hpx + border * 2);
  else {
    ctx.fillStyle = FLOOR;
    ctx.fillRect(x - border, y - border, wpx + border * 2, hpx + border * 2);
  }

  ctx.beginPath();
  ctx.rect(x, y, wpx, hpx);
  ctx.clip();
  ctx.strokeStyle = state.arena.surface === "grid" ? GRID : "transparent";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  for (let gx = TILE_M; gx < width; gx += TILE_M) {
    const [px] = toPx({ x: gx, y: 0 }, v);
    ctx.moveTo(px, y);
    ctx.lineTo(px, y + hpx);
  }
  for (let gy = TILE_M; gy < length; gy += TILE_M) {
    const [, py] = toPx({ x: 0, y: gy }, v);
    ctx.moveTo(x, py);
    ctx.lineTo(x + wpx, py);
  }
  ctx.stroke();
  ctx.restore();

  ctx.save();
  ctx.strokeStyle = FLOOR_EDGE;
  ctx.lineWidth = 2;
  ctx.strokeRect(x, y, wpx, hpx);
  ctx.restore();

  // The limit the robot's centre keeps to, as a dashed line. A goal clicked outside it is moved onto it by the planner.
  const margin = edgeLimit(state.arena);
  if (margin) {
    const limit = margin * v.scale;
    ctx.save();
    ctx.globalAlpha = 0.75;
    ctx.strokeStyle = EDGE_LIMIT;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([6, 4]);
    ctx.strokeRect(x + limit, y + limit, wpx - limit * 2, hpx - limit * 2);
    ctx.restore();
  }
}

/** Fixed calibration markers use the same ArUco artwork as the printed sheets. */
function drawCornerTags(ctx: CanvasRenderingContext2D, state: WorldState, v: View) {
  const size = (state.arena.tagSize ?? 0.08) * v.scale;
  ctx.save();
  cornerTags(state.arena).forEach((c) => {
    const [px, py] = toPx(c, v);
    drawMarker(ctx, c.id, px, py, size);
  });
  ctx.restore();
}

/** Picks a round-number bar length that comfortably fits inside the arena. */
function niceScaleMeters(maxMeters: number): number {
  const candidates = [1, 0.5, 0.25, 0.2, 0.1, 0.05, 0.02, 0.01];
  return candidates.find((c) => c <= maxMeters) ?? candidates[candidates.length - 1];
}

/** Scale reference, drawn inside the arena so it reads as part of the map. */
function drawScaleBar(ctx: CanvasRenderingContext2D, state: WorldState, v: View) {
  const meters = niceScaleMeters(Math.min(state.arena.width, state.arena.length) / 3);
  const len = meters * v.scale;
  const [left, bottom] = toPx({ x: state.arena.width / 2, y: 0 }, v);
  const x = left - len / 2;
  const y = bottom - 18;

  ctx.save();
  ctx.strokeStyle = "rgba(15, 23, 42, 0.35)";
  ctx.lineWidth = 2;
  ctx.lineCap = "butt";
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x + len, y);
  ctx.moveTo(x, y - 4);
  ctx.lineTo(x, y + 4);
  ctx.moveTo(x + len, y - 4);
  ctx.lineTo(x + len, y + 4);
  ctx.stroke();
  ctx.fillStyle = "rgba(15, 23, 42, 0.5)";
  ctx.font = "10px ui-monospace, monospace";
  ctx.textAlign = "center";
  const label = meters >= 1 ? `${meters} m` : `${Math.round(meters * 100)} cm`;
  ctx.fillText(label, x + len / 2, y - 8);
  ctx.restore();
}

function drawCameraPlaceholder(ctx: CanvasRenderingContext2D, state: WorldState, v: View) {
  const [x0, y0] = toPx({ x: 0, y: state.arena.length }, v);
  const wpx = state.arena.width * v.scale;
  const hpx = state.arena.length * v.scale;
  ctx.save();
  ctx.beginPath();
  ctx.rect(x0, y0, wpx, hpx);
  ctx.clip();
  ctx.fillStyle = "rgba(15, 23, 42, 0.06)";
  ctx.fillRect(x0, y0, wpx, hpx);
  ctx.fillStyle = "rgba(15, 23, 42, 0.45)";
  ctx.font = "12px ui-monospace, monospace";
  ctx.textAlign = "center";
  ctx.fillText("CAMERA FEED — NO STREAM", x0 + wpx / 2, y0 + hpx / 2);
  ctx.restore();
}

/**
 * Exact detected geometry in pixels. Polygons come straight from the contour the
 * vision side traced; rects and circles are the analytic shapes they reported.
 */
function outline(o: Obstacle, v: View): [number, number][] {
  return obstacleOutline(o).map((p) => toPx(p, v));
}

function polyPath(ctx: CanvasRenderingContext2D, pts: [number, number][]) {
  ctx.beginPath();
  pts.forEach(([x, y], i) => {
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.closePath();
}

function drawObstacle(
  ctx: CanvasRenderingContext2D,
  o: Obstacle,
  v: View,
  danger: boolean,
) {
  const pts = outline(o, v);

  ctx.save();
  polyPath(ctx, pts);
  ctx.fillStyle = obstacleColor(o);
  ctx.shadowColor = "rgba(45, 32, 18, 0.2)";
  ctx.shadowBlur = 4;
  ctx.shadowOffsetY = 2;
  ctx.fill();
  ctx.shadowColor = "transparent";
  const photo = o.textureUrl && obstacleImage(o.id, o.textureUrl);
  if (photo) {
    // The photo covers the oriented box. Canvas y points down, so a counter-clockwise yaw is a negative rotation.
    const [cx, cy] = toPx(o, v);
    const wpx = (o.width ?? 0.2) * v.scale, lpx = (o.length ?? 0.2) * v.scale;
    ctx.save();
    ctx.clip();
    ctx.translate(cx, cy);
    ctx.rotate(-o.yaw);
    ctx.drawImage(photo, -wpx / 2, -lpx / 2, wpx, lpx);
    ctx.restore();
  }
  ctx.strokeStyle = danger ? OBSTACLE_DANGER : "#334155";
  ctx.lineWidth = danger ? 2.5 : 1;
  if (o.height === undefined) ctx.setLineDash([4, 3]);
  ctx.stroke();
  ctx.restore();
}

function drawPath(ctx: CanvasRenderingContext2D, path: Point[] | undefined, v: View) {
  if (!path || path.length < 2) return;
  ctx.save();
  ctx.setLineDash([8, 6]);
  ctx.strokeStyle = "rgba(15, 23, 42, 0.55)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  path.forEach((p, i) => {
    const [px, py] = toPx(p, v);
    if (i === 0) ctx.moveTo(px, py);
    else ctx.lineTo(px, py);
  });
  ctx.stroke();
  ctx.restore();
}

function drawGoal(ctx: CanvasRenderingContext2D, goal: Point, v: View) {
  const [x, y] = toPx(goal, v);
  ctx.save();
  ctx.beginPath();
  ctx.arc(x, y, 12, 0, Math.PI * 2);
  ctx.fillStyle = "rgba(255,255,255,0.85)";
  ctx.fill();
  ctx.strokeStyle = "rgba(220,38,38,0.25)";
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(x, y, 7, 0, Math.PI * 2);
  ctx.strokeStyle = "#dc2626";
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(x, y, 2.5, 0, Math.PI * 2);
  ctx.fillStyle = "#dc2626";
  ctx.fill();
  ctx.restore();
}

/** Top-down view of the pick-and-place arm: a bracket at its table-edge mount. */
function drawArm(ctx: CanvasRenderingContext2D, state: WorldState, v: View) {
  const arm = state.arm;
  if (!arm) return;
  const [x, y] = toPx(arm.mount, v);
  const active = arm.mode !== "idle";
  drawSolids(ctx, armGeometry(arm.joints), arm.mount, v);

  if (active) {
    const target = state.robots.find((rob) => rob.id === arm.targetRobotId);
    if (target) {
      const [tx, ty] = toPx(target, v);
      ctx.save();
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = "rgba(249, 115, 22, 0.7)";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(tx, ty);
      ctx.stroke();
      ctx.restore();
    }
    ctx.save();
    ctx.font = "600 9px ui-monospace, monospace";
    ctx.textAlign = "center";
    ctx.fillStyle = "#c2410c";
    ctx.fillText(arm.mode.toUpperCase(), x, y + (arm.mount.side === "north" ? 18 : -14));
    ctx.restore();
  }
}

/**
 * Top-down projection of the same URDF collision-envelope geometry used in 3D.
 * Sesame is two MG90 servos per leg, so the legs sit outboard of the shell.
 */
function drawRobot(ctx: CanvasRenderingContext2D, r: Robot, v: View, carried: boolean) {
  const [cx, cy] = toPx({ x: r.x, y: r.y }, v);
  const L = 0.084 * v.scale; // along heading
  const W = 0.068 * v.scale; // across

  // Reported angles take precedence over the illustrative diagonal-pair walking gait.
  const model = sesameTopView(displayRobot(r, performance.now() / 1000, carried), 0.6);
  const s = v.scale;
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-r.yaw);
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  for (const { hip, knee, foot } of model.legs) {
    ctx.beginPath();
    ctx.moveTo(hip.x * s, -hip.y * s);
    ctx.lineTo(knee.x * s, -knee.y * s);
    ctx.lineTo(foot.x * s, -foot.y * s);
    ctx.lineWidth = 0.014 * s;
    ctx.strokeStyle = r.tracking ? "#14171b" : "#9ba6af";
    ctx.stroke();
    ctx.lineWidth = 0.006 * s;
    ctx.strokeStyle = r.tracking ? "#2b2f35" : "#bac3cb";
    ctx.stroke();
    ctx.save();
    ctx.translate(foot.x * s, -foot.y * s);
    ctx.rotate(Math.atan2(knee.y - foot.y, foot.x - knee.x));
    roundRect(ctx, -0.008 * s, -0.005 * s, 0.016 * s, 0.01 * s, 0.004 * s);
    ctx.fillStyle = r.tracking ? "#14171b" : "#9ba6af";
    ctx.fill();
    ctx.restore();
  }
  const bx = Math.min(...model.shell.map((p) => p.x)) * s;
  const by = -Math.max(...model.shell.map((p) => p.y)) * s;
  const paint = ctx.createLinearGradient(bx, by, bx, by + W);
  paint.addColorStop(0, r.tracking ? r.shellColor ?? "#24272c" : "#bac3cb");
  paint.addColorStop(1, r.tracking ? r.shellColor ?? "#121418" : "#8d99a4");
  roundRect(ctx, bx, by, L, W, 0.01 * s);
  ctx.fillStyle = paint;
  ctx.shadowColor = "rgba(15, 23, 42, 0.16)";
  ctx.shadowBlur = 4;
  ctx.shadowOffsetY = 2;
  ctx.fill();
  ctx.shadowColor = "transparent";
  ctx.strokeStyle = r.tracking ? "#1e2731" : "#84929e";
  ctx.lineWidth = 1;
  ctx.stroke();
  for (const handle of model.handles) {
    const x = Math.min(...handle.map((p) => p.x)) * s;
    const y = -Math.max(...handle.map((p) => p.y)) * s;
    const width = Math.max(...handle.map((p) => p.x)) * s - x;
    const height = -Math.min(...handle.map((p) => p.y)) * s - y;
    roundRect(ctx, x, y, width, height, height / 2);
    ctx.fillStyle = r.tracking ? "#363b42" : "#c3ccd4";
    ctx.fill();
    ctx.strokeStyle = r.tracking ? "#59616b" : "#e2e8ed";
    ctx.lineWidth = 0.65;
    ctx.stroke();
  }
  drawEyes(ctx, L, W, r.tracking);
  ctx.beginPath();
  ctx.moveTo(0.055 * s, -0.004 * s);
  ctx.lineTo(0.047 * s, -0.008 * s);
  ctx.lineTo(0.047 * s, 0);
  ctx.closePath();
  ctx.fillStyle = "#64748b";
  ctx.fill();
  ctx.translate(0.001 * s, -0.004 * s);
  ctx.rotate(Math.PI / 2);
  drawMarker(ctx, r.tagId, 0, 0, 0.036 * s);
  ctx.restore();
  if (L < 28) return;

  ctx.save();
  const top = Math.max(...model.legs.map(({ foot }) => foot.x * Math.sin(r.yaw) + foot.y * Math.cos(r.yaw)), 0.04);
  const label = r.tracking ? r.id : `${r.id} · no tracking`;
  const labelY = cy - top * s - 13;
  ctx.font = "500 10px system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  const width = ctx.measureText(label).width + 14;
  roundRect(ctx, cx - width / 2, labelY - 8, width, 16, 5);
  ctx.fillStyle = "rgba(255,255,255,0.9)";
  ctx.fill();
  ctx.fillStyle = r.tracking ? "#475569" : "#b45309";
  ctx.fillText(label, cx, labelY);
  ctx.restore();
}

/** The OLED face, facing +x so the robot always looks where it drives. */
function drawEyes(
  ctx: CanvasRenderingContext2D,
  bodyL: number,
  bodyW: number,
  tracking: boolean,
) {
  const r = Math.max(1, bodyW * 0.075);
  const ex = bodyL * 0.52;
  const ey = bodyW * 0.2;

  [-ey, ey].forEach((y) => {
    const cy = y - bodyW * 0.004 / 0.068;
    ctx.beginPath();
    ctx.ellipse(ex, cy, r * 0.55, r, 0, 0, Math.PI * 2);
    ctx.fillStyle = tracking ? ROBOT_FITTING : "#d5dde5";
    ctx.fill();
    ctx.strokeStyle = ROBOT_FITTING_INSET;
    ctx.lineWidth = 0.65;
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(ex, cy - r * 0.35);
    ctx.lineTo(ex, cy + r * 0.35);
    ctx.stroke();
  });
}

function drawSolids(ctx: CanvasRenderingContext2D, solids: Solid[], pose: Point & { yaw: number }, v: View) {
  const world = origin([pose.x, pose.y, 0], [0, 0, pose.yaw]);
  ctx.save();
  const outlines = solids.map((solid) => {
    const points = projectedOutline(solid);
    return { points, color: solid.color, height: Math.max(...points.map((p) => p.z)) };
  });
  outlines.sort((a, b) => a.height - b.height).forEach(({ points: outline, color }) => {
    const points = outline.map((p) => toPx(p.clone().applyMatrix4(world), v));
    polyPath(ctx, points);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.lineWidth = 0.65;
    ctx.strokeStyle = "rgba(0,0,0,0.35)";
    ctx.stroke();
  });
  ctx.restore();
}

function drawMarker(ctx: CanvasRenderingContext2D, id: number, x: number, y: number, size: number) {
  ctx.save();
  ctx.fillStyle = "white";
  ctx.fillRect(x - size * 0.625, y - size * 0.625, size * 1.25, size * 1.25);
  const image = markerImage(id);
  if (image) {
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(image, x - size / 2, y - size / 2, size, size);
  }
  ctx.restore();
}

function roundRect(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  w: number,
  h: number,
  r: number,
) {
  const radius = Math.max(0, Math.min(r, w / 2, h / 2));
  ctx.beginPath();
  ctx.moveTo(x + radius, y);
  ctx.arcTo(x + w, y, x + w, y + h, radius);
  ctx.arcTo(x + w, y + h, x, y + h, radius);
  ctx.arcTo(x, y + h, x, y, radius);
  ctx.arcTo(x, y, x + w, y, radius);
  ctx.closePath();
}

