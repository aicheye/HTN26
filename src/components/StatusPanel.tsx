import { useEffect, useRef, useState } from "react";
import { useWorld } from "../state/StateProvider";
import type { WorldState } from "../types/world";
import { costmap as planCostmap } from "../../bridge/planner.mjs";

function staleMs(lastSeen: number): number {
  return Math.max(0, Date.now() - lastSeen);
}

export function Telemetry() {
  const { state, selectedRobotId, setSelectedRobotId } = useWorld();
  const robot = state?.robots.find((r) => r.id === selectedRobotId) ?? state?.robots[0];

  if (!robot) return <p className="text-xs text-zinc-500">Waiting for first frame</p>;

  return (
    <div className="space-y-2">
      {state && state.robots.length > 1 && (
        <select
          value={selectedRobotId ?? ""}
          onChange={(e) => setSelectedRobotId(e.target.value)}
          className="rounded border border-zinc-700 bg-zinc-800 px-1 py-0.5 text-xs text-zinc-200"
        >
          {state.robots.map((r) => (
            <option key={r.id} value={r.id}>
              {r.id}
            </option>
          ))}
        </select>
      )}
      {!robot.tracking && (
        <p className="rounded bg-amber-500/10 px-2 py-1 text-xs text-amber-300">
          {robot.mode === "lost" ? "Lost — " : ""}Tag not detected — showing last known pose
          {staleMs(robot.lastSeen) > 1000 ? ` (${(staleMs(robot.lastSeen) / 1000).toFixed(1)}s ago)` : ""}.
        </p>
      )}
      <dl className="grid grid-cols-[auto_1fr] gap-x-3 gap-y-1 text-xs">
        <Row label="Robot" value={robot.id} />
        <Row label="Tag" value={`#${robot.tagId}`} />
        <Row label="Mode" value={robot.mode} muted={robot.mode === "lost"} />
        <Row label="X" value={`${robot.x.toFixed(3)} m`} />
        <Row label="Y" value={`${robot.y.toFixed(3)} m`} />
        <Row label="Yaw" value={`${((robot.yaw * 180) / Math.PI).toFixed(1)}°`} />
        <Row
          label="Tracking"
          value={robot.tracking ? "Yes" : "No"}
          muted={!robot.tracking}
        />
        <Row label="Last seen" value={`${(staleMs(robot.lastSeen) / 1000).toFixed(1)}s ago`} />
        <Row
          label="Conf"
          value={robot.confidence != null ? robot.confidence.toFixed(2) : "—"}
        />
        <Row label="Frame" value={state ? `#${state.seq}` : "—"} />
        <Row label="Face" value={robot.face ?? "—"} />
        <Row label="Pose" value={robot.pose ?? "—"} />
        {state?.calibration && (
          <Row
            label="Calib"
            value={
              state.calibration.ok
                ? `ok · ${state.calibration.reprojectionError?.toFixed(2) ?? "—"} px`
                : "failed"
            }
          />
        )}
      </dl>
      {state && <Navigation state={state} />}
      {state && <Objects state={state} robot={robot} />}
      {state && <Costmap state={state} />}
    </div>
  );
}

function Heading({ children }: { children: string }) {
  return <h3 className="pt-2 text-[10px] font-semibold uppercase tracking-wider text-zinc-500">{children}</h3>;
}

/** What the bridge's navigator is doing, and the numbers the planner works with. Live source only. */
function Navigation({ state }: { state: WorldState }) {
  const m = state.mission;
  if (!m) return null;
  const path = state.path ?? [];
  let remaining = 0;
  for (let i = 1; i < path.length; i++) remaining += Math.hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  const cm = (v?: number) => (v != null ? `${(v * 100).toFixed(1)} cm` : "—");
  return (
    <>
      <Heading>Navigation</Heading>
      {m.detail && <p className="rounded bg-zinc-800 px-2 py-1 text-xs text-zinc-300">{m.detail}</p>}
      <dl className="grid grid-cols-[auto_1fr] gap-x-3 gap-y-1 text-xs">
        <Row label="Mission" value={m.state} muted={m.state === "failed" || m.state === "carrying"} />
        <Row label="Command" value={m.command || "—"} />
        <Row label="Goal" value={state.goal ? `${state.goal.x.toFixed(2)}, ${state.goal.y.toFixed(2)} m` : "—"} />
        <Row label="Waypoints" value={`${m.waypoints ?? 0}`} />
        <Row label="Route left" value={path.length > 1 ? cm(remaining) : "—"} />
        <Row label="Recoveries" value={`${m.recoveries ?? 0}`} muted={(m.recoveries ?? 0) > 0} />
        <Row label="Carries" value={`${m.carries ?? 0}`} />
        <Row label="Edge stops" value={`${m.edgeStops ?? 0}`} muted={(m.edgeStops ?? 0) > 0} />
        <Row label="Robot reach" value={cm(m.robotRadius)} />
        <Row label="Edge limit" value={cm(state.arena.edgeMargin)} />
        <Row label="Drive" value={m.drive ?? "—"} />
        {/* The mock has no robot link and no camera, so these two rows are for the live bridge only. */}
        {m.trackerFps != null && <Row label="Robot link" value={m.robotConnected ? "connected" : "down"} muted={!m.robotConnected} />}
        {m.trackerFps != null && <Row label="Camera" value={`${m.trackerFps} fps · ${m.floorMarkers ?? 0} floor tags`} muted={(m.floorMarkers ?? 0) < 3} />}
      </dl>
    </>
  );
}

/** Every obstacle the planner avoids, nearest first. The arm's own detection is listed too: the maps hide it, the
 *  planner does not. */
function Objects({ state, robot }: { state: WorldState; robot: WorldState["robots"][number] }) {
  const all = [...state.obstacles, ...(state.hiddenObstacles ?? [])]
    .map((o) => ({ o, d: Math.hypot(o.x - robot.x, o.y - robot.y) }))
    .sort((a, b) => a.d - b.d);
  if (all.length === 0) return null;
  return (
    <>
      <Heading>{`Objects (${all.length})`}</Heading>
      <ul className="space-y-0.5 text-xs">
        {all.map(({ o, d }) => (
          <li key={o.id} className="flex items-center gap-2 text-zinc-300">
            <span className="h-2.5 w-2.5 shrink-0 rounded-sm border border-zinc-600" style={{ background: o.color ?? "#475569" }} />
            <span className="truncate">{o.label ?? o.id}</span>
            <span className="ml-auto shrink-0 font-mono tabular-nums text-zinc-500">
              {o.radius != null ? `r ${(o.radius * 100).toFixed(0)}` : `${((o.width ?? 0) * 100).toFixed(0)}×${((o.length ?? 0) * 100).toFixed(0)}`} cm · {(d * 100).toFixed(0)} cm
            </span>
          </li>
        ))}
      </ul>
    </>
  );
}

type CostmapData = { cell: number; cols: number; rows: number; clearance: number; edgeMargin: number; cost: number[] };

/** The planner's grid, refreshed once a second: grey where the robot's centre may not
 *  be, free cells from pale (cost 1) to orange (cost 4, right at a limit), with the path, the robot and the goal. */
function Costmap({ state }: { state: WorldState }) {
  const { wsUrl, sourceKind } = useWorld();
  const [map, setMap] = useState<CostmapData | null>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);

  // Live: the bridge's own grid, from GET /costmap. Mock: the same planner runs here in the browser, so the grid is
  // computed from the mock's scene. Both once a second.
  const stateRef = useRef(state);
  stateRef.current = state;
  useEffect(() => {
    const url = wsUrl.replace(/^ws/, "http").replace(/\/ws\/?$/, "") + "/costmap";
    let stopped = false;
    const load = () => {
      if (sourceKind !== "ws") {
        const now = stateRef.current;
        setMap(planCostmap(now.arena, now.obstacles, now.mission?.robotRadius));
        return;
      }
      fetch(url).then((r) => r.json()).then((data) => { if (!stopped && data.cost) setMap(data); }).catch(() => {});
    };
    load();
    const timer = setInterval(load, 1000);
    return () => { stopped = true; clearInterval(timer); };
  }, [wsUrl, sourceKind]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !map) return;
    const px = Math.max(3, Math.floor(232 / map.cols));
    canvas.width = map.cols * px;
    canvas.height = map.rows * px;
    const ctx = canvas.getContext("2d")!;
    for (let i = 0; i < map.cost.length; i++) {
      const cost = map.cost[i], cx = i % map.cols, cy = map.rows - 1 - Math.floor(i / map.cols);   // row 0 is y = 0, drawn at the bottom
      const t = Math.max(0, Math.min(1, (cost - 1) / 3));
      ctx.fillStyle = cost < 0 ? "#3f3f46" : `rgb(${Math.round(244 - t * 5)}, ${Math.round(240 - t * 125)}, ${Math.round(228 - t * 206)})`;
      ctx.fillRect(cx * px, cy * px, px, px);
    }
    const at = (p: { x: number; y: number }): [number, number] => [(p.x / map.cell) * px, canvas.height - (p.y / map.cell) * px];
    const path = state.path ?? [];
    if (path.length > 1) {
      ctx.strokeStyle = "#0f172a";
      ctx.lineWidth = 2;
      ctx.beginPath();
      path.forEach((p, i) => (i === 0 ? ctx.moveTo(...at(p)) : ctx.lineTo(...at(p))));
      ctx.stroke();
    }
    for (const drop of state.mission?.carry?.drops ?? []) { ctx.fillStyle = "#7c3aed"; ctx.beginPath(); ctx.arc(...at(drop), 3, 0, Math.PI * 2); ctx.fill(); }
    if (state.goal) { ctx.fillStyle = "#dc2626"; ctx.beginPath(); ctx.arc(...at(state.goal), 4, 0, Math.PI * 2); ctx.fill(); }
    for (const r of state.robots) {
      const [x, y] = at(r);
      ctx.fillStyle = r.tracking ? "#2563eb" : "#a1a1aa";
      ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = ctx.fillStyle; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + Math.cos(r.yaw) * 10, y - Math.sin(r.yaw) * 10); ctx.stroke();
    }
  }, [map, state]);

  if (!map) return null;
  const free = map.cost.filter((c) => c >= 0).length;
  return (
    <>
      <Heading>Costmap</Heading>
      <canvas ref={canvasRef} className="w-full rounded border border-zinc-700" style={{ imageRendering: "pixelated" }} />
      <p className="text-[11px] leading-snug text-zinc-500">
        {map.cols}×{map.rows} cells of {(map.cell * 100).toFixed(0)} cm · {Math.round((100 * free) / map.cost.length)}% walkable · grey: closed to the
        robot's centre (objects + {(map.clearance * 100).toFixed(1)} cm, edge {(map.edgeMargin * 100).toFixed(0)} cm) · orange: near a limit
      </p>
    </>
  );
}

export function CommandLog() {
  const { log } = useWorld();

  if (log.length === 0) {
    return <p className="text-xs text-zinc-500">No commands yet</p>;
  }

  return (
    <ul className="space-y-1 pr-1 text-xs">
      {log.map((e) => (
        <li key={e.command.id} className="flex items-center gap-2 text-zinc-400">
          <span
            className={
              e.ack ? (e.ack.ok ? "text-emerald-400" : "text-red-400") : "text-zinc-600"
            }
          >
            {e.ack ? (e.ack.ok ? "✓" : "✕") : "•"}
          </span>
          <span className="font-medium text-zinc-100">{e.command.type}</span>
          {(e.command.pose || e.command.face) && (
            <span>{e.command.pose ?? e.command.face}</span>
          )}
          {e.command.target && (
            <span>
              {e.command.target.x.toFixed(2)}, {e.command.target.y.toFixed(2)}
            </span>
          )}
          <span className="ml-auto tabular-nums text-zinc-500">
            {new Date(e.command.ts).toLocaleTimeString([], {
              hour: "2-digit",
              minute: "2-digit",
              second: "2-digit",
            })}
          </span>
        </li>
      ))}
    </ul>
  );
}

function Row({
  label,
  value,
  muted,
}: {
  label: string;
  value: string;
  muted?: boolean;
}) {
  return (
    <>
      <dt className="text-zinc-500">{label}</dt>
      <dd
        className={`text-right font-mono tabular-nums ${
          muted ? "text-amber-400" : "text-zinc-100"
        }`}
      >
        {value}
      </dd>
    </>
  );
}
