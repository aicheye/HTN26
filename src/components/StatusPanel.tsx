import { useWorld } from "../state/StateProvider";

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
    </div>
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
