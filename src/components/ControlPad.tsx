import { useEffect, useRef, useState } from "react";
import { useWorld } from "../state/StateProvider";
import { POSES, type CommandType } from "../types/world";

const KEY_MAP: Record<string, CommandType> = {
  w: "forward",
  arrowup: "forward",
  s: "backward",
  arrowdown: "backward",
  a: "left",
  arrowleft: "left",
  d: "right",
  arrowright: "right",
};

export function ControlPad() {
  const { send } = useWorld();
  const [active, setActive] = useState<CommandType | null>(null);
  const [posesOpen, setPosesOpen] = useState(false);
  const activeRef = useRef<CommandType | null>(null);

  const press = (type: CommandType) => {
    if (activeRef.current === type) return;
    activeRef.current = type;
    setActive(type);
    send(type);
  };

  const release = () => {
    if (!activeRef.current) return;
    activeRef.current = null;
    setActive(null);
    send("stop");
  };

  useEffect(() => {
    // arrow keys also drive sliders and selects, so ignore keys aimed at a control
    const typing = (t: EventTarget | null) =>
      t instanceof HTMLElement &&
      (t.tagName === "INPUT" ||
        t.tagName === "SELECT" ||
        t.tagName === "TEXTAREA" ||
        t.isContentEditable);

    const onKeyDown = (e: KeyboardEvent) => {
      if (typing(e.target)) return;
      const key = e.key.toLowerCase();
      if (key === " ") {
        e.preventDefault();
        activeRef.current = null;
        setActive(null);
        send("stop");
        return;
      }
      const type = KEY_MAP[key];
      if (!type || e.repeat) return;
      e.preventDefault();
      press(type);
    };
    const onKeyUp = (e: KeyboardEvent) => {
      const type = KEY_MAP[e.key.toLowerCase()];
      if (!type) return;
      if (activeRef.current === type) release();
    };
    const onVisibility = () => {
      if (document.hidden) release();
    };
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    window.addEventListener("blur", release);
    document.addEventListener("visibilitychange", onVisibility);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      window.removeEventListener("blur", release);
      document.removeEventListener("visibilitychange", onVisibility);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [send]);

  const key = (type: CommandType, rotation: number, cap: string) => (
    <button
      type="button"
      aria-label={type}
      onPointerDown={(e) => {
        e.currentTarget.setPointerCapture(e.pointerId);
        press(type);
      }}
      onPointerUp={release}
      onPointerCancel={release}
      onPointerLeave={release}
      className={`relative flex h-11 w-11 items-center justify-center rounded-md border transition-colors select-none ${
        active === type
          ? "border-zinc-100 bg-zinc-100 text-zinc-900"
          : "border-zinc-700/80 bg-zinc-800/60 text-zinc-300 hover:border-zinc-600 hover:bg-zinc-800"
      }`}
    >
      <ArrowIcon rotation={rotation} />
      <span className="absolute bottom-0.5 right-1 text-[9px] leading-none opacity-40">
        {cap}
      </span>
    </button>
  );

  return (
    <div className="space-y-3">
      <div className="flex flex-col items-center gap-1">
        {key("forward", 0, "W")}
        <div className="flex gap-1">
          {key("left", -90, "A")}
          {key("backward", 180, "S")}
          {key("right", 90, "D")}
        </div>
      </div>

      <div className="flex justify-center">
        <button
          type="button"
          onClick={() => {
            activeRef.current = null;
            setActive(null);
            send("stop");
          }}
          className="flex h-11 w-[140px] items-center justify-center gap-2 rounded-md border border-zinc-700/80 bg-zinc-800/60 text-sm text-zinc-300 transition-colors hover:border-zinc-600 hover:bg-zinc-800 active:bg-zinc-100 active:text-zinc-900"
        >
          Stop
          <kbd className="font-sans text-[9px] leading-none opacity-40">Space</kbd>
        </button>
      </div>

      <div className="space-y-1">
        <button
          type="button"
          aria-expanded={posesOpen}
          onClick={() => setPosesOpen((v) => !v)}
          className="flex h-9 w-full items-center justify-center gap-1.5 rounded-md border border-zinc-700/80 text-sm text-zinc-300 transition-colors hover:bg-zinc-800"
        >
          Poses
          <svg viewBox="0 0 12 12" className={`h-3 w-3 text-zinc-500 transition-transform ${posesOpen ? "rotate-180" : ""}`} aria-hidden>
            <path d="M2.5 4.5 6 8l3.5-3.5" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" />
          </svg>
        </button>
        {posesOpen && (
          <div className="grid grid-cols-3 gap-1">
            {POSES.map((pose) => (
              <button
                key={pose}
                type="button"
                onClick={() => send("pose", { pose })}
                className="truncate rounded-md border border-zinc-700/80 px-1.5 py-1.5 text-xs text-zinc-300 transition-colors hover:bg-zinc-800"
              >
                {pose}
              </button>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}

function ArrowIcon({ rotation }: { rotation: number }) {
  return (
    <svg
      viewBox="0 0 24 24"
      className="h-5 w-5"
      style={{ transform: `rotate(${rotation}deg)` }}
      aria-hidden
    >
      <path
        d="M12 5v14M12 5l-6 6M12 5l6 6"
        fill="none"
        stroke="currentColor"
        strokeWidth="3"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}
