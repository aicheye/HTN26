import { useEffect, useRef, useState } from "react";
import { Menu } from "./Disclosure";
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
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    window.addEventListener("blur", release);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      window.removeEventListener("blur", release);
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
      className={`relative flex h-10 w-10 items-center justify-center rounded-md border transition select-none ${
        active === type
          ? "border-zinc-100 bg-zinc-100 text-zinc-900 shadow-inner"
          : "border-zinc-700 bg-zinc-800 text-zinc-200 shadow-[0_1px_0_rgba(255,255,255,0.04),0_2px_4px_rgba(0,0,0,0.3)] hover:bg-zinc-700"
      }`}
    >
      <ArrowIcon rotation={rotation} />
      <span className="absolute bottom-0.5 right-1 text-[9px] font-medium opacity-50">
        {cap}
      </span>
    </button>
  );

  return (
    <div className="flex flex-col items-center gap-2">
      <div className="flex flex-col items-center gap-1.5">
        {key("forward", 0, "W")}
        <div className="flex gap-1.5">
          {key("left", -90, "A")}
          {key("backward", 180, "S")}
          {key("right", 90, "D")}
        </div>
      </div>

      <div className="flex items-center gap-2">
        <button
          type="button"
          onClick={() => {
            activeRef.current = null;
            setActive(null);
            send("stop");
          }}
          className="h-8 rounded-md bg-red-600 px-4 text-sm font-semibold text-white shadow-[0_2px_6px_rgba(220,38,38,0.35)] transition hover:bg-red-700"
        >
          Stop
          <span className="ml-1.5 text-[10px] font-normal opacity-70">Space</span>
        </button>

        <Menu label="Poses">
          <div className="flex flex-wrap gap-1">
            {POSES.map((pose) => (
              <button
                key={pose}
                type="button"
                onClick={() => send("pose", { pose })}
                className="rounded-md border border-zinc-700 bg-zinc-800 px-2 py-1 text-xs text-zinc-300 hover:bg-zinc-700"
              >
                {pose}
              </button>
            ))}
          </div>
        </Menu>
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
