import { useEffect, useRef, useState } from "react";
import { makeMockScenario } from "../data/mockScenarios";
import type { WorldState } from "../types/world";
import { BlurredBackdrop } from "./BlurredBackdrop";

/** The robot, arm and a few detected objects on the table, with no simulation overlay. */
const SHOWCASE: WorldState = (() => {
  const { goal: _g, path: _p, simulation: _s, ...scene } = makeMockScenario("mixed");
  return scene;
})();

const FADE_MS = 500;

export function TitleScreen({ onStart }: { onStart: () => void }) {
  const [leaving, setLeaving] = useState(false);
  const [gone, setGone] = useState(false);
  const button = useRef<HTMLButtonElement>(null);

  useEffect(() => button.current?.focus(), []);
  useEffect(() => {
    if (!leaving) return;
    const timer = window.setTimeout(() => setGone(true), FADE_MS);
    return () => window.clearTimeout(timer);
  }, [leaving]);
  const start = () => {
    if (leaving) return;
    setLeaving(true);
    onStart();
  };
  const startRef = useRef(start);
  startRef.current = start;
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => { if (e.key === "Enter" && !e.repeat) startRef.current(); };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);
  if (gone) return null;

  return (
    <div
      role="dialog"
      aria-label="Sesame Controller"
      style={{ transitionDuration: `${FADE_MS}ms` }}
      className={`fixed inset-0 z-50 overflow-hidden transition-opacity ${leaving ? "pointer-events-none opacity-0" : "opacity-100"}`}
    >
      <BlurredBackdrop state={SHOWCASE} orbit blur="blur-[2px]" tint="from-zinc-950/20 via-zinc-950/35 to-zinc-950/75" />
      <div className="absolute inset-0 flex flex-col items-center justify-center px-8 text-center">
        <h1
          style={{ fontFamily: '"Unbounded", ui-sans-serif, system-ui, sans-serif' }}
          className="text-4xl font-bold tracking-[0.08em] text-zinc-50 sm:text-6xl"
        >
          PLACEHOLDER
        </h1>
        <p className="mt-6 max-w-md text-sm leading-relaxed text-zinc-200 sm:text-base">
          A robot that finds its own way around the table. Tell it where to go and it plans the route,
          steering around obstacles and calling in the arm when it needs a lift.
        </p>
        <button
          ref={button}
          type="button"
          onClick={start}
          className="group mt-12 inline-flex items-center gap-4 py-2 text-xs font-medium uppercase tracking-[0.3em] text-zinc-100 focus:outline-none focus-visible:underline focus-visible:underline-offset-8"
        >
          Begin
          <span aria-hidden className="relative block h-px w-12 bg-zinc-400 transition-all duration-300 group-hover:w-20 group-hover:bg-zinc-100">
            <span className="absolute -right-px -top-[3px] h-[7px] w-[7px] rotate-45 border-r border-t border-current" />
          </span>
        </button>
      </div>
    </div>
  );
}
