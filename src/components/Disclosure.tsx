import { useEffect, useRef, useState, type ReactNode } from "react";

export function Disclosure({
  title,
  defaultOpen = false,
  children,
}: {
  title: string;
  defaultOpen?: boolean;
  children: ReactNode;
}) {
  return (
    <details open={defaultOpen} className="group border-b border-zinc-800 last:border-b-0">
      <summary className="flex cursor-pointer list-none items-center justify-between py-3 text-sm font-medium text-zinc-300 marker:hidden hover:text-zinc-100">
        {title}
        <Chevron />
      </summary>
      <div className="pb-4">{children}</div>
    </details>
  );
}

export function Menu({
  label,
  children,
}: {
  label: string;
  children: ReactNode;
}) {
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!open) return;
    const onDown = (e: MouseEvent) => {
      if (!ref.current?.contains(e.target as Node)) setOpen(false);
    };
    const onKey = (e: KeyboardEvent) => e.key === "Escape" && setOpen(false);
    document.addEventListener("mousedown", onDown);
    document.addEventListener("keydown", onKey);
    return () => {
      document.removeEventListener("mousedown", onDown);
      document.removeEventListener("keydown", onKey);
    };
  }, [open]);

  return (
    <div ref={ref} className="relative">
      <button
        type="button"
        onClick={() => setOpen((v) => !v)}
        aria-expanded={open}
        className="flex items-center gap-1.5 rounded-md border border-zinc-700 bg-zinc-900 px-3 py-1.5 text-sm text-zinc-200 hover:bg-zinc-800"
      >
        {label}
        <Chevron />
      </button>
      {open && (
        <div className="absolute right-0 z-50 mt-1 w-64 rounded-lg border border-zinc-800 bg-zinc-900 p-3 shadow-xl">
          {children}
        </div>
      )}
    </div>
  );
}

export function Chevron() {
  return (
    <svg
      viewBox="0 0 12 12"
      className="h-3 w-3 text-zinc-500 transition-transform group-open:rotate-180"
      aria-hidden
    >
      <path
        d="M2.5 4.5 6 8l3.5-3.5"
        fill="none"
        stroke="currentColor"
        strokeWidth="1.5"
        strokeLinecap="round"
      />
    </svg>
  );
}
