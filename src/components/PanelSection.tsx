import type { ReactNode } from "react";

/** A labelled group inside a side panel; every panel uses this so headings and spacing stay consistent. */
export function PanelSection({ title, children }: { title: string; children: ReactNode }) {
  return (
    <section className="space-y-3">
      <h3 className="text-[11px] font-medium uppercase tracking-wider text-zinc-500">{title}</h3>
      {children}
    </section>
  );
}
