import type { WorldState } from "../types/world";
import marker0 from "../../pi/markers/marker-0.png";
import marker1 from "../../pi/markers/marker-1.png";
import marker2 from "../../pi/markers/marker-2.png";
import marker3 from "../../pi/markers/marker-3.png";
import marker4 from "../../pi/markers/marker-4.png";

export const markerUrls: Record<number, string> = { 0: marker0, 1: marker1, 2: marker2, 3: marker3, 4: marker4 };
const markers = new Map<number, HTMLImageElement>();
let wood: HTMLCanvasElement | undefined;

export function markerImage(id: number, redraw?: () => void) {
  if (!markerUrls[id]) return undefined;
  let image = markers.get(id);
  if (!image) {
    image = new Image();
    image.src = markerUrls[id];
    markers.set(id, image);
  }
  if (!image.complete && redraw) image.addEventListener("load", redraw, { once: true });
  return image.complete && image.naturalWidth ? image : undefined;
}

export function tableBorder(arena: WorldState["arena"]) {
  return arena.border ?? (arena.tagSize ?? 0.08) * 0.75;
}

export function cornerTags(arena: WorldState["arena"]) {
  return (arena.cornerTagIds ?? []).slice(0, 4).map((id, i) => ({
    id, x: i === 1 || i === 2 ? arena.width : 0, y: i >= 2 ? arena.length : 0,
  }));
}

export function woodCanvas() {
  if (wood) return wood;
  wood = document.createElement("canvas");
  wood.width = 768;
  wood.height = 768;
  const ctx = wood.getContext("2d")!;
  ctx.fillStyle = "#c9ac77";
  ctx.fillRect(0, 0, 768, 768);
  for (let i = 0; i < 520; i++) {
    const y = i * 1.6;
    ctx.strokeStyle = `rgba(${i % 3 ? "111,76,36" : "255,233,178"},${0.035 + (Math.sin(i * 7.13) + 1) * 0.025})`;
    ctx.lineWidth = 0.5 + (i % 4) * 0.3;
    ctx.beginPath();
    for (let x = 0; x <= 768; x += 8) {
      const grain = y + Math.sin(x / 160 + i * 0.035) * 13 + Math.sin(x / 63 + i * 0.07) * 3;
      if (x === 0) ctx.moveTo(x, grain); else ctx.lineTo(x, grain);
    }
    ctx.stroke();
  }
  return wood;
}
