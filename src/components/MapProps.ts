import type { Point, WorldState } from "../types/world";

/** Both Map2D and the future Map3D take exactly this, so the swap is a one-line change. */
export type MapProps = {
  state: WorldState;
  selectedRobotId?: string | null;
  showCameraLayer?: boolean;
  /** Thumbnail mode: no overlay controls, no chrome, not interactive. */
  compact?: boolean;
  /** With compact: slowly circle the board at a 30 degree elevation instead of holding a fixed angle. */
  orbit?: boolean;
  onPickGoal?: (p: Point) => void;
};
