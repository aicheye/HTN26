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
  /** The two maps are one view, as in Google Maps. 3D calls this when the camera is tilted to within a few degrees
   *  of straight down, and the app shows the 2D map. 2D calls it when the map is dragged, which is the tilt
   *  gesture there (a click picks a goal), and the app shows 3D, starting from above and tilting in. */
  onSwapView?: () => void;
  /** 3D only: start looking almost straight down and glide to the usual angle, because the view was just 2D. */
  enterFromTop?: boolean;
  /** Pixels on the left that the side panel covers. The 2D map fits the arena into what is left of the width, so
   *  an open panel does not hide part of it. The canvas keeps its size. */
  insetLeft?: number;
};
