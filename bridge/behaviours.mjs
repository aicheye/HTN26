// What Spidey does that nobody asked for, on top of navigation. Pure logic: whatever hosts it (bridge.mjs for the
// real robot, the web UI's Mock source) passes in how to walk, and calls step() on every tick.
//
//   curious  when something new appears on the table and Spidey is free, it walks over to look at it
//   diary    a short log of what happened on the table: what appeared, moved or went, and what Spidey did
//
// The robot has no display and the diary is the only voice it has, so its entries name the two robots: Spidey is
// the quadruped, Armie the arm. All units metres and milliseconds, frontend frame.
import { distanceToObstacle } from "./planner.mjs";

const MOVED_M = 0.05;           // an object has moved when its centre shifts by more than this
const SETTLE_MS = 1500;         // an object must be seen this long before Spidey believes in it
const GONE_MS = 4000;           // and be missing this long before it is gone: a scan can lose one for a moment
const LOOK_MS = 2500;           // how long Spidey looks at an object it has walked to
const DIARY_LENGTH = 30;
const ACTIVE = ["navigating", "recovering", "carrying"];

export class Behaviours {
  // hooks: goto({x, y}). ignore(obstacle) is true for obstacles that are not things on the table, such as Armie.
  constructor(hooks, options = {}) {
    this.hooks = hooks;
    this.options = { curious: false, diary: true, ...options };
    this.known = new Map();      // id -> { x, y, label, since, seenAt, announced }
    this.diary = [];
    this.lastMission = "idle";
    this.errand = null;          // { id, label, at, started, lookUntil } while Spidey is off to look at something
  }

  status() {
    return { ...this.options, errand: this.errand ? { label: this.errand.label, looking: Boolean(this.errand.lookUntil) } : null, diary: this.diary };
  }

  configure(options) {
    Object.assign(this.options, options);
  }

  note(text, now, kind = "table") {
    if (!this.options.diary) return;
    this.diary.unshift({ at: now, text, kind });
    this.diary.length = Math.min(this.diary.length, DIARY_LENGTH);
  }

  // The user took over (any manual command): whatever Spidey was up to by itself is dropped.
  interrupt() {
    this.errand = null;
  }

  step({ robot, obstacles, mission }, now = Date.now()) {
    const objects = obstacles.filter((o) => o.source === "cv" && !(this.hooks.ignore?.(o)));
    const label = (o) => o.label ?? o.id;
    const fresh = this.watchTable(objects, now, label);
    const state = mission?.state ?? "idle", busy = ACTIVE.includes(state);

    // What just happened to Spidey.
    if (state !== this.lastMission) {
      if (state === "recovering") this.note("Spidey got stuck and is backing off.", now, "robot");
      if (state === "carrying") this.note("No way through. Spidey asks Armie for a lift.", now, "robot");
      if (state === "navigating" && this.lastMission === "carrying") this.note("Armie set Spidey down. Walking on.", now, "robot");
      if (state === "failed" && !this.errand) this.note(`Spidey gave up: ${mission.detail || "no way there"}.`, now, "robot");
      if (state === "done" && !this.errand) this.note("Spidey arrived.", now, "robot");
    }
    if (mission?.via && !this.viaNoted) this.note("No way through. Spidey walks over to where Armie can reach.", now, "robot");
    this.viaNoted = Boolean(mission?.via);

    // An errand of its own: walking to an object, then looking at it.
    if (this.errand) {
      const errand = this.errand;
      // The mission still reads "done" or "failed" from before until the host's navigator has picked the goto up.
      // An end only counts once the walk has been seen to start, or after 1.5 s for a goal it was already at.
      if (busy) errand.started = true;
      const ended = errand.started || now - errand.at > 1500;
      if (errand.lookUntil) {
        if (now >= errand.lookUntil) this.errand = null;
      } else if (!ended) {
        // wait for the walk to start
      } else if (state === "done") {
        errand.lookUntil = now + LOOK_MS;
        this.note(`Spidey had a good look at the ${errand.label}.`, now, "robot");
      } else if (state === "failed") {
        this.note(`Spidey could not get to the ${errand.label}.`, now, "robot");
        this.errand = null;
      }
    }
    this.lastMission = state;
    if (this.errand || busy || !robot?.tracking) return;

    if (this.options.curious && fresh) {
      this.note(`What is that? Spidey goes to look at the ${label(fresh)}.`, now, "robot");
      this.errand = { id: fresh.id, label: label(fresh), at: now, started: false };
      this.hooks.goto({ x: fresh.x, y: fresh.y });
    }
  }

  // Keeps track of what is on the table. Returns an object that has newly appeared and settled, if any.
  watchTable(objects, now, label) {
    let fresh = null;
    for (const o of objects) {
      const known = this.known.get(o.id);
      if (!known) { this.known.set(o.id, { x: o.x, y: o.y, label: label(o), since: now, seenAt: now, announced: false }); continue; }
      known.seenAt = now;
      if (!known.announced && now - known.since >= SETTLE_MS) {
        known.announced = true;
        // Whatever is there on the first look was there before: only later arrivals are news.
        if (this.started) {
          this.note(`A ${known.label} appeared.`, now);
          fresh = o;
        }
      } else if (known.announced && Math.hypot(o.x - known.x, o.y - known.y) > MOVED_M) {
        this.note(`The ${known.label} moved ${Math.round(100 * Math.hypot(o.x - known.x, o.y - known.y))} cm.`, now);
        Object.assign(known, { x: o.x, y: o.y });
      }
    }
    for (const [id, known] of this.known) {
      if (now - known.seenAt < GONE_MS) continue;
      if (known.announced && this.started) this.note(`The ${known.label} is gone.`, now);
      this.known.delete(id);
    }
    if (!this.started && (objects.length === 0 || [...this.known.values()].every((k) => k.announced))) this.started = true;
    return fresh;
  }
}

// True for a camera detection that is really Armie: its footprint reaches the arm's base.
export function isArmDetection(obstacle, armBase, radius = 0.1) {
  return Boolean(armBase) && distanceToObstacle(armBase, obstacle) < radius;
}
