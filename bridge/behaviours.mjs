// Small behaviours on top of navigation, so the robot does things nobody asked for. Pure logic: whatever hosts it
// (bridge.mjs for the real robot, the web UI's Mock source) passes in how to walk, show a face and play a pose, and
// calls step() on every tick. Each feature has its own switch, so one can be cut without touching the others.
//
//   moods    the OLED face and a pose follow what happens: excited when it sets off, happy and a wave when it
//            arrives, a shrug when there is no way, angry when stuck, surprised when the arm lifts it or when
//            something lands next to it, sleepy after a while with nothing to do
//   curious  when something new appears on the table and the robot is free, it walks over to look at it
//   tour     visits every object once, nearest first, looks at each for a moment, then takes a bow
//   diary    a short log of what happened on the table: what appeared, moved or went, and what the robot did
//
// All units metres and milliseconds, frontend frame.
import { distanceToObstacle } from "./planner.mjs";

const NEAR_M = 0.25;            // something that lands this close makes the robot jump
const MOVED_M = 0.05;           // an object has moved when its centre shifts by more than this
const SETTLE_MS = 1500;         // an object must be seen this long before the robot believes in it
const GONE_MS = 4000;           // and be missing this long before it is gone: a scan can lose one for a moment
const LOOK_MS = 2500;           // how long it looks at an object it has walked to
const SLEEPY_MS = 45000;
const DIARY_LENGTH = 30;
const ACTIVE = ["navigating", "recovering", "carrying"];

export class Behaviours {
  // hooks: goto({x, y}), cancel(), face(name), pose(name). ignore(obstacle) is true for obstacles that are not
  // things on the table, such as the arm.
  constructor(hooks, options = {}) {
    this.hooks = hooks;
    this.options = { moods: true, curious: false, diary: true, ...options };
    this.tour = null;            // { queue: [ids], visited: n, total: n } while a tour runs
    this.known = new Map();      // id -> { x, y, label, since, seenAt, announced }
    this.diary = [];
    this.lastMission = "idle";
    this.lastBusyAt = 0;
    this.errand = null;          // { id, label, kind: "curious" | "tour", lookUntil }
    this.sleepy = false;
  }

  status() {
    return { ...this.options, tour: this.tour ? { visited: this.tour.visited, total: this.tour.total } : null,
      errand: this.errand ? { label: this.errand.label, kind: this.errand.kind } : null, diary: this.diary };
  }

  configure(options) {
    Object.assign(this.options, options);
  }

  note(text, now, kind = "table") {
    if (!this.options.diary) return;
    this.diary.unshift({ at: now, text, kind });
    this.diary.length = Math.min(this.diary.length, DIARY_LENGTH);
  }

  mood(face, pose) {
    if (!this.options.moods) return;
    if (face) this.hooks.face(face);
    if (pose) this.hooks.pose(pose);
  }

  startTour(objects, robot, now) {
    const queue = objects.map((o) => ({ id: o.id, d: Math.hypot(o.x - robot.x, o.y - robot.y) })).sort((a, b) => a.d - b.d).map((o) => o.id);
    if (queue.length === 0) { this.note("Nothing on the table to visit.", now, "robot"); return false; }
    this.tour = { queue, visited: 0, total: queue.length };
    this.errand = null;
    this.note(`Setting off on a tour of ${queue.length} thing${queue.length === 1 ? "" : "s"}.`, now, "robot");
    return true;
  }

  // The user took over (any manual command): whatever the robot was up to by itself is dropped.
  interrupt() {
    this.tour = null;
    this.errand = null;
  }

  step({ robot, obstacles, mission }, now = Date.now()) {
    const objects = obstacles.filter((o) => o.source === "cv" && !(this.hooks.ignore?.(o)));
    const label = (o) => o.label ?? o.id;
    const fresh = this.watchTable(objects, robot, now, label);
    const state = mission?.state ?? "idle", busy = ACTIVE.includes(state);

    // Moods: what just happened to the robot.
    if (state !== this.lastMission) {
      if (state === "navigating" && !ACTIVE.includes(this.lastMission)) this.mood("excited");
      if (state === "recovering") { this.mood("angry"); this.note("Got stuck, backing off.", now, "robot"); }
      if (state === "carrying") { this.mood("surprised"); this.note("No way through: asking the arm for a lift.", now, "robot"); }
      if (state === "navigating" && this.lastMission === "carrying") { this.mood("love"); this.note("The arm set it down, walking on.", now, "robot"); }
      if (state === "failed" && !this.errand) { this.mood("sad", "shrug"); this.note(`Gave up: ${mission.detail || "no way there"}.`, now, "robot"); }
      if (state === "done" && !this.errand) { this.mood("happy", "wave"); this.note("Arrived.", now, "robot"); }
    }
    if (!this.lastBusyAt) this.lastBusyAt = now;  // the first step: it has not been idle for 45 s, it has just started
    if (busy || this.errand) { this.lastBusyAt = now; this.sleepy = false; }
    else if (!this.sleepy && now - this.lastBusyAt > SLEEPY_MS) { this.sleepy = true; this.mood("sleepy"); this.note("Nothing to do, dozing off.", now, "robot"); }

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
        this.mood(errand.kind === "curious" ? "love" : "thinking");
        this.note(errand.kind === "curious" ? `Had a look at the ${errand.label}.` : `Stop ${this.tour?.visited ?? "?"} of ${this.tour?.total ?? "?"}: the ${errand.label}.`, now, "robot");
      } else if (state === "failed") {
        this.note(`Could not get to the ${errand.label}.`, now, "robot");
        this.errand = null;
      }
    }
    this.lastMission = state;
    if (this.errand || busy || !robot?.tracking) return;

    const setOff = (object, kind) => {
      this.errand = { id: object.id, label: label(object), kind, at: now, started: false };
      this.hooks.goto({ x: object.x, y: object.y });
    };
    if (this.tour) {
      let next;
      while (this.tour.queue.length && !(next = objects.find((o) => o.id === this.tour.queue[0]))) this.tour.queue.shift();  // taken away meanwhile
      if (!next) {
        this.note("Tour finished.", now, "robot");
        this.mood("happy", "bow");
        this.tour = null;
        return;
      }
      this.tour.queue.shift();
      this.tour.visited++;
      return setOff(next, "tour");
    }
    if (this.options.curious && fresh) {
      this.mood("surprised");
      this.note(`What is that? Going to look at the ${label(fresh)}.`, now, "robot");
      setOff(fresh, "curious");
    }
  }

  // Keeps track of what is on the table. Returns an object that has newly appeared and settled, if any.
  watchTable(objects, robot, now, label) {
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
          if (robot && Math.hypot(o.x - robot.x, o.y - robot.y) < NEAR_M && !this.options.curious) this.mood("surprised");
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

// True for a camera detection that is really the arm: its footprint reaches the arm's base.
export function isArmDetection(obstacle, armBase, radius = 0.1) {
  return Boolean(armBase) && distanceToObstacle(armBase, obstacle) < radius;
}
