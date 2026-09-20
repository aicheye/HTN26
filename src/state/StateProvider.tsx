import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
  type ReactNode,
} from "react";
import {
  createSource,
  MockSource,
  DEFAULT_SOURCE,
  getWsUrl,
  setWsUrl as persistWsUrl,
  type ConnectionStatus,
  type SourceKind,
} from "../sources";
import { SesameHttpBridge } from "../robot/sesameApi";
import type { MockScenarioId } from "../data/mockScenarios";
import type { Ack, Command, CommandType, WorldState } from "../types/world";
import type { StateSource } from "../sources/StateSource";
import { useVoiceControl } from "./useVoiceControl";

const ACK_TIMEOUT_MS = 4000;

export type LogEntry = {
  command: Command;
  ack?: Ack;
};

type StateContextValue = ReturnType<typeof useVoiceControl> & {
  state: WorldState | null;
  status: ConnectionStatus;
  sourceKind: SourceKind;
  setSourceKind: (k: SourceKind) => void;
  wsUrl: string;
  setWsUrl: (url: string) => void;
  selectedRobotId: string | null;
  setSelectedRobotId: (id: string) => void;
  speed: number;
  setSpeed: (s: number) => void;
  log: LogEntry[];
  send: (type: CommandType, extra?: Partial<Command>) => void;
  resetMockScenario: (id: MockScenarioId) => void;
  runMockTest: () => void;
};

const StateContext = createContext<StateContextValue | null>(null);

const LOG_MAX = 40;

/** Set VITE_ROBOT_URL to also fire every command at the real firmware over HTTP. */
const robotBridge = import.meta.env.VITE_ROBOT_URL
  ? new SesameHttpBridge(import.meta.env.VITE_ROBOT_URL)
  : null;

let commandCounter = 0;
function nextCommandId() {
  commandCounter += 1;
  return `cmd-${Date.now().toString(36)}-${commandCounter}`;
}

export function StateProvider({ children }: { children: ReactNode }) {
  const [sourceKind, setSourceKindState] = useState<SourceKind>(DEFAULT_SOURCE);
  const [wsUrl, setWsUrlState] = useState<string>(getWsUrl());
  const [state, setState] = useState<WorldState | null>(null);
  const [status, setStatus] = useState<ConnectionStatus>("connecting");
  const [selectedRobotId, setSelectedRobotIdState] = useState<string | null>(null);
  const [speed, setSpeed] = useState(1);
  const [log, setLog] = useState<LogEntry[]>([]);
  const worldRef = useRef<WorldState | null>(null);
  const statusRef = useRef<ConnectionStatus>("connecting");
  const receivedRef = useRef(0);

  const source = useMemo(
    () => createSource(sourceKind, wsUrl),
    [sourceKind, wsUrl],
  );
  const sourceRef = useRef(source);
  sourceRef.current = source;
  const selectedRef = useRef<string | null>(null);
  selectedRef.current = selectedRobotId;
  const speedRef = useRef(speed);
  speedRef.current = speed;
  const ackedRef = useRef<Set<string>>(new Set());

  const dispatch = useCallback(
    (destination: StateSource, robotId: string | null, type: CommandType, extra?: Partial<Command>, voice = false) => {
      if (!robotId) return;
      const command: Command = {
        id: nextCommandId(),
        ts: Date.now(),
        robotId,
        type,
        speed: speedRef.current,
        ...extra,
      };
      // log first: mock acks arrive synchronously
      setLog((prev) => [{ command }, ...prev].slice(0, LOG_MAX));
      const accepted = destination.sendCommand(command, !voice || type === "stop");
      if (accepted !== false && (!voice || !(destination instanceof MockSource))) robotBridge?.send(command);

      window.setTimeout(() => {
        if (ackedRef.current.has(command.id)) return;
        setLog((prev) =>
          prev.map((e) =>
            e.command.id === command.id && !e.ack
              ? { ...e, ack: { commandId: command.id, ok: false, error: "timed out" } }
              : e,
          ),
        );
      }, ACK_TIMEOUT_MS);
      return accepted !== false;
    },
    [],
  );
  const voice = useVoiceControl(() => {
    const destination = sourceRef.current;
    const robotId = selectedRef.current;
    return {
      state: worldRef.current, robotId, status: statusRef.current, receivedAt: receivedRef.current,
      send: (type, extra) => dispatch(destination, robotId, type, extra, true),
    };
  }, wsUrl, sourceKind);
  const { cancelVoice } = voice;

  const setSourceKind = useCallback((kind: SourceKind) => {
    if (kind === sourceKind) return;
    cancelVoice();
    worldRef.current = null;
    setSourceKindState(kind);
  }, [sourceKind, cancelVoice]);
  const setWsUrl = useCallback((url: string) => {
    persistWsUrl(url);
    if (getWsUrl() === wsUrl) return;
    cancelVoice();
    worldRef.current = null;
    setWsUrlState(getWsUrl());
  }, [wsUrl, cancelVoice]);
  const setSelectedRobotId = useCallback((id: string) => {
    cancelVoice();
    selectedRef.current = id;
    setSelectedRobotIdState(id);
  }, [cancelVoice]);

  useEffect(() => {
    worldRef.current = null;
    receivedRef.current = 0;
    statusRef.current = "connecting";
    setState(null);
    setStatus("connecting");
    const unsubStatus = source.onStatus?.((next) => {
      statusRef.current = next;
      setStatus(next);
      if (next !== "live") {
        receivedRef.current = 0;
        cancelVoice();
      }
    });
    const unsubState = source.subscribe((s) => {
      worldRef.current = s;
      receivedRef.current = Date.now();
      setState(s);
      if (!s.robots.some((robot) => robot.id === selectedRef.current)) {
        cancelVoice();
        selectedRef.current = s.robots[0]?.id ?? null;
        setSelectedRobotIdState(selectedRef.current);
      }
    });
    const unsubAck = source.onAck?.((ack) => {
      ackedRef.current.add(ack.commandId);
      setLog((prev) =>
        prev.map((e) => (e.command.id === ack.commandId ? { ...e, ack } : e)),
      );
    });
    return () => {
      cancelVoice();
      unsubState();
      unsubAck?.();
      unsubStatus?.();
      source.stop?.();
    };
  }, [source, cancelVoice]);

  const send = useCallback((type: CommandType, extra?: Partial<Command>) => {
    cancelVoice();
    dispatch(sourceRef.current, selectedRef.current, type, extra);
  }, [cancelVoice, dispatch]);
  const resetMockScenario = useCallback((id: MockScenarioId) => {
    cancelVoice();
    if (sourceRef.current instanceof MockSource) sourceRef.current.resetScenario(id);
  }, [cancelVoice]);
  const runMockTest = useCallback(() => {
    cancelVoice();
    if (sourceRef.current instanceof MockSource) sourceRef.current.runScenarioTest();
  }, [cancelVoice]);

  const value: StateContextValue = {
    ...voice,
    state,
    status,
    sourceKind,
    setSourceKind,
    wsUrl,
    setWsUrl,
    selectedRobotId,
    setSelectedRobotId,
    speed,
    setSpeed,
    log,
    send,
    resetMockScenario,
    runMockTest,
  };

  return <StateContext.Provider value={value}>{children}</StateContext.Provider>;
}

export function useWorld() {
  const ctx = useContext(StateContext);
  if (!ctx) throw new Error("useWorld must be used inside <StateProvider>");
  return ctx;
}
