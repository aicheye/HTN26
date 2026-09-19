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
  DEFAULT_SOURCE,
  getWsUrl,
  setWsUrl as persistWsUrl,
  type ConnectionStatus,
  type SourceKind,
} from "../sources";
import { SesameHttpBridge } from "../robot/sesameApi";
import type { Ack, Command, CommandType, WorldState } from "../types/world";

const ACK_TIMEOUT_MS = 4000;

export type LogEntry = {
  command: Command;
  ack?: Ack;
};

type StateContextValue = {
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
  const [sourceKind, setSourceKind] = useState<SourceKind>(DEFAULT_SOURCE);
  const [wsUrl, setWsUrlState] = useState<string>(getWsUrl());
  const [state, setState] = useState<WorldState | null>(null);
  const [status, setStatus] = useState<ConnectionStatus>("connecting");
  const [selectedRobotId, setSelectedRobotId] = useState<string | null>(null);
  const [speed, setSpeed] = useState(1);
  const [log, setLog] = useState<LogEntry[]>([]);

  const setWsUrl = useCallback((url: string) => {
    persistWsUrl(url);
    setWsUrlState(getWsUrl());
  }, []);

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

  useEffect(() => {
    setState(null);
    const unsubState = source.subscribe((s) => {
      setState(s);
      if (!selectedRef.current && s.robots.length) {
        selectedRef.current = s.robots[0].id;
        setSelectedRobotId(s.robots[0].id);
      }
    });
    const unsubAck = source.onAck?.((ack) => {
      ackedRef.current.add(ack.commandId);
      setLog((prev) =>
        prev.map((e) => (e.command.id === ack.commandId ? { ...e, ack } : e)),
      );
    });
    const unsubStatus = source.onStatus?.(setStatus);
    return () => {
      unsubState();
      unsubAck?.();
      unsubStatus?.();
      source.stop?.();
    };
  }, [source]);

  const send = useCallback(
    (type: CommandType, extra?: Partial<Command>) => {
      const robotId = selectedRef.current;
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
      sourceRef.current.sendCommand(command);
      robotBridge?.send(command);

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
    },
    [],
  );
  const value: StateContextValue = {
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
  };

  return <StateContext.Provider value={value}>{children}</StateContext.Provider>;
}

export function useWorld() {
  const ctx = useContext(StateContext);
  if (!ctx) throw new Error("useWorld must be used inside <StateProvider>");
  return ctx;
}
