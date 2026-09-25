/// <reference types="vite/client" />

interface ImportMetaEnv {
  readonly VITE_SOURCE?: string;
  readonly VITE_WS_URL?: string;
  readonly VITE_ROBOT_URL?: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
