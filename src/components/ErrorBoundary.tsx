import { Component, type ReactNode } from "react";

type Props = { fallback: (retry: () => void) => ReactNode; children: ReactNode };
type State = { error: Error | null };

/** Catches render/WebGL crashes in a subtree (e.g. Map3D) instead of taking down the whole app. */
export class ErrorBoundary extends Component<Props, State> {
  state: State = { error: null };

  static getDerivedStateFromError(error: Error): State {
    return { error };
  }

  componentDidCatch(error: Error, info: { componentStack: string }) {
    console.error("ErrorBoundary caught:", error, info.componentStack);
  }

  private retry = () => this.setState({ error: null });

  render() {
    if (this.state.error) return this.props.fallback(this.retry);
    return this.props.children;
  }
}
