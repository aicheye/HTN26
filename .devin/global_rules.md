# Project constraints

- This user's scope is frontend work. Do not change the bridge, tracker, firmware, or detection pipeline unless explicitly asked.
- Frontend verification: `npm test` (Node 22.15+), `npm run build`.
- This workspace has Windows-native dependencies. From WSL, use `cmd.exe /c npm run build` and `cmd.exe /c npm run dev -- --host 0.0.0.0` rather than reinstalling dependencies to fix missing Linux native binaries.
- The workspace already had widespread CRLF-only git differences. Preserve unrelated edits and use `git diff --ignore-space-at-eol` for review.
- Scene units are metres, Z-up; robot-local +x is forward. URDF fixed-axis RPY uses Three.js Euler order ZYX.
- Referenced STL meshes are not present. Primitive robot models and unsupplied joint poses are visual approximations, not measured CAD surfaces or live telemetry.
