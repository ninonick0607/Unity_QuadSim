# QuadSim Documentation

This folder contains package-level documentation for the Unity QuadSim plugin.

## Documents

[Runtime workspace](runtime-workspace.md): Default HUD, controls, code map, developer tools and build guidance.

[Platform and extension ideas](platform-extension-ideas.md): Future direction for reusable extensions, standalone/embedded use, and swarm choreography.

[Swarm choreography and night-show lighting](swarm-show-idea.md): Fleet choreography, scriptable drone lights, and a proposed first demonstration.

[Engineering backlog](engineering-backlog.md): Verified bugs, architecture work, fidelity gaps, paper evidence, and pending decisions.

[RPC and ZeroMQ study track](learning-rpc.md): Staged exercises for learning QuadSim's Python-to-Unity link.

AI working rules for all three repositories live in the repository-root `CLAUDE.md`.

```text
px4-sitl.md          PX4 SITL bridge workflow and architecture
networking.md        Ports, topology, WSL/firewall notes, multi-instance setup
troubleshooting.md   Common failures and isolation steps
headless-tools.md    Generic external visualization/headless tooling
```

## Related folders

```text
Samples~/QuadSim Starter/     Importable scene and editable user content
Samples~/PX4 SITL Tools/      PX4-specific Python helper scripts
headless-tools/               Generic standalone Python tools
Runtime/                      Unity runtime package content
Editor/                       Unity editor tools
```
