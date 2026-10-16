# Swarm choreography and night-show lighting

Date: 2026-10-01. Status: user wants to pursue; concept recorded, implementation pending.

## Idea

Build a reusable QuadSim extension for simulated drone shows: fleets transition between shapes, follow synchronized trajectories, and display timed colors in a nighttime environment. Use the same foundation for later formation-control research.

Existing foundations include registered drone identities, individual controllers, batched per-agent commands with shared physics stepping, episode resets, and collision/state reporting. Fleet-scale performance has not been measured.

## Light component

- Give standard drone prefabs a named show-light mount and an optional reusable light assembly, disabled by default. Each model supplies its own mount position, orientation, and size; custom models may opt out.
- Use a small emissive mesh, such as an LED dome or ring. Its material supplies RGB color and emission intensity; a texture is optional for patterned surfaces. Enable HDRP bloom in the viewing camera's environment profile to produce the visible glow.
- Keep actual illumination separate: an optional, short-range Unity Light can illuminate nearby surfaces for close views. Avoid enabling a shadow-casting light on every drone as the default fleet representation.
- Expose enabled state, color, intensity, and timed cues through a future C#/Python API. Batch fleet changes and evaluate cues against simulation time, including pause, stepping, and reset. Preserve independent colors per drone.
- Start with a night environment preset using controlled exposure, restrained ambient light, and bloom. Validate distant LED visibility and avoid washing colors out. Headless runs can retain light state without rendering it.

## First demonstration

Use 16 drones for a grid-to-ring transition and landing, with synchronized color changes. Add bounded trajectories, assignment of drones to formation points, and minimum-separation checks throughout transitions. Record tracking error, closest separation, and simulation/render performance. Sixteen is a proposed starting size, not a capacity claim.

Start with Python choreography and a small Unity lighting component. Follow with trajectory upload/playback and larger-fleet benchmarks. Treat lightweight visual playback and full-physics validation as distinct modes.

## Open design choices

Choose mount conventions, color/intensity units, cue interpolation/reset behavior, batch API shape, and how the extension declares its capabilities in an entity definition. Compare emissive geometry with a distant-marker rendering mode before optimizing large fleets.

Reference: [HDRP bloom](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@17.0/manual/Post-Processing-Bloom.html).
