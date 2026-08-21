# Changelog

## [0.3.0] - Unreleased

- Added protocol 1.0 negotiation with SDK, feature, frame, and time contracts.
- Bound RPC to localhost by default and added request size, shape, and rate limits.
- Added schema-v1 drone profiles and Unity protocol/YAML tests.
- Added a Drone YAML Inspector with rotor-layout and effectiveness previews.
- Added EditMode/PlayMode, Linux/Windows build, built-player smoke, log, and
  security CI definitions for the prepared release candidate.
- Added Phase 3/4 validation, compatibility, security, and authoring documentation.
- Updated runtime and bundled UImGui code for Unity 6000.5 compiler and
  serialization-analyzer compatibility.

## [0.2.3] - 2026-07-21

- Bundled the pinned UImGui 7.1.0 runtime and matching ImGui.NET binding directly with QuadSim.
- Bundled native ImGui libraries for Windows, Linux, Intel macOS, and Apple Silicon macOS.
- Marked the managed ImGui.NET assembly as `AnyCPU` so Apple Silicon Unity Editors load the correct binding.
- Removed the external UImGui/OpenUPM installation requirement and warned against separately installed duplicate bindings.
- Added an explicit sample assembly reference to `Nino.QuadSim` so `InternalApiExample` resolves the runtime API reliably in consumer projects.
- Documented the required HDRP project configuration and stale-sample cleanup procedure.

## [0.2.2] - 2026-07-21

- Fixed a global `Editor` namespace collision that prevented consumer projects containing common tutorial scripts from compiling.
- Restored the editable Starter sample hierarchy to `Configuration`, `Prefabs`, and `Prefabs/Quad Prefabs`.
- Renamed the editable scene configuration to `Default_SceneConfig` and retained its complete simulation, mode, spawning, payload, RPC, origin, noise, and wind settings.
- Documented the direct Git installation order for UImGui and QuadSim.

## [0.2.1] - 2026-07-21

- Promoted the installable QuadSim package to the repository root.
- Removed the Unity development host project and unrelated assets from the distribution branch.
- Simplified Git installation URLs so they no longer require a `?path=` parameter.

## [0.2.0] - 2026-07-21

- Converted QuadSim from a project-local `Assets/QuadSim` tree into an embedded Unity Package Manager package.
- Added the importable QuadSim Starter sample with Land, editable scene configuration, drone catalog, drone prefabs, example code, and YAML overrides.
- Preserved simulation internals, fallback configurations, drone models, and RPC dependencies as package-owned runtime content.
- Split editor-only tools into a dedicated editor assembly.
- Restored missing visualization materials and removed stale serialized asset references.
- Bundled the runtime DLL dependency set and removed the NuGetForUnity installation requirement.
