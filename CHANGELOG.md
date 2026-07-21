# Changelog

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
