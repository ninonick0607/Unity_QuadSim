# Repository Guidelines

## Project Structure & Module Organization

This directory contains the Unity C# runtime scripts for `Unity_QuadSim`.
Simulation timing, stepping, and RPC integration live in `SimCore/`.
Drone vehicle behavior, controllers, APIs, interfaces, and debug helpers live in `DroneCore/`.
Robot abstractions and sensors live in `RobotCore/`, shared math in `MathUtil/`, UI controllers in `UI/`, and drone YAML parsing in `Yaml/Drone/`.
`Examples/` contains sample integration code. Treat `Experimental/` as prototype code, not stable API surface.

Keep Unity `.meta` files with their matching scripts and assets.

## Build, Test, and Development Commands

Run builds and play-mode checks from the Unity Editor for the full project, not from this `Assets/Scripts` folder alone.

- Open the project root in Unity, then use Play Mode to validate simulation behavior.
- Use Unity's Build Settings for player builds.
- Use Unity Test Runner for Edit Mode or Play Mode tests.

There is no local `make`, `npm`, or standalone CLI build script in this directory.

## Coding Style & Naming Conventions

Use C# with Unity conventions: four-space indentation, braces on their own lines, and namespaces matching modules such as `SimCore`, `DroneCore`, or `RobotCore`.

Use `PascalCase` for public types, properties, methods, and enum members. Use `_camelCase` for private runtime fields and `camelCase` for serialized inspector fields, following existing files such as `SimulationManager.cs` and `QuadPawn.cs`.

Prefer `[SerializeField] private` fields for Inspector wiring. Keep Unity lifecycle methods focused and delegate larger behavior to named private methods.

## Testing Guidelines

No test files are currently present under this scripts tree. Add tests through Unity Test Framework using `EditMode` or `PlayMode` folders under `Assets/Tests/` or another project-approved test location.

Name test files after the class or behavior under test, for example `SimulationManagerTests.cs`. Prefer deterministic tests for fixed-step timing, controller math, parsing, and sensor data transformations.

## Commit & Pull Request Guidelines

Recent commit history uses short, imperative subjects, but it is inconsistent. Use clearer subjects going forward, for example `Fix simulation step clamping` or `Add drone config validation`.

Pull requests should include a concise description, affected modules, Unity validation performed, and any scene or prefab setup changes. Link related issues when available, and include screenshots or recordings for UI or visible simulation changes.

## Agent-Specific Instructions

Keep changes narrowly scoped to the requested module. Avoid editing generated Unity metadata except when adding, moving, or deleting the corresponding asset through Unity-aware workflows.
