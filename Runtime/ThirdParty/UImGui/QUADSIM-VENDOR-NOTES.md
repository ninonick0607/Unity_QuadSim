# QuadSim UImGui vendor notes

QuadSim vendors the runtime subset of UImGui 7.1.0 from upstream commit
`44315262a2261428280deeea4d8d42736bbc8e7c`.

Included content:

- UImGui runtime and editor integration source
- Runtime resources and shaders
- Matching ImGui.NET managed binding
- cimgui native libraries for Windows x86/x64/ARM64, Linux x64, and universal macOS x64/ARM64

Local packaging adjustments:

- `ImGui.NET.dll` is imported as `AnyCPU` in the Unity Editor so Apple Silicon uses the matching managed binding.
- UImGui's bundled `System.Runtime.CompilerServices.Unsafe.dll` is omitted because Unity 6 already supplies a newer compatible assembly.
- Optional ImPlot, ImNodes, ImGuizmo, ImGuizmoQuat, and cimCTE integrations are omitted because QuadSim does not use them.

Do not install a second UImGui or ImGui.NET package alongside QuadSim. Doing so can introduce duplicate assemblies or mismatched generated bindings.
