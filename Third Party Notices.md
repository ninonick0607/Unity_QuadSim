# Third-Party Notices

QuadSim redistributes the following runtime libraries:

- AsyncIO 0.1.69 — MIT — https://github.com/somdoron/AsyncIO
- MessagePack and MessagePack.Annotations 3.1.4 — MIT — https://github.com/MessagePack-CSharp/MessagePack-CSharp
- Microsoft.NET.StringTools 17.11.4 — MIT — https://github.com/dotnet/msbuild
- NaCl.Net 0.1.13 — MPL-2.0 — https://github.com/somdoron/NaCl.net
- NetMQ 4.0.2.2 — MIT — https://github.com/zeromq/netmq
  - Includes QuadSim's backport of the NetMQ 4.0.4.3 macOS ARM64 poller timeout fix, which prevents `Socket.Select` from blocking shutdown indefinitely.
- System.Collections.Immutable 8.0.0 — MIT — https://github.com/dotnet/runtime
- System.ServiceModel.Primitives 8.1.2 — MIT — https://github.com/dotnet/wcf
- UImGui 7.1.0 — MIT — https://github.com/psydack/uimgui
- ImGui.NET — MIT — https://github.com/psydack/ImGui.NET.4Unity
- Dear ImGui — MIT — https://github.com/ocornut/imgui
- cimgui — MIT — https://github.com/cimgui/cimgui
- YamlDotNet 18.1.0 — MIT — https://github.com/aaubry/YamlDotNet

The corresponding licenses and source are available at the linked upstream repositories. Redistributed license texts and vendor notes are included under `Runtime/ThirdParty/UImGui/`.
