# Runtime UI handoff

The current architecture, controls, startup policy and build guidance are maintained in [runtime-workspace.md](runtime-workspace.md). Historical rollout notes and generated review exports have been removed.

Preserve these boundaries when extending the HUD:
- SimulationManager owns time; presentation uses wall time.
- DroneManager and ScenarioObjectManager own lifecycle; views call services.
- Validate authority and selection identity before applying commands.
- Keep camera target, scene selection and controlled drone independent.
- Keep sensor cameras untouched and highlighting in the existing custom pass.
- Headless runs create no workspace. Release startup uses the workspace; ImGui is developer-only.
- Verify changes in the host Unity project, including a player build for startup or build-pipeline changes.
