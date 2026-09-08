# Config-driven drone assets

Drone configs may declare `prefab_key`, resolved by the `DronePrefabCatalog` resource at `Runtime/Resources/QuadSim/DronePrefabCatalog.asset`. The catalog holds serialized prefab references so model dependencies are included in standalone builds; runtime spawning needs no AssetDatabase or filesystem prefab loading.

The package and starter YAML configs map Standard, Flightmare, and Crazyflie to their corresponding existing pawn prefabs. Sentinel currently uses an explicitly documented Standard visual fallback because no Sentinel model is bundled. Its physics still comes from the Sentinel config.

To add a model, create a pawn prefab based on the existing base, assign its renderer/rotor references and FPV mount, add a unique catalog entry, and put that entry's key in the YAML. Assign a custom catalog on DroneManager to replace the package catalog. An unknown explicit key fails spawning with an actionable error. Older configs without a key resolve by source filename, then legacy `body_mesh`; otherwise the manager's legacy prefab is used with a warning.

All manager-based spawning uses the same resolver, including the HUD, config replacement and SDK paths that call `SpawnDroneWithConfig`. Model names are not used as identity: some historical YAMLs share the same display name.

FPV mounts are authored per prefab, forward along the drone's local +X, positioned just ahead of the visible model. The global display rig reads that mount directly. The training camera remains a separate camera under the mount, so its optical origin also follows the corrected mounting position. This intentionally changes FPV/sensor extrinsics; no sensor-image equivalence is claimed.

Validation: 70 Editor tests passed, including six-direction free movement at zero time scale, boost, input release, config resolution, and mount orientation. Three PlayMode tests passed, including mixed Flightmare/Crazyflie spawning through DroneManager and attached camera pose checks. Flightmare FPV was visually checked in the main Unity scene. Native automated key taps did not provide a reliable live movement check; sustained hardware movement remains a manual verification item.
