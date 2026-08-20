# Phase 1/2 Validation Checkpoint

Status: passed on the Unity 6 development build and local QuadSimLib source.

- Phase 1 analytic rotor, wrench, allocation, saturation, rank, and conditioning tests: 21/21 passed.
- Phase 2 SDK lockstep, multi-agent, replay, and reset-verifier tests: 35/35 passed.
- Live 50 Hz lockstep completed 20 ticks as 5 physics steps per tick with exact target stepping and pause restoration.
- Two-drone seeded runs repeatedly matched within `1e-6`, remained unchanged when action order was reversed, changed sensor output with a different seed, emitted collision events, and cleared collision state on reset.

This is the internal analytic and runtime baseline. The final validation will compare conventional Quad-X plant/allocation results against ProjectAirSim using the same geometry, commands, timing, and reported wrench/state fields.
