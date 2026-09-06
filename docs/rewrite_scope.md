# Rewrite Scope (v1 Foundation)

## Keep
- Public package entrypoint `testbed`.
- Dual backend concept: virtual simulation and real hardware.
- Control module (`testbed/control`) APIs.
- Utility modules for geometry and MATLAB trajectory loading.

## Replace / Refactor
- Core base abstractions and shared validation flow.
- Virtual simulator implementation for a stable minimal backend.
- Hardware boundary interfaces (vision + serial wiring).
- Broken or legacy example scripts.

## Target Module Layout
- `testbed/core`: shared base classes and robot model.
- `testbed/simulators`: virtual backend and visualization.
- `testbed/hardware`: camera tracking and communication.
- `testbed/control`: controllers, safety barriers, decentralized controller orchestration.
- `testbed/utils`: geometry and data I/O helpers.
- `examples`: executable usage samples.
- `tests`: smoke tests for import and basic loop behavior.

## Migration Strategy
1. Stabilize public API and virtual backend.
2. Isolate hardware dependencies behind explicit interfaces.
3. Add decentralized control orchestration for multi-robot scaling.
4. Keep incremental compatibility in examples and docs.
5. Validate each phase with smoke tests before expanding scope.
