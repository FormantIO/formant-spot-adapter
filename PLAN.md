# PLAN

This file must contain only remaining (open) issues.
When an issue is finished, remove it from this file and archive the completed work in `IMPLEMENTATION_HISTORY.md`.

## Open Issues

### P2 - Capability and Contract Consistency

- [ ] Resolve camera auto-rotation mismatch (docs/config vs implementation)
  - Problem: Docs/README describe hand-camera auto-rotation, but runtime currently forwards JPEG bytes without rotation logic.
  - Risk: Operator expectation mismatch and incorrect UI assumptions.
  - Initial fix direction:
    - Either implement the advertised rotation path or remove related claims/config flags.
  - References: `README.md`, `docs/formant-streams.md`, `src/adapter.cpp`, `include/formant_spot_adapter/config.hpp`

- [ ] Resolve unused arm-raise capability surface
  - Problem: `arm_raise_*` config and button stream are defined but not wired into adapter command handling.
  - Risk: Dead config, confusing controls, incomplete feature surface.
  - Initial fix direction:
    - Implement arm-raise behavior and button handling, or remove deprecated fields/docs.
  - References: `include/formant_spot_adapter/config.hpp`, `src/config.cpp`, `src/adapter.cpp`

- [ ] Reconcile SpotClient calibration API declarations with implementation
  - Problem: `CancelCameraCalibration` and `GetCameraCalibrationFeedback` are declared but not implemented/used.
  - Risk: API drift and future linker/maintenance issues.
  - Initial fix direction:
    - Implement the methods and expose command/status wiring, or remove declarations until needed.
  - References: `include/formant_spot_adapter/spot_client.hpp`, `src/spot_client.cpp`

### P3 - Validation and Operational Readiness

- [ ] Add repeatable production smoke/regression checks
  - Problem: No formal automated suite; operational regressions may slip through.
  - Risk: Late discovery of failures in reconnect, lease handling, throttling, and nav/dock workflows.
  - Initial fix direction:
    - Create scripted smoke checks covering disconnect/reconnect, lease contention recovery, throttling scenarios, and key command flows.
    - Document expected pass/fail signals from logs and status streams.
  - References: `scripts/`, `README.md`
