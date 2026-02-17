# Repository Guidelines

## Project Structure & Module Organization
- `src/`: C++ adapter runtime (`main.cpp`, teleop/Spot integration loops).
- `include/formant_spot_adapter/`: public headers for adapter, config, and clients.
- `proto/`: protobuf and gRPC service definitions compiled at build time into `build/generated/`.
- `config/`: runtime configuration templates (`*.json.example`, `*.env.example`).
- `scripts/`: operational scripts for bootstrap, build, run, install, and systemd setup.
- `systemd/`: service unit template.
- `docs/`: integration notes and stream mapping docs.
- `third_party/`: local Spot SDK checkout and vendored dependencies.

## Build, Test, and Development Commands
- `./scripts/bootstrap_ubuntu.sh`: install apt dependencies and fetch Spot SDK.
- `./scripts/build.sh`: configure with CMake/Ninja and build `formant-spot-adapter`.
- `./scripts/run.sh`: run locally with env/config validation and log capture.
- `sudo ./scripts/install.sh`: install binary and default config files.
- `sudo ./scripts/setup_service.sh && sudo systemctl enable --now formant-spot-adapter.service`: register and start service.
- Optional direct build: `cmake -S . -B build -G Ninja -DSPOT_CPP_SDK_DIR=/path/to/spot-cpp-sdk && cmake --build build --target formant-spot-adapter`.

## Coding Style & Naming Conventions
- Language: C++17 and Bash.
- Indentation: 2 spaces; braces on same line for functions/control blocks.
- Naming: `snake_case` for functions/variables, `PascalCase` for classes, `kConstantStyle` for enum constants.
- Keep headers in `include/` aligned with implementations in `src/`.
- Prefer focused, single-purpose scripts in `scripts/` with `set -euo pipefail`.

## Testing Guidelines
- No formal automated test suite is currently checked in (`ctest`/`gtest` not configured).
- Before opening a PR, validate by:
  1. Successful clean build via `./scripts/build.sh`.
  2. Runtime smoke test via `./scripts/run.sh` using example config/env.
  3. Verifying expected log output in `logs/` and critical control paths (teleop heartbeat, lease acquire/return).
- If adding tests, place C++ tests under a new `tests/` directory and wire them into CMake with `add_test`.

## Commit & Pull Request Guidelines
- Commit history is minimal; use clear, imperative commit subjects (example: `Add dock status publish retry`).
- Keep commits scoped to one change area (config, runtime logic, build scripts).
- PRs should include:
  - concise problem/solution summary,
  - linked issue or task,
  - validation steps run (build + smoke test),
  - config or service-impact notes,
  - relevant logs/screenshots for behavior changes.

## Security & Configuration Tips
- Never commit secrets; keep credentials in `config/formant-spot-adapter.env`.
- Treat `config/formant-spot-adapter.json` as non-secret runtime config and document any new keys in `README.md`.
