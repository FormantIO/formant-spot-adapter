# Contributing

This repository is a focused adapter for Spot + Formant Agent teleoperation on
Ubuntu-based Jetson deployments. Contributions are welcome, but the bar here is
practicality rather than process.

## Scope

- Prefer targeted fixes, documentation improvements, and behavior changes that
  are clearly motivated by real use on Spot.
- Avoid unrelated refactors, broad cleanup passes, or architectural rewrites
  unless they are discussed first.
- Keep the repository Jetson-focused. Changes that add complexity for other
  environments should have a strong reason.

## Before You Start

- For anything non-trivial, open an issue or start a discussion before writing
  a large patch.
- If a change affects robot behavior, commands, stream names, deployment, or
  configuration, update the README and example config as part of the same
  change.
- Never commit secrets, local config, logs, downloaded SDK contents, or robot
  data.

## Development Notes

Typical local setup:

```bash
./scripts/bootstrap_ubuntu.sh
./scripts/build.sh
```

Useful lightweight checks:

```bash
bash -n scripts/*.sh
python3 -m py_compile scripts/*.py
```

If you are not working on a Jetson or do not have access to a Spot robot, that
is fine, but say so clearly in your change notes. Do not claim hardware
verification you did not perform.

## Pull Requests

When opening a PR, include:

- what changed
- why it changed
- what you verified manually
- what you did not verify

For behavior changes on hardware-facing code, include any relevant environment
details you have, such as Jetson model, Ubuntu version, and Spot SDK version.

## License

By submitting a contribution, you agree that it will be licensed under the
repository license.
