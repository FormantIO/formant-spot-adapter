# Security Policy

## Supported Scope

Security fixes are expected to land on the current default branch. Older
snapshots may not receive updates.

## Reporting

For non-sensitive bugs, use the public issue tracker.

For security-sensitive issues, do not open a public issue. Report them to:

- `security@formant.io`

Please include:

- repository name
- commit SHA or version in use
- Jetson model and Ubuntu version, if relevant
- Spot SDK version, if relevant
- a short impact summary
- reproduction steps or a proof of concept

Do not include long-lived credentials in the report. If credentials are needed
to explain the issue, redact them.

## Deployment Notes

- This adapter expects the Formant agent connection to stay on local host
  networking. Do not expose that endpoint on an untrusted network without
  adding the transport protections you need.
- Keep `config/formant-spot-adapter.env` out of version control and limit file
  permissions appropriately.
- Review commands that can reboot the host or robot before enabling them in a
  shared environment.
