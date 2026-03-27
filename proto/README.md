# Proto Layout

This directory contains:

- `adapter_config.proto`: local adapter-specific configuration schema.
- `protos/`: vendored Formant upstream protobuf definitions used by this adapter.

The vendored Formant proto files under `protos/` are sourced from:

- repo: `https://github.com/FormantIO/formant.git`
- pinned commit: see [`formant_vendor.lock`](formant_vendor.lock)

Maintainers can use [`../scripts/sync_formant_protos.sh`](../scripts/sync_formant_protos.sh)
to refresh the vendored Formant files from a specific upstream ref.

Local compatibility note:

- `protos/model/v1/navigation.proto` removes two `proto3 optional` labels from
  the upstream file so the schema can still be compiled by Ubuntu 20.04's
  `libprotoc 3.6.1`.
