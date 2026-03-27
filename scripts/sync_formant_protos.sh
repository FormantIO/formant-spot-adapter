#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FORMANT_PROTO_REPO="${FORMANT_PROTO_REPO:-https://github.com/FormantIO/formant.git}"
FORMANT_PROTO_REF="${1:-${FORMANT_PROTO_REF:-c2df919bec01052454254a22d73a418b08b01d41}}"
VENDOR_ROOT="${ROOT_DIR}/proto/protos"
LOCK_FILE="${ROOT_DIR}/proto/formant_vendor.lock"

FORMANT_PROTO_FILES=(
  agent/v1/agent.proto
  model/v1/commands.proto
  model/v1/config.proto
  model/v1/datapoint.proto
  model/v1/event.proto
  model/v1/file.proto
  model/v1/health.proto
  model/v1/intervention.proto
  model/v1/math.proto
  model/v1/media.proto
  model/v1/navigation.proto
  model/v1/ros.proto
  model/v1/text.proto
  model/v1/views.proto
)

tmp_dir="$(mktemp -d "${TMPDIR:-/tmp}/formant-protos.XXXXXX")"
trap "rm -rf '${tmp_dir}'" EXIT

repo_dir="${tmp_dir}/formant"
git clone --depth 1 "${FORMANT_PROTO_REPO}" "${repo_dir}" >/dev/null
git -C "${repo_dir}" fetch --depth 1 origin "${FORMANT_PROTO_REF}" >/dev/null
git -C "${repo_dir}" checkout --detach FETCH_HEAD >/dev/null
resolved_commit="$(git -C "${repo_dir}" rev-parse HEAD)"

for rel_path in "${FORMANT_PROTO_FILES[@]}"; do
  src_path="${repo_dir}/protos/${rel_path}"
  dst_path="${VENDOR_ROOT}/${rel_path}"
  if [[ ! -f "${src_path}" ]]; then
    echo "[formant-protos] missing upstream proto: ${src_path}" >&2
    exit 1
  fi
  mkdir -p "$(dirname "${dst_path}")"
  cp "${src_path}" "${dst_path}"
done

# Ubuntu 20.04 ships protoc 3.6.1, which does not support proto3 optional.
sed -i \
  -e 's/^    optional double altitude = 3;/    double altitude = 3;/' \
  -e 's/^    optional double orientation = 4;/    double orientation = 4;/' \
  "${VENDOR_ROOT}/model/v1/navigation.proto"

cat > "${LOCK_FILE}" <<EOF
repo=${FORMANT_PROTO_REPO}
commit=${resolved_commit}
EOF

echo "[formant-protos] synced ${resolved_commit}"
