#!/usr/bin/env bash

docker_image_name() {
  printf '%s\n' "${ADAPTER_IMAGE:-formant-spot-adapter:local}"
}

docker_container_name() {
  printf '%s\n' "${ADAPTER_CONTAINER_NAME:-formant-spot-adapter}"
}

host_service_name() {
  printf '%s\n' "${HOST_SERVICE_NAME:-formant-spot-adapter.service}"
}

require_docker_cli() {
  if command -v docker >/dev/null 2>&1; then
    return 0
  fi
  echo "Docker is not installed or not on PATH." >&2
  exit 1
}

init_docker_cmd() {
  if [[ "${DOCKER_CMD_INITIALIZED:-0}" == "1" ]]; then
    return 0
  fi

  require_docker_cli

  if docker info >/dev/null 2>&1; then
    DOCKER_CMD=(docker)
  else
    if ! command -v sudo >/dev/null 2>&1; then
      echo "Docker daemon access requires sudo, but sudo is not installed." >&2
      exit 1
    fi
    DOCKER_CMD=(sudo docker)
  fi

  DOCKER_CMD_INITIALIZED=1
}

docker_cmd() {
  init_docker_cmd
  "${DOCKER_CMD[@]}" "$@"
}
