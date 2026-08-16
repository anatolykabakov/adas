#!/usr/bin/env bash
# Build and run ADAS in a container — no host install required.
#
#   ./scripts/docker.sh build            build image (C++ / python tools)
#   ./scripts/docker.sh build android    image with Android SDK/NDK for APK build
#   ./scripts/docker.sh shell            interactive shell in container
#   ./scripts/docker.sh host             build C++ host part and pyadas
#   ./scripts/docker.sh tests            build and run unit tests
#   ./scripts/docker.sh apk              build APK (needs android image)
#   ./scripts/docker.sh sim              controller run on track (sim.eval)
#   ./scripts/docker.sh run <command>    run arbitrary command inside
#
# Examples:
#   ./scripts/docker.sh run python3 scripts/bag/bag_controller_ab.py \
#                              adas_logs/<session> --controllers fp,mpc
#   ./scripts/docker.sh run ./scripts/run_bag_vis.sh adas_logs/<session>
#
# adb is not forwarded into the container: connect the phone on the host, pull bags with
# ./scripts/pull_bags.sh, and use the container for build and analysis.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT/.devcontainer/docker-compose.yml"
# Build uses the same build_cpp.sh as on the host — otherwise flags diverge (e.g. cached
# protoc path is reset there, and after a conan cache change the build breaks).
# Separate build dir: conan and cmake write absolute paths into cache; one dir for two
# environments does not work.
BUILD_ENV="ADAS_BUILD_DIR_SUFFIX=-docker"

export ADAS_UID="$(id -u)"
export ADAS_GID="$(id -g)"
export ADAS_TARGET="${ADAS_TARGET:-tools}"

compose() { docker compose -f "$COMPOSE_FILE" "$@"; }

# Named volumes for ~/.conan2 and ~/.gradle are root-owned when first created empty;
# Conan then cannot write global.conf. Fix ownership once before any build/run.
ensure_cache_perms() {
  compose run --rm -T -u root --entrypoint bash adas -lc \
    'chown -R dev:dev /home/dev/.conan2 /home/dev/.gradle 2>/dev/null || true' \
    >/dev/null
}

# One-shot container: leaves no junk and needs no running service.
run_in() {
  ensure_cache_perms
  compose run --rm -T adas bash -lc "$*"
}

cmd="${1:-help}"
shift || true

case "$cmd" in
  build)
    [ "${1:-}" = "android" ] && export ADAS_TARGET=android
    compose build
    echo "image adas-$ADAS_TARGET ready"
    ;;

  shell)
    ensure_cache_perms
    compose run --rm adas bash
    ;;

  host)
    run_in "$BUILD_ENV ./app/src/main/cpp/build_cpp.sh -t linux"
    echo "pyadas built → scripts/pyadas/"
    ;;

  tests)
    run_in "$BUILD_ENV ./app/src/main/cpp/build_cpp.sh -t linux --test"
    ;;

  apk)
    export ADAS_TARGET=android
    # Separate build tree and gradle cache: host and container have different absolute paths inside
    # cmake/conan caches, and the gradle daemon from .tools/gradle-home on the host will not run in the container.
    run_in "$BUILD_ENV GRADLE_USER_HOME=/home/dev/.gradle \
            GRADLE_OPTS=-Dorg.gradle.daemon=false ./scripts/build_project.sh ${*:-}"
    ;;

  sim)
    # pyadas must be built first: ./scripts/docker.sh host
    run_in "cd scripts && python3 -m sim.eval ${*:-}"
    ;;

  run)
    [ $# -gt 0 ] || { echo "command required: ./scripts/docker.sh run <command>" >&2; exit 2; }
    run_in "$*"
    ;;

  *)
    sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    ;;
esac
