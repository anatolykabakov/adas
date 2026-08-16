#!/usr/bin/env bash
# Interactive MetaDrive from project root: ./scripts/run_sim.sh --controller fp --lanes supercombo --show
# Controller run with metrics — python3 -m sim.eval (see docs/SIM_CONTROLLER_TEST.md)
set -euo pipefail
# script lives in scripts/, project root is one level up
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPTS="$ROOT/scripts"
export PYTHONPATH="$SCRIPTS${PYTHONPATH:+:$PYTHONPATH}"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
cd "$SCRIPTS"
exec python3 -m sim.main "$@"
