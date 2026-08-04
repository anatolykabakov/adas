#!/usr/bin/env bash
# Run bag visualizer from project root: ./run_bag_vis.sh /path/to/bag
set -euo pipefail
# script lives in scripts/, project root is one level up
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPTS="$ROOT/app/src/main/scripts"
export PYTHONPATH="$SCRIPTS${PYTHONPATH:+:$PYTHONPATH}"
# Pure-Python protobuf: avoids TypeError with newer google.protobuf vs older _pb2.py
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
exec python3 "$SCRIPTS/vis/interactive_visualizer.py" "$@"
