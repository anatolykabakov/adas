#!/bin/bash
set -e
cd "$(dirname "$0")"
PROTO_DIR="../proto"
OUTPUT_DIR="./proto"
mkdir -p "$OUTPUT_DIR"

if ! command -v protoc &> /dev/null; then
  echo "protoc not found"
  exit 1
fi

protoc --proto_path="$PROTO_DIR" --python_out="$OUTPUT_DIR" "$PROTO_DIR"/*.proto
# Flat imports for scripts (from proto import messages_pb2 / bag_io via scripts/proto)
echo "Generated:"
ls -1 "$OUTPUT_DIR"/*_pb2.py
