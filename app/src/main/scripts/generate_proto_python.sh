#!/bin/bash
set -e
cd "$(dirname "$0")"
PROTO_DIR="../proto"
OUTPUT_DIR="./proto"
mkdir -p "$OUTPUT_DIR"

# protoc below 3.19 emits stubs the modern runtime refuses to load with its C parser, and the
# scripts fall back to pure-Python protobuf: decoding one bag then takes minutes instead of
# seconds. Set PROTOC to point at a newer compiler if the system one is old.
PROTOC="${PROTOC:-protoc}"
if ! command -v "$PROTOC" &> /dev/null; then
  echo "protoc not found (set PROTOC=/path/to/protoc)"
  exit 1
fi

version="$("$PROTOC" --version | awk '{print $2}')"
major="${version%%.*}"
minor="${version#*.}"; minor="${minor%%.*}"
if [ "$major" -lt 3 ] || { [ "$major" -eq 3 ] && [ "$minor" -lt 19 ]; }; then
  echo "protoc $version is too old: need >= 3.19 (set PROTOC=/path/to/protoc)"
  exit 1
fi

"$PROTOC" --proto_path="$PROTO_DIR" --python_out="$OUTPUT_DIR" "$PROTO_DIR"/*.proto
# Flat imports for scripts (from proto import messages_pb2 / bag_io via scripts/proto)
echo "Generated with protoc $version:"
ls -1 "$OUTPUT_DIR"/*_pb2.py
