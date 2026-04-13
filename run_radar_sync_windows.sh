#!/usr/bin/env bash
set -euo pipefail

RADAR_BIN="/home/lhlj/Radar/out/bin/radar"
SRC_DIR="/home/lhlj/Radar/data"
DST_DIR="/mnt/d/Project/RaderEchoSimulation/回波模拟仿真/过程文件/data"

echo "[1/3] Running radar binary..."
"$RADAR_BIN"

echo "[2/3] Ensuring destination directory exists..."
mkdir -p "$DST_DIR"

if [ ! -d "$SRC_DIR" ]; then
  echo "ERROR: Source directory not found: $SRC_DIR"
  exit 1
fi

echo "[3/3] Syncing output to Windows path (overwrite old files)..."
rsync -av --delete "$SRC_DIR"/ "$DST_DIR"/

echo "Done. Synced: $SRC_DIR -> $DST_DIR"
