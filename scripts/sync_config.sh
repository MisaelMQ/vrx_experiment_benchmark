#!/usr/bin/env bash
# sync_config.sh — Sycs YAMLs y Python from source at install.
# Use: ./scripts/sync_config.sh
set -e

SRC="/home/misael/ros2_workspaces/diplomado_ws/src/vrx_experiment_benchmark"
DST="/home/misael/ros2_workspaces/diplomado_ws/install/vrx_experiment_benchmark/share/vrx_experiment_benchmark"
PY_DST="/home/misael/ros2_workspaces/diplomado_ws/install/vrx_experiment_benchmark/lib/python3.12/site-packages/vrx_experiment_benchmark"

echo "[sync_config] Copiando config/*.yaml ..."
for f in "$SRC/config/"*.yaml; do
    fname=$(basename "$f")
    cp "$f" "$DST/config/$fname" && echo "  OK: $fname"
done

echo "[sync_config] Copiando config/routes/*.yaml ..."
for f in "$SRC/config/routes/"*.yaml; do
    fname=$(basename "$f")
    cp "$f" "$DST/config/routes/$fname" && echo "  OK: routes/$fname"
done

echo "[sync_config] Copiando *.py al install ..."
for f in "$SRC/vrx_experiment_benchmark/"*.py; do
    fname=$(basename "$f")
    cp "$f" "$PY_DST/$fname" && echo "  OK: $fname"
done

echo "[sync_config] Done."
