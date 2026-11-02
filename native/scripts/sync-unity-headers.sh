#!/usr/bin/env bash
# Usage: sync-unity-headers.sh [--check|--update] [EDITOR_DIR]
set -euo pipefail

MODE="--check"
if [[ $# -gt 0 && ( "$1" == "--check" || "$1" == "--update" ) ]]; then
	MODE="$1"
	shift
fi

EDITOR_DIR="${1:-$HOME/Unity/Hub/Editor/6000.6.7f1/Editor}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$SCRIPT_DIR/../third_party/unity"
SRC="$EDITOR_DIR/Data/PluginAPI"

if [[ ! -d "$SRC/PhysicsBackend" ]]; then
	echo "PluginAPI not found: $SRC" >&2
	exit 2
fi

TOP_HEADERS=(IUnityInterface.h IUnityLog.h IUnityPhysicsBackendSystem.h)

case "$MODE" in
	--check)
		status=0
		for h in "${TOP_HEADERS[@]}"; do
			if ! diff -q "$SRC/$h" "$DEST/$h" >/dev/null 2>&1; then
				echo "differs: $h"
				status=1
			fi
		done
		for f in "$SRC"/PhysicsBackend/*.h; do
			name="$(basename "$f")"
			if ! diff -q "$f" "$DEST/PhysicsBackend/$name" >/dev/null 2>&1; then
				echo "differs: PhysicsBackend/$name"
				status=1
			fi
		done
		for f in "$DEST"/PhysicsBackend/*.h; do
			name="$(basename "$f")"
			if [[ ! -f "$SRC/PhysicsBackend/$name" ]]; then
				echo "extra in repo: PhysicsBackend/$name"
				status=1
			fi
		done
		if [[ $status -eq 0 ]]; then
			echo "Unity headers are in sync with $EDITOR_DIR"
		fi
		exit $status
		;;
	--update)
		mkdir -p "$DEST/PhysicsBackend"
		for h in "${TOP_HEADERS[@]}"; do
			cp "$SRC/$h" "$DEST/$h"
		done
		rm -f "$DEST"/PhysicsBackend/*.h
		cp "$SRC"/PhysicsBackend/*.h "$DEST/PhysicsBackend/"
		echo "Updated headers from $EDITOR_DIR"
		;;
esac
