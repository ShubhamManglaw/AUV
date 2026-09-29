#!/usr/bin/env bash
# End-to-end full chain demo run script (§14.4, Q7).
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
cd "$REPO_DIR"

export DISPLAY="${DISPLAY:-:0}"
source tools/env.sh

# Run full-chain demo with timeout 600s
timeout 600 python3 tools/run_e2e_demo.py
