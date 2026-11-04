#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
/usr/bin/python3 -m unittest discover -s "$ROOT/src/hc_realman_driver/test" -v
