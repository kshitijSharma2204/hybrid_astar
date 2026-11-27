#!/usr/bin/env bash
# Creates .venv, installs requirements.txt and adds python/ to the venv's
# path, so "import hybrid_astar" works once the CMake build has run.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
python3 -m venv "$root/.venv"
"$root/.venv/bin/pip" install -q -r "$root/requirements.txt"
site="$("$root/.venv/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
echo "$root/python" > "$site/hybrid_astar.pth"
echo "venv ready: $root/.venv (python/ added via $site/hybrid_astar.pth)"
