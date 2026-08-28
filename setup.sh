#!/usr/bin/env bash
# Dev-machine environment setup: install pixi if missing, then create the
# locked Python environment defined by pixi.toml / pixi.lock.
# The robot needs none of this — it runs on its preinstalled robot_env.
set -euo pipefail

if ! command -v pixi >/dev/null 2>&1 && [[ ! -x "$HOME/.pixi/bin/pixi" ]]; then
  echo "setup: installing pixi..."
  curl -fsSL https://pixi.sh/install.sh | bash
fi
PIXI="$(command -v pixi || echo "$HOME/.pixi/bin/pixi")"

cd "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")"
"$PIXI" install
echo "setup: done. Run tests with: $PIXI run test"
