#!/bin/bash
set -euo pipefail

OS="$(uname -s)"
if [[ "$OS" == "Linux" ]]; then
    if ! dpkg -l libhidapi-dev libudev-dev >/dev/null 2>&1; then
        sudo apt-get update || true
        sudo apt-get install -y --no-install-recommends \
            -o DPkg::Lock::Timeout=60 \
            ca-certificates \
            libhidapi-dev \
            libhidapi-hidraw0 \
            libhidapi-libusb0 \
            libudev-dev
    fi
elif [[ "$OS" == "Darwin" ]]; then
    if ! brew list hidapi >/dev/null 2>&1; then
        brew install hidapi
    fi
fi
