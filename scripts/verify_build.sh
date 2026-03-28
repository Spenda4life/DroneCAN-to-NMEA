#!/usr/bin/env bash
set -euo pipefail

echo "=== Building production firmware ==="
pio run -e esp32doit-devkit-v1

echo "=== Building and running native tests ==="
pio test -e native

echo "=== Building embedded test environment ==="
pio run -e test_embedded

echo "=== All builds passed ==="
