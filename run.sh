#!/usr/bin/env bash
# DEUX CHEVAUX — build (if node is available) and serve on 0.0.0.0:${PORT:-9070}
cd "$(dirname "$0")"
PORT=${PORT:-9070}
if command -v node >/dev/null 2>&1 && [ -z "$NO_BUILD" ]; then node build.mjs || echo "build failed — serving the last dist/"; fi
echo "DEUX CHEVAUX on http://0.0.0.0:$PORT/  (readable build: /dev.html)"
exec python3 -m http.server "$PORT" --bind 0.0.0.0 --directory dist
