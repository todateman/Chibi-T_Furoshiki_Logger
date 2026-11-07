#!/bin/sh
# push の前に走らせるタスク。
# ~/.config/jj/config.toml の `jj push` エイリアスから呼ばれる。
# 1 つでも失敗したら、その時点で止まり、push しない。
set -eu

cd "$(dirname "$0")/.."

echo "== pre-push: tools/*.py の構文チェック"
python3 - tools/*.py <<'PY'
import ast, sys
for path in sys.argv[1:]:
    with open(path, encoding="utf-8") as f:
        ast.parse(f.read(), path)
    print("  OK", path)
PY

echo "== pre-push: tools/*.py の --help"
for f in tools/setup_atgm336h.py tools/measure_atgm336h.py; do
    if [ -f "$f" ]; then
        PYTHONDONTWRITEBYTECODE=1 python3 "$f" --help >/dev/null
        echo "  OK $f --help"
    fi
done

echo "== pre-push: pio test -e native"
if command -v pio >/dev/null 2>&1; then
    PIO=pio
elif [ -x "$HOME/.platformio/penv/bin/pio" ]; then
    PIO="$HOME/.platformio/penv/bin/pio"
else
    echo "pio が見つかりません（PATH にも ~/.platformio/penv/bin/pio にもありません）" >&2
    exit 1
fi
"$PIO" test -e native

echo "== pre-push: すべて OK"
