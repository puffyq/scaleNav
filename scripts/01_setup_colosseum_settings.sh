#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SOURCE_SETTINGS="${SOURCE_SETTINGS:-$PROJECT_ROOT/config/colosseum_yopo_simple.json}"
SETTINGS_PATH="${SETTINGS_PATH:-$HOME/Documents/Colosseum/settings.json}"

[[ -f "$SOURCE_SETTINGS" ]] || { echo "错误: 找不到配置模板: $SOURCE_SETTINGS" >&2; exit 1; }
command -v python3 >/dev/null || { echo "错误: 需要 python3 校验 JSON。" >&2; exit 1; }
python3 -m json.tool "$SOURCE_SETTINGS" >/dev/null
mkdir -p "$(dirname -- "$SETTINGS_PATH")"
cp -- "$SOURCE_SETTINGS" "$SETTINGS_PATH"
echo "Colosseum settings 已安装到: $SETTINGS_PATH"
