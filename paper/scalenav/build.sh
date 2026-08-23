#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TEX_CACHE_DIR="${TMPDIR:-/tmp}/scalenav-texmf"

mkdir -p \
  "${TEX_CACHE_DIR}/var" \
  "${TEX_CACHE_DIR}/config" \
  "${TEX_CACHE_DIR}/home"

cd "${SCRIPT_DIR}"

export TEXMFVAR="${TEX_CACHE_DIR}/var"
export TEXMFCONFIG="${TEX_CACHE_DIR}/config"
export TEXMFHOME="${TEX_CACHE_DIR}/home"

for pass in 1 2; do
  xelatex \
    -interaction=nonstopmode \
    -halt-on-error \
    root.tex
done

printf 'Built %s/root.pdf\n' "${SCRIPT_DIR}"
