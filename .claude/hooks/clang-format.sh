#!/bin/bash
# PostToolUse hook: auto-format C/C++ files after Edit/Write.
# Reads Claude Code hook JSON from stdin, runs clang-format if the edited
# file is a C/C++ source or header. Silently no-ops for other file types.

set -euo pipefail

file=$(jq -r '.tool_input.file_path // empty')

if [[ -z "$file" || ! -f "$file" ]]; then
  exit 0
fi

case "$file" in
  *.cpp|*.cxx|*.cc|*.hpp|*.hxx|*.h)
    if command -v clang-format >/dev/null 2>&1; then
      clang-format -i "$file" 2>/dev/null || true
    fi
    ;;
esac

exit 0
