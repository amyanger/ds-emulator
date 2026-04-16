#!/bin/bash
# PreToolUse hook: block any Bash command that would stage or commit
# CLAUDE.md. Project house rule — CLAUDE.md is gitignored and must
# stay untracked.

set -euo pipefail

cmd=$(jq -r '.tool_input.command // empty')

if [[ -z "$cmd" ]]; then
  exit 0
fi

# Block `git add ... CLAUDE.md` (with or without -f) and any direct
# commit that names CLAUDE.md as a pathspec.
if echo "$cmd" | grep -qE '(^|[^a-zA-Z0-9_])git[[:space:]]+add([[:space:]]+-[A-Za-z]+)*([[:space:]]+\S+)*[[:space:]]+.*CLAUDE\.md'; then
  echo "BLOCKED: CLAUDE.md is gitignored and must never be committed (project house rule in CLAUDE.md)." >&2
  exit 2
fi

if echo "$cmd" | grep -qE '(^|[^a-zA-Z0-9_])git[[:space:]]+commit[[:space:]].*CLAUDE\.md'; then
  echo "BLOCKED: CLAUDE.md is gitignored and must never be committed (project house rule in CLAUDE.md)." >&2
  exit 2
fi

exit 0
