---
name: git-workflow
description: When to commit/push and update memory+docs for the AsteroidDefense repo
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f5cc34dd-dad9-418c-9791-57031e47c59c
---

Always update memory and docs, then commit and push, at the end of every work
batch, planning session, or docs update — and whenever the user says "session
end."

**Why:** The user wants the public repo and the persistent memory to stay in
lockstep with progress, so nothing is lost between sessions and the repo always
reflects the latest thinking. Stated explicitly when the repo was created
(2026-06-23).

**How to apply:** Treat "finished a batch of work / planning / a docs edit" or an
explicit "session end" as a trigger to: (1) update relevant memory files and the
docs (e.g. [[project-overview]], HANDOFF.md, README.md); (2) mirror the changed
memory files into the repo's `memory/` directory so the committed copy stays
byte-identical to the canonical live copy (the canonical copy lives outside the
repo in the user's local Claude config); (3) `git add` + commit with a clear
message; (4) `git push`. Don't wait to be asked each time. Repo is public on
GitHub — see [[project-overview]] for the remote.
