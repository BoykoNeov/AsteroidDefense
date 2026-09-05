---
name: always-commit-push
description: "Standing order — commit AND push at every batch boundary, not just \"session end\""
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 0a0b19eb-ba5d-4c0d-a10e-34a497cf5af2
  modified: 2026-07-20T18:35:01.275Z
---

The user's standing instruction (2026-07-20): **always commit and push.** Not
just at an explicit "session end" — at the end of every coherent batch of work,
commit and push to the remote without waiting to be told.

**Why:** The user wants durable, pushed checkpoints as the default, so work is
never sitting uncommitted locally. This strengthens [[git-workflow]] (which
already said commit+push at session end) into "every batch."

**How to apply:** After finishing a logical unit of work — a feature commit, a
memory/doc update, a fix — run the commit and `git push`. Don't ask whether to
commit; just do it. Keep commit messages substantive per the repo's style.
