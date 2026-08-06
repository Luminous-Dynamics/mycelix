#!/usr/bin/env python3
"""check-author-binding.py — fail when a NEW create-validator discards its action.

WHY
---
A Holochain integrity validator is the only thing a malicious peer cannot skip:
coordinator functions are advisory, because a peer runs its own conductor and
commits straight to its source chain. So if `validate_create_X` takes `_action`
and throws it away, any self-reported identity field on that entry — `from_did`,
`voter_did`, `owner_did` — is a string nobody ever checks against the signer.

That is not hypothetical here. The 2026-07 audit found 48 of 51 named Class-A
validators in `mycelix-finance` and `mycelix-governance` open, 19 days after a
triage doc named every one with file and line. The fix pattern is mechanical; the
failure mode is that nothing *notices* when a new one appears — and this campaign
has already lost a landed binding once (commit 48a4460dcf).

This check converts that audit from a one-time sweep into a standing invariant.

WHAT IT DOES
------------
Flags every `fn validate_create_*(_action: ...)` in an integrity crate. A hit is
allowed only if it appears in `scripts/author-binding-allowlist.txt` WITH A
REASON. That file is the point: it forces each exemption to be a recorded
decision rather than an oversight.

Legitimate reasons (from the audit's taxonomy) — see
MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md:

  Class C  no forgeable owner field — the entry has nothing to bind.
  Class D  the committer is legitimately NOT the subject. Two shapes:
             * two-party / on-behalf-of records (a mediator resolving a dispute)
             * shared mutable state (`SapBalance` — `transfer_sap` credits the
               payee from the SENDER's agent context)
           These need derivation from a validated witness entry, NOT an author
           bind. Binding them breaks real flows — see risk R1 in
           MYCELIX_PHASE1_EXECUTION_PLAN_2026-07-28.md.

Before adding an allowlist entry, read the coordinator's creation path. If every
legitimate caller IS the subject, it is Class A: bind it, do not exempt it.

USAGE
-----
    scripts/check-author-binding.py                  # CI gate
    scripts/check-author-binding.py --list           # inventory, exit 0
    scripts/check-author-binding.py --update-allowlist   # seed/refresh

Exit 0 = clean, 1 = an unexplained create-validator discards its action.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ALLOWLIST = os.path.join(ROOT, "scripts", "author-binding-allowlist.txt")

# `fn validate_create_<name>(` whose first parameter is `_action` (discarded).
PATTERN = re.compile(r"fn\s+(validate_create_\w+)\s*\(\s*_action\s*:", re.S)


def integrity_sources() -> list[str]:
    """Tracked .rs files inside an `integrity` crate."""
    try:
        out = subprocess.run(
            ["git", "ls-files", "-z", "--", "*.rs"],
            cwd=ROOT, capture_output=True, check=True,
        ).stdout.decode("utf8", "replace")
    except Exception:
        return []
    return [
        p for p in out.split("\0")
        if p and f"{os.sep}integrity{os.sep}" in f"{os.sep}{p}"
    ]


def find_hits() -> list[str]:
    hits: list[str] = []
    for rel in integrity_sources():
        path = os.path.join(ROOT, rel)
        try:
            with open(path, encoding="utf8", errors="replace") as fh:
                src = fh.read()
        except OSError:
            continue
        for m in PATTERN.finditer(src):
            line = src[: m.start()].count("\n") + 1
            hits.append(f"{rel}:{line}:{m.group(1)}")
    return sorted(hits)


def load_allowlist() -> set[str]:
    """Allowlist lines are `path:line:fn  # reason`. The line number is ignored
    when matching, so unrelated edits above a validator do not spuriously fail."""
    if not os.path.exists(ALLOWLIST):
        return set()
    keys: set[str] = set()
    with open(ALLOWLIST, encoding="utf8") as fh:
        for raw in fh:
            entry = raw.split("#", 1)[0].strip()
            if not entry:
                continue
            keys.add(key_of(entry))
    return keys


def key_of(hit: str) -> str:
    parts = hit.split(":")
    return f"{parts[0]}::{parts[-1]}" if len(parts) >= 3 else hit


def main() -> int:
    args = sys.argv[1:]
    hits = find_hits()

    if "--list" in args:
        for h in hits:
            print(h)
        print(f"\n{len(hits)} create-validator(s) discard their action.")
        return 0

    if "--update-allowlist" in args:
        existing: dict[str, str] = {}
        if os.path.exists(ALLOWLIST):
            with open(ALLOWLIST, encoding="utf8") as fh:
                for raw in fh:
                    if "#" in raw and raw.split("#", 1)[0].strip():
                        body, reason = raw.split("#", 1)
                        existing[key_of(body.strip())] = reason.strip()
        with open(ALLOWLIST, "w", encoding="utf8") as fh:
            fh.write(
                "# Create-validators that deliberately discard their action.\n"
                "# Format:  <path>:<line>:<fn>  # <Class C|Class D>: <why>\n"
                "#\n"
                "# The check fails on any create-validator NOT listed here, so this file\n"
                "# stops the backlog from growing while it is worked down. Entries are\n"
                "# matched on path+function, NOT line number.\n"
                "#\n"
                "# DO NOT add an entry to silence a failure without reading the\n"
                "# coordinator's creation path first. If every legitimate caller IS the\n"
                "# subject, it is Class A — bind it, do not exempt it.\n"
                "# See MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md for the taxonomy.\n"
            )
            for h in hits:
                reason = existing.get(key_of(h), "TODO: triage — Class A (bind), C (nothing to bind), or D (authority model)?")
                fh.write(f"{h}  # {reason}\n")
        print(f"allowlist written: {len(hits)} entries -> {os.path.relpath(ALLOWLIST, ROOT)}")
        return 0

    allowed = load_allowlist()
    new = [h for h in hits if key_of(h) not in allowed]

    if new:
        print("author-binding check FAILED\n")
        for h in new:
            print(f"  UNBOUND CREATE VALIDATOR  {h}")
        print(
            f"\n{len(new)} create-validator(s) discard their action and are not allowlisted.\n\n"
            "A discarded action means any self-reported DID on that entry is unchecked\n"
            "against the signer — coordinator checks do NOT cover this, since a malicious\n"
            "peer never calls your coordinator.\n\n"
            "Fix (preferred): bind it —\n"
            "    let author_did = did_for_author(action.author());\n"
            "    if let ValidateCallbackResult::Invalid(msg) =\n"
            "        require_did_is_author(\"Entry\", \"field\", &entry.field, &author_did)\n"
            "    { return Ok(ValidateCallbackResult::Invalid(msg)); }\n"
            "  (both helpers live in crates/mycelix-bridge-entry-types)\n\n"
            "Or, if the committer is legitimately not the subject, add an allowlist entry\n"
            "with a Class C/D reason: scripts/check-author-binding.py --update-allowlist\n"
        )
        return 1

    stale = sorted(allowed - {key_of(h) for h in hits})
    if stale:
        print(f"note: {len(stale)} allowlisted validator(s) are now bound or gone — "
              "re-run with --update-allowlist to shrink the allowlist.\n")

    print(f"author-binding OK — {len(hits)} exemption(s), all explained; no new unbound validators")
    return 0


if __name__ == "__main__":
    sys.exit(main())
