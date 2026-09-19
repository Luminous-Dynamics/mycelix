# Forge Change-Set Provenance v1

Issue: CI-GOV-001C3A / #1798.

This is an **offline read-only evidence producer** for the exact path set between
one exact base commit and one exact head commit. It does not decide which Forge
CI lanes execute and grants no cancellation, merge, or qualification authority.

## V1 comparison profile

The first profile is deliberately narrow:

- Git object format must be SHA-1;
- base/head are lowercase 40-hex commit IDs;
- base must be an ancestor of head;
- paths come from exact base-tree -> head-tree difference;
- external diff and textconv are disabled;
- rename detection is disabled;
- names are NUL-delimited;
- non-UTF-8 or noncanonical repository paths are refused.

The exact diff profile is:

`git-tree-diff-no-renames-v1`

A rename therefore appears conservatively as deletion of the old path plus
addition of the new path.

## Failure semantics

Any producer refusal must become **FullMatrix** in future CI integration.

In particular, v1 refuses non-ancestor PR state rather than guessing merge-base
semantics. A later profile may define merge-base behavior separately.

## Evidence object

A successful result binds:

- Git object format;
- observed Git version;
- exact base commit;
- exact head commit;
- diff profile;
- deterministic sorted changed paths;
- changed-path count;
- a domain-separated SHA-256 integrity commitment over those fields.

The commitment is integrity plumbing, not a signature or authority grant.

## Trusted event boundary

The CLI accepts base/head values for offline testing and replay. Future workflow
suppression authority must **not** trust arbitrary caller-selected OIDs. The
workflow must independently bind them to the trusted pull-request event's exact
base/head subject before consuming this evidence.

Conceptually:

`caller supplied base/head != trusted PR comparison subject`

## Rollout

1. independently qualify this producer;
2. bind exact PR base/head from trusted workflow event data;
3. feed the resulting change set into the source-bound #1786 selector;
4. run observation-only proposals while the full admitted Forge matrix executes;
5. only a later qualified convergence theorem may suppress lanes.
