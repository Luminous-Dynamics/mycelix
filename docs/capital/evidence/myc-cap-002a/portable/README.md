# MYC-CAP-002A1 — portable exact-subject qualification evidence

This tranche preserves MYC-CAP-002A candidate subject `270b852e0ac744dfca3a2cb966bf53fce78f2ab9` unchanged and adds a portable evidence/replay layer around it.

## Evidence correction

Hosted run `34959974354` completed successfully and is associated with candidate head `270b852e0ac744dfca3a2cb966bf53fce78f2ab9`. Its original workflow used the default `actions/checkout` behavior for a `pull_request` event.

GitHub's `pull_request` event normally sets `GITHUB_REF` to the synthetic merge ref, and a default checkout therefore tests the merge result rather than literally checking out the PR head commit. Because MYC-CAP evidence uses exact-subject semantics, the successful run is preserved as useful historical hosted evidence, but this child does not pretend that the original lane itself proved literal exact-head execution.

MYC-CAP-002A1 closes that gap by:

1. checking out this child PR's literal `github.event.pull_request.head.sha`;
2. requiring this child to be exactly one commit over `270b852e...`;
3. requiring all six qualified MYC-CAP-002A source files to remain byte-identical to `270b852e...`;
4. creating a detached Git worktree at exactly `270b852e...`;
5. running the original qualification suite and receipt generator inside that exact-subject worktree;
6. building a bounded portable bundle;
7. independently replaying that bundle again against the same exact subject before upload.

## Bundle authority boundary

The uploaded artifact is not a signature and is not self-authenticating authority.

It contains:

- `subject.txt` — the exact qualified subject;
- `source-digests.json` — SHA-256 digests of the six frozen qualified source files as read from that Git commit;
- `case.json` — exact frozen fixture bytes;
- `receipt.json` — byte-for-byte regenerated receipt;
- `qualification.json` — semantic commitments, bounded PASS scope, runtime provenance and hosted-run discovery metadata;
- `NONCLAIMS.md` — strong negative claims.

Independent replay requires the exact Git subject to be available locally. The verifier recomputes source commitments from the commit, reruns the unit qualification in a detached worktree, regenerates the receipt, checks exact bundle membership, and rejects drift or extra authority-bearing members.

## What is canonical

Canonical authority is attached to:

- exact subject Git commit;
- exact frozen source bytes;
- canonical JSON semantic commitments;
- byte-for-byte generated receipt;
- successful independent replay.

The ZIP/container bytes emitted by the artifact service are deliberately not treated as canonical semantic evidence. Archive metadata may vary without changing the evidence members.

## Runtime provenance

The artifact records the Python implementation/version/platform used to build it. Independent replay may use another compatible runtime. Runtime provenance is evidence, not semantic identity for this profile.

## Hosted-run metadata

The original run ID/attempt is retained as discovery/provenance metadata. A green badge or successful run identifier alone is never sufficient PASS authority.

See `NONCLAIMS.md` for the bounded negative claim set.
