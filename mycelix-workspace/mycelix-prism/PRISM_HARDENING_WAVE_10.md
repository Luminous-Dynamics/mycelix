# Prism hardening Wave 10

Wave 10 turns Prism's release evidence into a monotonic, policy-governed release
system. It begins at the exact Wave 9 tree and is intentionally organized as an
ordered mail-patch series. Apply every patch in numeric order; do not apply the
cumulative series on top of an already hardened tree.

## Objectives

1. Replace loosely interpreted index evidence with one typed producer/verifier
   contract.
2. Make signer identity, release channel, sequence, predecessor, and version
   scope independently anchored policy decisions.
3. Preserve admitted release history locally and reject rollback, gaps, and
   same-sequence equivocation.
4. Bind the files users actually download, not only the source and build logs.
5. Make ceremony reads and publication resistant to path substitution,
   overlapping roots, and replacement races.
6. Provide an exact operator ceremony and reproducible static evidence without
   claiming unavailable Cargo, Nix, namespace, or cryptographic execution.

## Ordered patch sets

### 1. Typed index publication receipts

`IndexBuildReceipt`, `PublishedIndexEvidence`, and `ReviewInputEvidence` become
shared exact schemas. The index producer self-validates the receipt it writes;
release preparation and verification consume the same type and reject unknown,
missing, or mode-inconsistent fields.

### 2. Typed receipt consumption in release attestations

The release path no longer selects fields from an untyped JSON value. It
requires a reviewed core/full receipt with identical reviewer policy, policy
history, and ledger roots.

### 3. Externally anchored release policy

A strict `ReleasePolicy` names the project, policy ID, channel, signer key,
sequence, predecessor, and admitted version scope. The exact policy bytes are
anchored through `PRISM_RELEASE_POLICY_BLAKE3`.

### 4. Signed monotonic release lineage

Release schema v5 adds channel, sequence, predecessor statement digest, and the
exact release-policy digest to the canonical hybrid-signed statement.

### 5. Policy enforcement across the ceremony

Preparation, offline assembly, and independent verification all load and apply
the same externally anchored policy. A signer station cannot assemble a request
for another channel, signer, sequence, predecessor, or version scope.

### 6. Canonical artifact semantics

Every release artifact kind has one canonical logical name and media type. Those
semantics are included in canonical signing bytes, preventing a digest from
being reinterpreted as another artifact type.

### 7. Append-only rollback-resistant release journal

Independent verification appends a checkpoint only after both hybrid signature
components and every local artifact pass. The journal rejects rollback,
sequence gaps, broken predecessor links, and same-sequence equivocation.

### 8. Unified ceremony filesystem admission

Release preparation and verification use shared stable readers, path-overlap
checks, and create-once publication primitives.

### 9. Hardened journal channel and lock admission

Release state is channel-specific, caller-owned, private, and canonical. The
journal lock is uniquely linked, mode-restricted, UID-checked, and protected by
`flock`.

### 10. Cross-crate receipt and policy substitution tests

System-level tests prove that semantically altered index receipts and
signer/lineage policy substitutions fail across package boundaries.

### 11. Wave 10 canonical evidence lane

Rust, Nix, workflow, release-verifier, and live-gate references advance to
`wave10-static`; historical Wave 9 evidence remains immutable.

### 12. Wave 10 static and supply-chain receipts

The repository publishes reproducible static and dependency-admission receipts
for the promoted lane.

### 13. Checkpoint-derived successor policy

`prism-prepare-release-policy` derives the next exact sequence and predecessor
from a checkpoint emitted by successful independent verification. Operators no
longer hand-type lineage values.

### 14. Evidence for policy derivation

The static gate binds the successor-policy tool, checkpoint derivation,
create-once publication, and printed external anchor.

### 15. Hardened live release ceremony

The fixed live lane now uses an explicitly admitted Git executable with external
configuration disabled, stable file measurements, non-overlapping evidence and
state roots, private directories, exact one-checkpoint advancement, and Linux
`renameat2(RENAME_NOREPLACE)` publication.

### 16. Measured release payload manifests

Release schema v6 requires `prism-release-payloads.json`. A typed descriptor and
preparation tool measure every distributable file's name, media type, target,
executable intent, size, SHA-256, and BLAKE3. Preparation and independent
verification both remeasure the exact payload directory and reject any extra,
missing, renamed, altered, or permission-changed file.

### 17. Segment-exact version policy

Version prefix matching is boundary-aware: `1.2` admits `1.2.0` but not
`1.20.0`. Prefixes reject leading-zero numeric components and ambiguous repeated
separators.

### 18. Pathname-bound stable reads

Ceremony inputs, reviewer trust files, build evidence, and release payloads now
verify both that the opened inode remained stable and that the original pathname
still names that same inode after reading.

### 19. Operator release ceremony

`security/RELEASE_CEREMONY.md` documents the exact Nix lock review, payload
measurement, policy derivation, offline hybrid signing, independent
verification, rollback-journal advancement, and fixed live-gate order. A strict
example payload descriptor is included.

### 20. Wave 10 campaign document

This document records the threat model, patch order, validation boundaries, and
remaining release blocker without rewriting historical evidence.

## Security invariants after Wave 10

- A release is not valid merely because a signature verifies.
- The release signer, channel, sequence, predecessor, and version scope are
  external policy inputs bound into the signed statement.
- The latest admitted release cannot be replaced by an older, skipped, or
  equivocal sequence without deleting both local state and its external anchor.
- Build evidence, reviewed index evidence, lock review, and distributable
  payloads are exact typed inputs rather than labels attached to arbitrary JSON.
- Every shipped file is measured independently before signing and during final
  verification.
- Security-critical reads reject duplicate JSON, schema extension, symbolic
  links, insecure links or permissions, mid-read mutation, and pathname
  replacement.
- Ceremony outputs are create-once; the live evidence directory uses a
  no-replacement kernel primitive.

## Canonical validation commands

```sh
python3 scripts/verify-wave10-static.py \
  --check security/wave10-static-evidence.json
python3 scripts/verify-supply-chain.py \
  --check security/supply-chain-evidence.json
python3 scripts/verify-supply-chain.py --release
python3 scripts/run-wave10-live-gates.py --help

git diff --check
git fsck --full
```

The complete executable release ceremony is in
`security/RELEASE_CEREMONY.md`.

## Honest remaining blocker

The uploaded environment does not provide Cargo, Rust, Nix, Bubblewrap,
Chromium, or the production verifier agent. More importantly, this source still
has no generated and reviewed `flake.lock`. Release admission therefore remains
fail-closed until a Nix-enabled canonical host:

1. generates and commits `flake.lock` without mutable inputs;
2. performs and hybrid-signs its semantic review;
3. runs frozen Rust and no-update Nix evidence lanes;
4. measures the final payload directory;
5. executes the namespace and adversarial live tests;
6. completes the hybrid release ceremony; and
7. advances and externally anchors the release checkpoint.
