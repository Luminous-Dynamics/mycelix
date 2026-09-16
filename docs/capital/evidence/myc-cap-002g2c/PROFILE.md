# MYC-CAP-002G2C — Successor constitution designation and atomic cutover v1

Status: candidate executable profile. Superseded run `35087196258` passed exact-subject/parent verification and Python compilation, then all 24 tests errored only because the legacy test harness looked for `scripts/capital/example_case.json`. The verifier itself was not semantically falsified. This replacement stages the exact checked-in fixture for the test harness while retaining the original verifier/test/fixture/receipt bytes.

## Purpose

Qualify one atomic active-registry cutover from constitution epoch N to the exact N+1 successor committed by one accepted G2B amendment.

Core separation:

```text
accepted amendment != successor designation
successor designation != execution authority
STALE compare-and-swap != semantically invalid amendment
```

## Atomic cutover theorem

The designation event binds both:

```text
expected_current_checkpoint_sha256
expected_current_chain_tip_sha256
```

An `ACTIVE` result requires an ACTIVE live checkpoint, exact accepted G2B receipt, no amendment blockers, no self-designation claim, contiguous N→N+1 epoch, exact prior/profile match, exact successor profile bytes/digest, exact designated authority, and a successful compare-and-swap against the exact current checkpoint and chain tip.

The successor G1 profile is revalidated at cutover and must preserve the protected chamber topology and the asset-lock/steward-seat-sale invariants.

## Concurrency semantics

```text
ACTIVE
BLOCKED
STALE
```

`STALE` means currentness/CAS conflict, not semantic invalidity. Two candidates verified from the same checkpoint cannot both become active: once one succeeds, the other becomes stale. Replaying the winning designation against its successor checkpoint is likewise stale.

v1 has no rollback pointer. Restoring an older policy requires a fresh amendment and a later monotonic epoch.

## Output boundary

Only `ACTIVE` emits a successor checkpoint and fixes:

```text
prior_epoch_decisions_become_historical = true
```

Every receipt also fixes:

```text
execution_authority_established = false
legal_validity_established = false
democratic_legitimacy_established = false
```

Designation makes one constitution current; it does not execute a governance decision.

## Semantic commitments

Designation profile semantic SHA-256:

`851a248ab374f6e7435ddb2d78cb264eccf525956e792d9a4c395df75e6659a2`

Live checkpoint semantic SHA-256:

`001d5a00e3db07683c3ac33fbf511c7a4a08bad9451cefe1463870a75bf5f660`

Accepted G2B receipt semantic SHA-256:

`d26ee8750ef11b23809321501cdd2dc8e7d491b9e12a5baa5ace9afea85b339b`

Successor G1 profile semantic SHA-256:

`84cbcaff53366ab611402fc5114e9f4eafa066ef753f8e722ae9caff13b79284`

Designation event semantic SHA-256:

`bcf0ebd1d031d630da116334c83aa0942210da42807c1cbba8b000e8968f2ab6`

Frozen designation receipt semantic SHA-256:

`c87c5ebe735957458f9d4363abec8a67e9d37e7754fbab016d49e0e0479b6d70`

Implementation/test/fixture/receipt file identity is committed by the exact Git subject and exact six-file diff scope. The hosted workflow reconstructs the receipt twice and requires byte identity; manually duplicated source-file SHA literals are not a second authority plane.

## Test surface

The stdlib suite contains **24** fail-closed regressions covering positive cutover, blocked/self-designating amendment, checkpoint/chain-tip staleness, competing cutover/replay staleness, authority/project/registry substitution, amendment/successor substitution, successor byte mismatch, protected-invariant weakening, capital chamber injection, epoch skip, deterministic event identity, authority contamination, strong nonclaims, and determinism.

The qualification workflow stages the exact checked-in fixture into the legacy test-harness location with automatic cleanup, then separately invokes the verifier against the canonical evidence path for receipt reconstruction.

## Nonclaims

Even hosted PASS would establish only deterministic registry cutover under this frozen profile and supplied evidence. It would not establish execution authority, legal/constitutional validity, democratic legitimacy, identity/signature authenticity, social consensus, jurisdictional enforceability, or wisdom.
