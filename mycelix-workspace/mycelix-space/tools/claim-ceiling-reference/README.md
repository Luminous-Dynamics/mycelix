# Claim Ceiling Meet Reference V1

This is an executable, fail-closed reference algebra for the Epistemic Fabric claim-ceiling contract tracked by [EPI-FABRIC-005 / issue #3475](https://github.com/Luminous-Dynamics/mycelix/issues/3475).

It composes declared ceilings. It does **not** decide whether evidence is true, authenticate sources, authorize actions, or replace the canonical admission service.

## Run

From the repository checkout:

```sh
cargo test --manifest-path mycelix-workspace/mycelix-space/tools/claim-ceiling-reference/Cargo.toml -p claim-ceiling-reference
cargo run --manifest-path mycelix-workspace/mycelix-space/tools/claim-ceiling-reference/Cargo.toml -p claim-ceiling-reference --bin claim-ceiling-meet -- \
  --input mycelix-workspace/mycelix-space/tools/claim-ceiling-reference/fixtures/claim-ceiling-meet-request-v1.json \
  --source-commit "$(git rev-parse HEAD)"
```

A source pin is accepted only if the supplied lower-case commit SHA equals the actual checked-out Git HEAD and the tracked tree is clean. Omitting `--source-commit` emits `UNPINNED_LOCAL_RUN` and records `source_commit_pinned: false`. A syntactically valid but mismatched SHA fails closed.

## Algebra

Each permitted claim is a typed atom coupling predicate, evidence class, derivation kind, and intended use. The coupling matters: intersecting separate lists independently could manufacture a combination no input permitted.

The meet:
- intersects allowed claim atoms, subject IDs, contexts, half-open time windows, and action-scope IDs;
- unions mandatory qualifications, independence controls, and provenance requirements;
- returns an explicit bottom result with the empty dimensions when no common claim/scope exists;
- permits an empty action-scope set without erasing a still-usable analytical claim. Empty means no action scope is represented, not permission by default.

There is no scalar trust score, confidence multiplier, wildcard scope, or implicit promotion. Time boundaries are declared scope, not trusted-time evidence.

## Evidence

- Two JSON Schemas use Draft 2020-12 and are compiled at runtime.
- Five portable golden vectors cover overlap, disjoint subjects, disjoint intervals, incompatible evidence/claim atoms, and disjoint action scopes.
- Unit tests cover idempotence, commutativity, associativity, non-amplification, preservation of obligations, exact checkout binding, malformed input rejection, and schema constraints.
- A deterministic CLI receipt binds the raw input bytes and both schema byte strings by SHA-256.

The vectors are synthetic contract examples, not evidence that any real source, qualification, independence claim, or action permission is valid. This first slice is not yet integrated into production Mycelix admission, Symthaea, Symtropy, or Sol-Atlas consumers.

## Separate policy-profile admissibility gate

The meet computes an intersection; it does not establish that the surviving atoms are permitted by policy. The `claim-ceiling-policy-check` binary compares a candidate against a versioned, explicit `maximum_ceiling` and rejects out-of-profile claim atoms, scopes, time windows, action scopes, or dropped mandatory controls.

```bash
cargo run --locked --manifest-path mycelix-workspace/mycelix-space/tools/claim-ceiling-reference/Cargo.toml \
  -p claim-ceiling-reference --bin claim-ceiling-policy-check -- \
  --input mycelix-workspace/mycelix-space/tools/claim-ceiling-reference/fixtures/claim-ceiling-policy-allowed-v1.json
```

Use `fixtures/claim-ceiling-policy-rejected-v1.json` to inspect a multi-dimension rejection receipt. The gate is a policy-fit check only; a passing receipt grants no evidence-admission or action authority. See [the research contract](../../../../docs/research/CLAIM_CEILING_POLICY_ADMISSIBILITY_V1_2026-10.md).
