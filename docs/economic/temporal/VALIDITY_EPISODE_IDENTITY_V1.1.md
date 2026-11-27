# Validity episode identity — Temporal Evidence v1.1

Status: implemented in the v1.1 Rust reference kernel for finite intervals; explicit open-end assertions remain future work.

## Problem

A subject and claim can have more than one validity period. For example, a grant may be active in January, revoked in February, and reissued in April. Selecting the latest maximal VALID_FROM and VALID_TO independently by (subject, claim, verifier) can pair different periods.

## Contract

- Every v1.1 VALID_FROM and VALID_TO carries a stable, non-empty `validity_interval_ref`.
- A finite interval is formed only when its start and end have the exact same episode reference.
- Episode identity is scoped by the existing claim, subject, owner/history, verifier and policy/frontier context; the reference is not globally authoritative by itself.
- Corrections supersede endpoints only within the same episode reference. A correction cannot move an endpoint between episodes.
- Query callers specify the exact episode reference through `evaluate_validity_interval_at`.
- The scan, indexed and prevalidated scan engines expose the same episode-scoped API and are cross-compared.
- An unscoped query across multiple simultaneous episodes is Conflict/Indeterminate, never an arbitrary selection.
- Legacy v1.0 untagged inputs remain structurally parseable by the Rust in-memory model, but the finite interval query does not promote them to episode-complete answers.
- Missing VALID_TO is Indeterminate. The old caller boolean cannot establish OPEN.

## Migration

- Preserve the original v1.0 JSON schema at `docs/economic/temporal/schemas/temporal-evidence-envelope.v1.schema.json` and its unmodified fixture at `docs/economic/temporal/fixtures/temporal-evidence-envelope.v0.1.json`.
- The new v1.1 schema is `docs/economic/temporal/schemas/temporal-evidence-envelope.v1.1.schema.json`; its matching fixture is `docs/economic/temporal/fixtures/temporal-evidence-envelope.v1.1.json`.
- Readers should dispatch by the explicit envelope `version`, not by a filename, timestamp heuristic or presence inferred from unrelated fields.
- The Rust in-memory API accepts `None` for legacy endpoints for compatibility, but such endpoints cannot yield an episode-complete answer.

The JSON envelope schema advances to version 1.1 and requires `validity_interval_ref` for VALID_FROM/VALID_TO. Non-endpoint dimensions cannot carry it. Existing persisted records require an explicit migration decision; do not synthesize episode identifiers from timestamps or array position. Such synthesis would create an unreviewed semantic claim.

## Tests

The v1.1 kernel includes:
- two disjoint periods under one claim, queried independently;
- exact identity returned in the result;
- parity across scan, prevalidated scan and index;
- cross-episode start/end mismatch -> Indeterminate;
- untagged legacy pair -> Indeterminate;
- correction changes episode identity -> rejected;
- absent VALID_TO plus caller opt-in -> Indeterminate.

The corpus adds TME-037 through TME-042 to the original 36 temporal adversarial cases.

## Nonclaims

This does not derive endpoints from canonical assertion payloads, authenticate a source checkpoint, prove a verifier frontier complete, or authorize legal/financial effects. The separate semantic mapper work remains tracked by [#6786](https://github.com/Luminous-Dynamics/mycelix/issues/6786). Explicit open-end disposition is also not yet implemented.
