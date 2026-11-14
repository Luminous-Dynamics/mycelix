# FSRE as-of evidence admissibility reference v0.1

**Status:** Deterministic read-only temporal admissibility reference. Synthetic fixture only. It does not authenticate sources, prove physical truth, make operational decisions or grant authority.

## Why this exists

A recovery claim can be true in hindsight yet unsupported at the time a decision was made. Conversely, evidence may have been valid and available to an operator at the time, then be retrospectively revoked or corrected later.

This reference separates:
- **Observation time** — when the phenomenon was measured.
- **Availability time** — when the evidence was actually available to the declared decision process.
- **Valid time** — when the asserted state was true in the world.
- **Revocation effective time** — when the source says the evidence ceased to apply.
- **Revocation knowledge time** — when that revocation became available to the decision process.
- **Decision cutoff** — the point-in-time replay being evaluated.

Validity intervals are half-open: `[valid_from, valid_until)`. The end instant is excluded. All timestamps are non-negative seconds in the explicitly declared `UTC_UNIX_SECONDS_V1` basis. The reference rejects clock-basis disagreement instead of silently comparing incompatible clocks.

## Dispositions

An evidence row is assessed as one of:
- `ADMISSIBLE`: observed and available by the cutoff, within its validity interval and freshness limit.
- `NOT_YET_AVAILABLE`: observed but not available to the decision process by the cutoff.
- `FUTURE_OBSERVATION`: the observation itself occurred after the cutoff.
- `NOT_YET_VALID`, `EXPIRED`, or `STALE`: it does not support the claim at the cutoff.
- `REVOKED_AS_KNOWN`: revocation was both known and effective by the cutoff.
- `REVOCATION_PENDING`: revocation was known but its effective time is in the future.
- `ADMISSIBLE_WITH_HINDSIGHT_REVOCATION`: the revocation became known only after the cutoff, so the evidence remains admissible in the as-known replay; the later correction is disclosed separately.
- `UNKNOWN`: the source does not support an assertion.

A claim is `SUPPORTED_AS_KNOWN_AT_CUTOFF` only when its declared minimum number of admissible evidence references is met. Otherwise it is `NOT_ESTIMABLE`. The minimum is a claim-owner requirement, not an evidence-confidence score. Duplicate source references cannot be counted twice for the same claim.

## Input contract

Metadata declares scenario, decision cutoff, maximum observation age and clock basis. `CLAIM` rows specify claim owner/reference and required minimum evidence count. `EVIDENCE` rows contain temporal facts, source owner and opaque reference. An `UNKNOWN` evidence row cannot carry temporal assertions.

The replay uses only `available_at <= decision_at`. Later-known revocations are reported as hindsight; they do not rewrite what was knowable at the historical cutoff. This is a replay semantics, not a statement that the evidence remained objectively correct.

## Run

`cargo test --locked --offline --manifest-path tools/fsre-asof-evidence/Cargo.toml`

`cargo run --quiet --locked --offline --manifest-path tools/fsre-asof-evidence/Cargo.toml -- tools/fsre-asof-evidence/fixtures/synthetic-asof.psv`

The tool preserves owner and source references but does not authenticate them. It does not replace the owner-native evidence stores, W3C PROV, OGC temporal records, a bitemporal database, or a canonical authority service.
