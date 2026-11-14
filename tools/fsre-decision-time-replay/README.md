# FSRE decision-time evidence replay v0.1

**Status:** Deterministic, read-only bitemporal evidence eligibility reference. Synthetic fixture only. Not an evidence ledger, authority service, adjudicator, or operational control.

## Why this exists

A later correction must not silently rewrite what an operator could have known when a decision was made. Conversely, a fact that was true at a point in time but had not yet been observed or entered into the decision system was not available to that decision-maker.

This reference keeps separate:
- valid time: when the evidence claim applies in the represented world;
- observation time: when the source observed or issued the evidence;
- transaction time: when the system recorded the evidence;
- revocation time: when the evidence was withdrawn;
- decision time: the information frontier being reconstructed.

A version is eligible for as-of selection only when both observation and recording time are at or before the decision cut-off. The latest eligible revision is then classified against its valid-time interval and revocation time.

## Input contract

Metadata declares one scenario, decision ID, decision timestamp, evidence snapshot ID and time basis. Rows bind an evidence ID and monotonically increasing revision to one stable claim/subject/predicate/owner. Timestamps are non-negative Unix seconds in the declared time basis. Valid intervals are half-open: `[valid_from, valid_until)`.

The parser rejects transaction time earlier than observation time, duplicate revisions, revisions that change claim identity/subject/predicate/owner, and transaction timestamps that move backward across revisions. Future observations or recordings are retained as excluded revisions, not used to reconstruct the past.

## Output

Per evidence ID the report records the selected as-of revision or the absence of one, whether that revision was valid/not-yet-valid/expired/revoked/unknown at decision time, and how many later versions were excluded.

The aggregate result applies only to submitted evidence IDs. The tool does not know whether the submitted IDs form a complete set of required claims. It does not authenticate owners or references, determine the truth of assertions, establish legal authority, or authorise an action.

## Run

`cargo test --locked --offline --manifest-path tools/fsre-decision-time-replay/Cargo.toml`

`cargo run --quiet --locked --offline --manifest-path tools/fsre-decision-time-replay/Cargo.toml -- tools/fsre-decision-time-replay/fixtures/synthetic-decision.psv`

This is an as-known-at-the-time replay primitive, not a complete bitemporal database or immutable event ledger.
