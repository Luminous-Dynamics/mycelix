# mycelix-business-hospitality-derived-evidence

Protocol-bound, read-only evidence derivation for the Mycelix hospitality shadow pilot.

This crate removes a remaining optimistic-evaluation surface from the field-qualification path: callers do **not** provide `shadow_gate_passed` for breakfast, lunch, evening, or weekend slices. Instead, the crate validates exact `ForecastCase` records, derives slice membership, scores each slice, derives the pass/fail result, and only then constructs the lower-level `SliceEvidence` consumed by the generic field gate.

## Boundaries

This crate:

- grants no authority;
- performs no business writes;
- creates no purchasing, scheduling, refund, payment, or other execution command;
- does not turn a forecast or scorecard into an observation;
- does not claim that a forecast's actual observation is proven to belong to the extraction campaign until a stronger cross-binding exists.

The last limitation is preserved explicitly as `limitation:forecast-actual-campaign-membership-unverified:v1`.

## Qualification bindings

Before scoring, every case must match the preregistered:

- candidate model lineage;
- baseline model lineage;
- evaluation window;
- operational scope;
- external source system; and
- mapping digest.

Candidate forecast IDs, baseline forecast IDs, actual observation IDs, source-event IDs, and target windows are unique within the evaluation set so duplicate records cannot gain extra statistical weight.

The overall forecast scorecard is derived from the same case set and evaluated through the preregistered shadow protocol.

## Slice clock v1

Slice assignment uses a separately preregistered fixed UTC offset bound to the pilot registration and exact slice predicate digests. It intentionally does **not** claim to implement an IANA timezone database or daylight-saving transitions. Evidence therefore carries `limitation:fixed-offset-clock-rules:v1`.

Daypart windows must remain within one local day. Weekend membership is limited to one weekend day or a Saturday-to-Sunday crossing; a multi-day window cannot qualify merely because both endpoints happen to be weekends.

## Evaluation order

Extraction and field evidence are validated even when the candidate model already fails its overall shadow gate. A bad model therefore cannot conceal a malformed denominator, connector mismatch, or invalid data-quality claim.

A successful result is still only `PassShadowFieldGate` evidence for the registered scope/model/protocol. It grants no execution authority and does not promote the capability beyond shadow evaluation.
