# mycelix-business-hospitality-strong-field

Composed strong, read-only field qualification for the Mycelix hospitality shadow pilot.

This crate is the first layer that requires the independent import/time/model proofs to agree before emitting one stronger field report. It does not grant authority and contains no provider mutation surface.

## Required proof chain

A report is constructed only after all of the following succeed:

1. pilot registration and model/baseline lineage validation;
2. preregistered transition-aware local-time schedule validation;
3. exact extraction-campaign replay;
4. campaign-wide accepted source-event uniqueness;
5. exact normalized membership for every forecast actual;
6. overall candidate-vs-baseline scoring;
7. deterministic breakfast/lunch/evening/weekend slice derivation;
8. extraction-bound denominator/data-quality validation.

Forecast windows that cross an offset transition fail closed in v0.1. A later version may introduce an explicitly preregistered split-window policy, but this implementation does not invent one after seeing results.

## Limitation handling

This stronger path does not carry the older:

- `limitation:fixed-offset-clock-rules:v1`; or
- `limitation:forecast-actual-campaign-membership-unverified:v1`.

Those limitations belonged to weaker historical evidence and are not rewritten. This is a separate stronger report that requires transition-aware time evidence and exact actual membership from the outset.

Two limitations remain explicit:

- `limitation:upstream-export-completeness-unverified:v1` — replay can prove what is present in an export, not that the provider exported every real transaction;
- `limitation:time-rule-source-authority-unverified:v1` — the civil-time rules are preregistered and digest-bound, but this crate does not certify that the supplied rule source is authoritative tzdata or jurisdictional evidence.

## Anti-gaming properties

- candidate/baseline/actual identities and target windows must be unique;
- candidate and baseline lineage must match preregistration;
- exact actual observations are replayed from the exact registered source files;
- a bad overall model still cannot hide malformed field/data-quality evidence;
- time semantics must be frozen no later than the pilot protocol itself;
- obsolete weaker limitations cannot be reintroduced into a strong report;
- report, scorecard, slice, campaign-integrity, membership, time-schedule, and field evidence are digest-bound.

## Safety boundary

`PassShadowFieldGate` remains evidence about one model/profile/scope/protocol. It is not an authorization token, purchase approval, scheduling permission, or write capability.
