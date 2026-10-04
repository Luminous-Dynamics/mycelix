# AC-057 — Temporal Policy-Profile Binding

## Purpose

An economic event must not only identify the policy profile that governed it;
the profile must also have been active when the event occurred.

AC-057 adds an explicit effective-interval check to the Economic OS policy
profile and binds event and policy-decision timestamps to that interval.

## Invariant

For a supplied profile P and timestamp T:

- P must have the exact referenced profile ID;
- P must have the exact referenced content fingerprint;
- T must be at or after P.effective_from;
- when P.effective_until exists, T must be at or before that value.

A later profile supersession does not rewrite historical meaning. Historical
events continue to point to the profile that was active for their event time.

## Enforcement points

### Policy profiles

`EconomicPolicyProfile::is_active_at(timestamp)` provides the deterministic
interval predicate.

### Governed policy decisions

`apply_governed_adjustment_for_profile` now checks that the policy decision
timestamp falls inside the supplied profile interval.

### Economic OS envelopes

`validate_against_profiles` verifies every policy context against an
authoritative supplied profile set and the event occurrence timestamp.

The ordinary structural `validate()` method remains transport-friendly and does
not require a profile registry. The stronger method is used when exact profile
materialization is available.

## Why this matters

This makes historical policy interpretation reproducible:

**event time → exact profile identity → profile version → effective interval**

Without the temporal binding, an otherwise valid content fingerprint could be
attached to a time at which that profile had not yet taken effect or had already
ceased to apply.

## Non-claims

This is a technical consistency check. It does not determine which jurisdiction
has legal priority, resolve conflicts between legal systems, or prove that a
policy authority was legitimate.

## Tests

The reference implementation covers:

- open-ended profile intervals;
- bounded profile intervals;
- expired-profile rejection;
- exact profile fingerprint mismatch;
- event validation against supplied profiles;
- governed policy rejection when the decision timestamp is outside the profile
  interval.
