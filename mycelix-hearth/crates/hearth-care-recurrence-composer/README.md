# hearth-care-recurrence-composer

Authoritative bounded composition for Hearth Care recurrence.

This crate is the only layer in the A6 stack that composes all three recurrence authorities:

1. `hearth-care-recurrence` — typed household intent/evidence contract;
2. `hearth-care-recurrence-enumerator` — deterministic original local recurrence identities;
3. `hearth-care-timezone-jiff` — exact pinned bundled-tzdb civil-time resolution.

The composer derives its own local scan window from the requested UTC interval, applies exclusions/one-off overrides, resolves each requested local time through the pinned timezone adapter, and emits a validated `ExpansionReceipt`.

It never truncates an authoritative result to satisfy `max_instances`. It fails closed if the result exceeds the requested cap. Operational recurrence exceptions are bounded before validation, and one-off moves can bring an occurrence into the UTC window even when the original recurrence date is outside the derived scan range.

This crate performs no assignment, Care mutation, DHT write, or completion logic. Its output is still only recurrence/time evidence for later assignment-aware materialization.
