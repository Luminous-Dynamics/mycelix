# Hearth Care Occurrence Authority

Pure read-side theorem for classifying one concrete Care occurrence against its two independent authority channels:

- assignment-state evidence;
- recurrence/time evidence.

The classifier never chooses a latest record. Each channel is independently `Missing`, `Current`, `Stale`, or `Conflict`; an occurrence is authoritative only when both channels are `Current`.

This crate performs no Holochain I/O, timezone calculation, assignment mutation, recurrence expansion, or completion logic. It is intended to be reused by DHT coordinators, UI state, replay qualification, and migration tooling.
