# facility_capability

Evidence-scoped facility capability matching for Mycelix manufacturing.

This crate is a companion to the existing manufacturing machine records. It does not replace the current DHT ABI and does not treat `Machine.capabilities: Vec<String>` as verified engineering data.

Key rules:

- missing capability data remains `Unknown`;
- legacy free-form capability strings remain descriptive `Unknown` tags;
- categorical support/unsupported claims require acceptable evidence before they become `Satisfied`/`Unsatisfied`;
- stale or unaccepted evidence downgrades the affected constraint to `Unknown`;
- contradictory supported/unsupported assertions remain `Unknown` and require review;
- numeric capability limits are integer micrometre values to avoid floating-point ambiguity in the semantic boundary;
- matching all constraints under a profile is not manufacturing authorization and does not prove produced-part conformity.
