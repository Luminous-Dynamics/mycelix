# hearth-care-occurrence-admission

Pure admission theorem for converting recurrence evidence into a concrete Care occurrence candidate.

The theorem does not perform timezone calculation, Holochain reads/writes, assignment derivation, or completion logic. It requires those authorities to already be supplied as exact current snapshots and then proves they all refer to the same household obligation.

Admission requires:

- an A6.1 recurrence spec bound to the current recurrence-state reference;
- an A6.1 expansion receipt that validates against that exact spec;
- one exact recurrence instance key from that receipt;
- a current assignment snapshot for the same Care schedule;
- non-empty exact assignment-state and assignee identities.

A stale recurrence receipt is rejected even if its resolved UTC window is identical to the current revision. A stale assignment state is likewise not interchangeable merely because the assignee string happens to match.

The admitted result preserves exact recurrence engine/tzdb evidence, original/requested/effective local times, concrete UTC window, recurrence-state reference, assignment-state reference, and assignee. It is suitable as input to a later DHT materialization tranche; it is not itself execution authority.
