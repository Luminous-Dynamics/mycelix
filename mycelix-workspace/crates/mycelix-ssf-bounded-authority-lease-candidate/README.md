# SSF Bounded Authority Lease Candidate v0.1

This crate consumes one exact `CurrentlyRevalidatedReservedIssuanceV1` and produces a bounded **non-authoritative** lease candidate.

The candidate cannot widen scope, consequence, or lifetime. v0.1 has no verified scope-containment lattice, so scope must remain exactly equal to the approved adoption scope. Consequence and lifetime may only attenuate, and the requested lease must still be live under the conservative latest-plausible current time.

The candidate retains the full current-revalidated lineage by value, forbids delegation, and explicitly requires a later durable lease-issuance boundary. It cannot install state or execute effects.
