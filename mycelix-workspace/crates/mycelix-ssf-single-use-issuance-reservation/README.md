# SSF Single-Use Issuance Reservation v0.1

This crate durably consumes an exact rebound persisted registration for at most one future authority-issuance lineage.

It enforces both attempt-ID uniqueness and uniqueness by persisted registration record. Ambiguous outcomes freeze the exact reservation attempt and may only be reconciled through the same read-only attempt surface.

A successful reservation is still not authority. It carries no current membership, revocation, policy, identity-key, machine-health, or time revalidation and cannot install state or execute effects.
