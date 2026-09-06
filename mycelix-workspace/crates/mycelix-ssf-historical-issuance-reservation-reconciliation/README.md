# SSF Historical Issuance Reservation Reconciliation v0.1

This crate lets a currently trusted same-identity reservation-store verifier reconcile an old exact issuance-reservation attempt after verifier generation or store-policy rotation.

The reconciliation surface is read-only. Store-identity substitution is rejected, and fresh historical evidence cannot extend the original authority-eligibility ceiling.

A historical `Reserved` result remains evidence only and cannot recreate the rebound registration lineage or issue authority by itself.
