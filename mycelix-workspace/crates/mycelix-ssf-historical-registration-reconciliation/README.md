# SSF Historical Registration Reconciliation v0.1

This crate separates write-time registration trust from later historical reconciliation trust.

A current reconciler may attest an old unresolved attempt after verifier/policy rotation only when it retains the exact stable registration-store identity from the original manifest. Store-identity migration is intentionally excluded from v0.1.

Historical reconciliation freshness never refreshes the original candidate. The authority-eligibility ceiling remains `min(current reconciliation receipt, original candidate validity)`.

The result remains evidence-only: persisted historical reconciliation still requires explicit candidate-lineage rebind and fresh authority issuance before any state-install or effect power can exist.
