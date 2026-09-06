# SSF Current Authority Revalidation v0.1

This crate proves that one exact single-use reserved issuance lineage is still eligible for a later bounded authority-issuance decision under **current** local security state.

v0.1 requires the freshly qualified current local state to equal the exact local state originally bound by the approved adoption lineage. It does not infer compatibility from a newer epoch, revocation generation, policy, or snapshot head.

All six current prerequisites—membership, identity key, revocation snapshot, policy, machine health, and scientific qualification—must contribute known bounded freshness.

Because generic `ValidityInputsV1` intentionally leaves its time units uninterpreted, this profile additionally requires a concrete verifier-bound `AuthorityTimeBasisReceiptV1` binding the exact reserved issuance and exact current-local-context receipt to `UnixMillisecondsUtc` before any freshness scalar is compared with trusted absolute time.

Trusted time carries explicit uncertainty. Authority uses the latest plausible current instant (`observed_unix_ms + uncertainty_ms`) and fails closed on overflow or expiry.

The resulting token is still pre-authority: it contains current revalidation and is eligible for a later bounded issuance boundary, but it cannot install state or execute effects.
