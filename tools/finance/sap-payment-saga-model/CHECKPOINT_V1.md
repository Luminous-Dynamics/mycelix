# SAP payment journal owner-scoped checkpoint subject v1

Status: effects-disabled reference profile. This document defines canonical subject identity and local semantic checks. It is not a production Holochain checkpoint validator.

## Purpose

A replayed event hash chain detects frame mutation, but an unauthenticated tail anchor can be replaced together with the rewritten stream. This profile gives each economic owner a deterministic subject to commit as a native Holochain record/action:

- Payments attests only to its own local-payment source-chain frontier.
- Treasury attests only to its own fee-destination source-chain frontier.
- An aggregate operation view preserves both owner states and does not manufacture one global event order.

For a checkpoint stored in the same Holochain DNA, use the signed Holochain Create action as the authorship attestation. Do not add a second detached Ed25519 signature field to the entry. The integrity callback must still bind the action author to the issuer DID, check the action author's authorization for the exact owner domain and operation scope, and resolve/type-check the referenced owner-native source action. Those runtime checks are not implemented by this reference module.

## Canonical profile

Two versioned SHA-256 profiles are defined:

- Intent: `MXSAP-PAYMENT-INTENT-V1\0`
- Checkpoint subject: `MXSAP-JOURNAL-CHECKPOINT-V1\0`

All strings are exact UTF-8 bytes with a four-byte unsigned big-endian byte length. Empty strings and fields longer than 16 KiB are rejected. Integers are unsigned 64-bit big-endian. The profile performs no Unicode normalization. Digests must be exactly `sha256:` followed by 64 lowercase hexadecimal characters.

Intent fields, in order: sender DID, recipient DID, client idempotency key, payload fingerprint, principal minor units, fee minor units, currency.

Checkpoint subject fields, in order:

1. profile version byte (1);
2. owner byte (Payments = 1, Treasury = 2);
3. lifecycle byte (Current = 1, Supersedes = 2, Retracts = 3);
4. operation ID, recomputed from the immutable intent;
5. canonical intent digest, recomputed from the immutable intent;
6. owner-local event-log identity;
7. event count;
8. exact tail-event digest;
9. owner-local checkpoint sequence;
10. prior-checkpoint option tag (0 = absent, 1 = followed by digest);
11. source action-hash reference;
12. issuer DID;
13. verification-method identifier;
14. authority-profile digest;
15. issuance time in Unix microseconds;
16. superseded/retracted checkpoint option tag (0 = absent, 1 = followed by digest).

Sequence 1 has no prior checkpoint. Sequences greater than 1 must bind a valid prior-checkpoint digest. Current lifecycle cannot name a superseded checkpoint. Supersedes and Retracts must name the checkpoint they supersede or retract. A second owner cannot use the same owner tag, and each owner maintains its own checkpoint sequence.

The shared fixture is `tools/finance/sap-payment-saga-go-verifier/fixtures/sap-payment-journal-checkpoint-v1.kat`. Rust and Go independently compute the intent and subject digests from the same typed input.

## Security and claim boundary

The reference function validates canonical fields and their local relationships. It does not:

- verify that a source-action hash resolves to the exact Holochain Create action or entry;
- authenticate the actual action author or issuer DID;
- prove the issuer is authorized to attest Payments or Treasury;
- establish source-chain uniqueness, fork exclusion, DHT convergence or network-wide visibility;
- prove event history completeness beyond the supplied event count and tail binding;
- prove an account movement, Treasury credit, provider acceptance, bank settlement or legal finality.

A digest is an identity commitment, not an authorization token. A valid native Holochain action is still only evidence under the DNA's integrity rules; owner authorization and source-action binding must be independently checked by the eventual coordinator/integrity adapter. Unknown, stale, withheld, conflicted or unavailable owner state must remain partial/indeterminate.

## Next runtime qualification

1. Use the Finance receipt-authentication profile and DID-to-agent-key boundary; do not create a second identity scheme.
2. Validate the native Create action author against the claimed issuer and an owner-scoped authorization grant.
3. Resolve the exact source action and entry type; verify the referenced action is owned by the expected agent/owner and is in the declared source-chain frontier.
4. Add previous-checkpoint linkage, correction/retraction semantics and competing-history detection to a deterministic reducer.
5. Add two-agent Sweettests for Payments and Treasury, including wrong-owner, wrong-author, substituted source action, stale checkpoint, fork/rollback, missing owner checkpoint, and partial DHT visibility.
6. Preserve receipts/fixtures so an offline verifier can reconstruct the subject and its source dependencies without relying on a live DHT lookup.

## Benchmark plan

Safety criteria are non-compensatory: zero wrong-owner, wrong-operation, stale-frontier or malformed-subject acceptances across the frozen corpus. Cross-language vectors must match exactly. Measure subject encoding/hash latency separately from Holochain action validation, source-record resolution, DHT propagation and reconciliation. For 1, 10, 100, 1,000 and 10,000 owner checkpoints, report raw samples, p50/p95/p99, memory and bytes/checkpoint. Add witnessed-history equivocation and rollback detection as a separate distributed qualification, not as a local microbenchmark.
