# AC-172 — Sender-bound payment creation claims v1

**Status:** implementation candidate, awaiting Rust/workspace and Holochain concurrency qualification  
**Scope:** local Mycelix SAP/TEND `send_payment` operation identity. This is not an external-bank or card-network exactly-once guarantee.

## Problem found in source

Before this change, `send_payment` built its ID from `sender DID + sys_time().as_micros()`. A retry could generate a new ID and repeat the value-moving path after the caller lost the original response. `get_payment_record` also selected the first indexed link, which did not resolve an ambiguous logical operation safely.

The new contract is:

```text
(sender DID, client idempotency key)
  -> deterministic operation ID
  -> sender-authored immutable creation claim
  -> one request fingerprint
  -> one payment record returned on exact retry
```

## Input contract and client migration

`SendPaymentInput` and the public Finance wire type now include `idempotency_key: String`.

Example JSON:

```json
{
  "from_did": "did:mycelix:...",
  "to_did": "did:mycelix:...",
  "amount": 1250000,
  "currency": "SAP",
  "payment_type": "Direct",
  "memo": "invoice-42",
  "idempotency_key": "client-operation-uuid"
}
```

Use a fresh unpredictable identifier for each new logical payment. Reuse that exact identifier after timeouts, retries, or lost responses for the same operation. Do not reuse it for changed recipients, amounts, currencies, payment types, or memos. The identifier is not a secret and must not be treated as authorization.

For decoding compatibility, a missing field deserializes as an empty string and is rejected during preflight before financial writes. Old clients must be upgraded before calling this endpoint successfully.

## Coordinator path

1. Verify participant tier, sender/caller binding, and currency.
2. Validate the key (non-empty, at most 128 bytes, no control characters or surrounding whitespace).
3. Derive `payment-op:v1:<digest>` from a domain-separated hash of sender DID and key. The raw key is not written into the claim or Payment ID.
4. Serialize the request fields in a versioned, fixed-field-order representation and derive a request fingerprint. The fingerprint binds sender, recipient, amount, currency, payment type, and memo.
5. Search the deterministic claim index before rate limiting or any value-moving operation.
6. On exact match, resolve the existing Payment record and return its latest update. On mismatched payload, missing Payment evidence, invalid target type, or ambiguous index state, fail closed and do not repeat the potential effect.
7. If no claim or Payment exists for the operation ID, apply the rate limit, create the immutable sender-bound claim, and then enter the existing local SAP/TEND payment path.
8. The Payment ID uses the deterministic operation ID. Subsequent Payment updates cannot mutate that ID or the original request payload fields.

The claim index is a discovery mechanism. A count of mutable links is **not** treated as proof of uniqueness: multiple candidates are rejected as ambiguous, and the identity is established by the sender-scoped key/claim protocol. The create-link validator checks exact deterministic anchor, target entry type, and that the link author is the same agent that authored the claim.

## Defensive additions in the same path

- Request retries resolve before rate-limit bookkeeping so they do not count as a new logical payment.
- SAP amount-plus-fee uses `checked_add` rather than unchecked `u64` arithmetic.
- Payment fees and demurrage compost are written to a durable pending-compost outbox in the same local source-chain transaction as the source balance update. The payments zome's `post_commit` hook drains the outbox only after that source-chain transaction commits.
- Treasury compost delivery carries a stable effect ID. `receive_compost` checks the immutable receipt for that pool/effect before crediting the balance; a retry after a committed delivery returns current pool state rather than applying the amount again.
- Treasury receipt IDs bind the destination pool and effect ID. Reusing the key with changed amount/source is rejected. Legacy calls without an effect ID remain wire-compatible but do not obtain idempotent retry guarantees.
- SAP amount-plus-fee uses `checked_add` rather than unchecked `u64` arithmetic.
- Payment identity, parties, amount, fee, currency, payment type, memo, and creation time are immutable across later status updates.
- Both canonical and workspace copies are updated byte-for-byte together; entry-type and link-type variants are append-only to preserve existing indices.

## Verification added

Coordinator unit tests cover:
- deterministic and sender-scoped operation IDs;
- rejection of malformed/overlong keys;
- no raw idempotency key in the operation ID;
- fingerprint changes for amount, payee, currency, payment type, and memo;
- same key/same sender with changed payload retaining the same operation ID but changing the fingerprint;
- checked debit-total overflow.

Integrity unit tests cover:
- valid sender-authored creation claims;
- forged sender rejection;
- invalid operation ID/fingerprint/amount/currency rejection;
- immutable Payment identity/request fields across status updates.

## Failure policy and limitations

- An exact retry returns the original/latest Payment record and does not run debit/credit/fee operations again.
- Same key with changed request payload fails as an idempotency conflict.
- Claim present but Payment missing/unreadable, or any ambiguous discovery state, returns an error and does not try to execute again. This is intentionally fail-closed but currently requires operator/reconciliation work to resolve; this patch does not implement an orphan-claim recovery worker.
- New fee and demurrage compost items have stable effect IDs, and delivery is initiated from `post_commit` after the payment/balance transaction commits. If Treasury commits but the queue-removal transaction fails, the next drain resolves the same effect ID and must not credit twice.
- Pending-compost rows created by older code without an effect ID are left queued and logged for reconciliation instead of being automatically retried, because a prior attempt may already have committed despite losing its response.
- The coordinator's local source-chain/concurrent-call behavior must be tested with Holochain integration tests (e.g., same-agent competing calls, response loss after commit, and competing calls that observe a moved chain head). Unit tests alone cannot qualify that boundary.
- This affects local Mycelix SAP/TEND payment records only. It does not establish bank-side idempotency, Visa payout idempotency, cross-hApp atomicity, external finality, legal discharge, or production readiness.

## Local verification commands

```sh
cargo test -p payments
cargo test -p payments_integrity
cargo test -p finance-wire-types
```

Run these in both the canonical and workspace project locations where their manifests differ. Before merge, run the Holochain integration corpus for concurrent same-key submissions, sequential exact retry, conflicting payload reuse, lost response after commit, missing claim/payment evidence, and Treasury delivery response loss, idempotent receipt retry, and legacy outbox reconciliation.

**Environment note:** this change was authored through repository file operations without a local Rust checkout/toolchain available in this session. The source and mirrored-file consistency were inspected via repository read-back, but the Rust tests and Holochain integration tests have not been run here.
