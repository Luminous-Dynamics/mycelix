# AC-172 / AC-168 industry benchmark and engineering gap analysis v1

**Assessment date:** 2026-10-11  
**Scope:** Mycelix local SAP/TEND idempotency and Treasury outbox, plus the design boundary for external payment adapters  
**Status:** engineering comparison; not a compliance attestation or provider certification

## Executive assessment

The design direction is aligned with established payment-engineering practice: stable logical operation identity, payload binding, fail-closed handling of ambiguous outcomes, a durable outbox, idempotent consumers, and separate evidence for external lifecycle states.

The implementation should not yet be described as industry-equivalent or production-qualified. The strongest next improvements are runtime concurrency/failure qualification, operational recovery and reconciliation, complete provider capability profiles, and an audited external security/compliance boundary.

## Comparison with published industry contracts

| Area | Published practice | Mycelix implication |
|---|---|---|
| Request idempotency | Stripe compares parameters for a retained key and replays the first result once execution starts; keys may be pruned after at least 24 hours. | Persist the local logical operation durably, but separately track the external provider's key lifetime. A long-lived local key cannot extend a provider's deduplication window. |
| Key scope and retention | Adyen documents company-account scope, a 7–14 day key window, and no cross-region duplicate check when multiple regional endpoints are targeted. | Provider profiles must include account/merchant, endpoint and region scope; retention bound; concurrent-key behavior; and query/recovery capability. The adapter must not infer one global namespace. |
| Duplicate/concurrent requests | Adyen documents in-progress/duplicate conflict responses and a transient-error signal that affects whether a retry is appropriate. | Treat provider responses as typed outcomes. Do not collapse conflict, known rejection, transient/no-effect error, and unknown effect into one retry path. |
| Provider-specific duplicate references | Visa Direct for Card documents distinct errors for a conflicting Client Reference ID and amount/currency inconsistency against a previously processed payout. | Keep the original client reference and request fingerprint stable. A changed request under a claimed identity is a conflict requiring reconciliation or a new explicit intent, never silent key rotation. |
| Reliable cross-component delivery | The transactional-outbox pattern atomically records business state and pending work; delivery can be repeated, so consumers still need idempotent effect handling. | Mycelix's committed outbox + stable Treasury receipt is the right pattern. Delivery success, queue deletion and Payment completion must be recoverable across crashes. |
| Holochain write/dispatch boundary | A zome function's writes are atomic and transactional; post_commit runs after the actions are committed and must handle follow-up errors itself using the infallible callback contract. | The Payments callback must use the hdk_extern(infallible) ABI with no return value. External/local cross-zome dispatch remains a separate transaction and must be recovered from durable state. |
| External payment evidence | ISO 20022 definitions and external code sets are versioned; provider status codes acquire meaning only within the relevant message/product usage profile. | Pin message/API version, jurisdiction, endpoint, lifecycle mapping, finality basis, return/dispute contract, and policy snapshot per actual connection. |
| Card data security | PCI SSC states that sensitive authentication data (such as card verification codes and PIN/PIN blocks) must not be retained after authorization, even if encrypted. | Keep card credentials/PAN/SAD outside Mycelix receipts, corpus fixtures and general logs. Prefer PSP-hosted/tokenized collection and store only provider tokens/references that the contract permits. Do not claim PCI scope reduction without an actual scope assessment. |

Official references:
- Stripe, idempotent requests: https://docs.stripe.com/api/idempotent_requests
- Adyen, API idempotency: https://docs.adyen.com/development-resources/api-idempotency
- Visa Direct, error codes: https://developer.visa.com/capabilities/visa-direct-connect/error-codes
- AWS, transactional outbox pattern: https://aws.amazon.com/blogs/compute/implementing-the-transactional-outbox-pattern-with-amazon-eventbridge-pipes/
- Holochain, zome function transactions: https://developer.holochain.org/build/zome-functions/
- Holochain, callbacks and lifecycle hooks: https://developer.holochain.org/build/callbacks-and-lifecycle-hooks/
- ISO 20022, change requests and external code-set maintenance: https://www.iso20022.org/maintenance/catalogue-change-requests
- PCI SSC, sensitive authentication data FAQ: https://www.pcisecuritystandards.org/faqs/1533/

## Current strengths

1. **Stable local operation identity.** Sender + key derives the operation ID; a versioned request fingerprint binds money, recipient, currency, payment type and memo. Raw idempotency keys are not emitted as public IDs.
2. **Fail-closed replay behavior.** Exact retries resolve existing evidence; conflicting payloads and ambiguous/missing indexed records do not trigger another value-moving attempt.
3. **Durable Treasury work.** A fee obligation is stored as an outbox item alongside the source transaction. Treasury requires a matching committed effect and records a destination-scoped receipt before applying balance changes.
4. **Semantic honesty.** A fee-bearing Payment remains Pending until its Treasury receipt is confirmed, and external adapters are not represented as live or certified merely because the reference corpus passes.
5. **Mirrored source discipline.** Canonical and workspace copies of the edited Finance sources have identical Git blob SHAs at the current branch head.

## Gaps that matter most

### G1 — Integration qualification is still absent

Unit tests authored in the PR do not prove source-chain serialization, competing calls, DHT visibility, post-commit recovery, or the behavior when a nested Treasury call commits but Payments fails before clearing its outbox. These require Holochain/SweetConductor/Tryorama integration tests.

Minimum required scenarios:
- same sender/key concurrently with same payload;
- same sender/key concurrently with a changed payload;
- chain-head movement and retry of the original request;
- lost caller response after payment commit;
- Treasury receipt committed but Payments queue removal or status update fails;
- duplicate or ambiguous claim/payment/receipt indexes;
- two distinct effects racing to credit the same Treasury pool;
- legacy queue entries with no effect ID remain quarantined and visible.

### G2 — Retry work is durable, but recovery operations need explicit SLOs

The outbox is a recovery source of truth, not a continuously running delivery service by itself. post_commit is best-effort and can be skipped operationally if a process/cell is unavailable; failed items require another drain invocation. The hook dispatches only when a newly committed queue tag has retry_count = 0, so retry metadata commits do not recursively trigger immediate drains. Failed delivery cycles now persist attempt count, last-attempt time, next-attempt time, and bounded exponential backoff with deterministic per-effect jitter; drain calls skip items until their persisted due time. An explicit worker/operator trigger, bounded per-cycle work, oldest-pending-age visibility, error classification, and a dead-letter/reconciliation state are still needed. Avoid reporting a queue as healthy merely because an item is still stored.

A fee-bearing Payment must remain Pending while delivery is unresolved. Provide a read-only queue inspection surface and alert on oldest pending age, repeated delivery failures, orphaned claim/payment evidence, and mismatched Treasury receipts. Do not put raw provider error text or secrets into public evidence.

### G3 — retry_count must be meaningful

New items start at zero. A failed bounded delivery cycle advances the counter by the number of provider attempts and atomically persists the new counter plus last/next attempt timestamps. Backoff is capped at one hour, includes deterministic per-effect jitter, and is computed from the number of failed delivery cycles. Counter saturation does not create an identical queue record; attempt timestamps can still advance without duplicating the queue link. Regression tests cover counter monotonicity/saturation, backoff bounds and schedule creation. Successful-attempt history, structured last-error classes, and an autonomous recovery worker remain unimplemented.

### G4 — Provider idempotency needs a capability model

Every external adapter profile should bind the following, based on the exact provider/product contract:
- key transport and constraints (header/field, syntax, maximum length);
- key namespace (merchant/company account, endpoint, region, instrument);
- key retention and provider-side pruning behavior;
- same-key/same-payload behavior (replay response, return existing transaction, or other);
- same-key/changed-payload behavior;
- concurrent in-flight request behavior and provider error classification;
- query by client reference / provider transaction ID and supported search window;
- webhook authentication, replay protection, ordering/duplication behavior;
- refund, return, dispute, reversal and correction semantics;
- finality basis, settlement evidence and reconciliation source;
- rate limits, retry-after semantics, exponential backoff and circuit-breaker rules.

A generic supports_idempotency=true boolean is insufficient. If a required field is unknown, route admission should fail closed or require manual reconciliation.

### G5 — Retry policy must depend on effect certainty

Use at least four separate outcomes:
- **Definitive no-effect rejection:** retry only if the contract says the failure has no effect and the request remains valid.
- **Accepted / in progress:** query or consume authenticated provider events; do not resubmit as a new operation.
- **Transient explicitly no-effect failure:** retry the same provider key with documented backoff.
- **Unknown outcome:** mark Indeterminate and reconcile first. Do not issue a new logical operation just to make progress.

Timeouts, 5xx responses and network errors are not universally equivalent across providers. Map each exact provider/product error contract to one of these categories, and test the mapping.

### G6 — Money representation and accounting evidence should be typed

Keep integer minor/micro units for value movement. Bind each value to an instrument definition, currency exponent, scale, rounding policy and fee basis. Avoid f64 in monetary paths; compute proportional fees using checked integer/rational arithmetic and explicit rounding. Reconcile gross amount, net amount, fees and FX separately. Enforce conservation equations at each ledger boundary, rather than relying only on the aggregate Payment status.

### G7 — Security and compliance claims must remain bounded

Use hosted or tokenized collection for card data where possible. Prohibit PAN, card verification code, PIN/PIN block, bearer credentials, private keys and reusable authentication data from receipts and corpus artifacts. Store secret references/digests only where appropriate. Provider sandbox approval, production certification, licensing, PCI scope, AML/sanctions obligations and legal settlement finality are separate qualification gates; passing local tests does not establish them.

## Proposed provider capability record

A provider connection should have one frozen, versioned profile, not a name-only adapter. Example shape (all values must be sourced from that exact provider/product contract):

~~~json
{
  "profile_id": "provider-product-jurisdiction-api-version",
  "provider": "REQUIRED",
  "product": "REQUIRED",
  "jurisdiction": "REQUIRED",
  "api_or_message_version": "REQUIRED",
  "idempotency": {
    "transport": "REQUIRED",
    "namespace_scope": "REQUIRED",
    "key_constraints": "REQUIRED",
    "retention_seconds": "REQUIRED_OR_UNKNOWN",
    "same_key_same_payload": "REQUIRED",
    "same_key_changed_payload": "REQUIRED",
    "concurrent_request_behavior": "REQUIRED",
    "query_by_client_reference": "REQUIRED",
    "safe_retry_after_retention_expiry": false
  },
  "reconciliation": {
    "authoritative_source": "REQUIRED",
    "settlement_evidence": "REQUIRED",
    "return_and_reversal_contract": "REQUIRED",
    "dispute_contract": "REQUIRED"
  },
  "qualification": {
    "sandbox_evidence_digest": "REQUIRED_BEFORE_SANDBOX_PASS",
    "production_approval_ref": "REQUIRED_BEFORE_PRODUCTION"
  }
}
~~~

REQUIRED and UNKNOWN are schema/documentation sentinels, not valid production values. An unknown critical field must prevent a profile from being labelled production-qualified.

## Execution order

1. **Now:** enforce the correct Holochain post_commit ABI, test it in the static compatibility guard, and wire the guard into Finance CI.
2. **Next:** fix retry_count semantics; add durable attempt metadata and explicit operator/worker drain visibility without hiding stuck pending fees.
3. **Then:** run the Holochain integration matrix against real coordinator functions, including concurrency and commit/response-loss injection.
4. **Then:** introduce the typed provider capability profile and run a provider-free contract simulator against every adapter error/finality mapping.
5. **Only afterward:** qualify one named sandbox product end to end, with signed callbacks, query/recovery, return handling, reconciliation, and partner approval evidence.

## Claim ceiling

The current branch is an implementation candidate for local Mycelix SAP/TEND operation identity and Treasury compost delivery. It is not yet an industry-certified payment system, does not establish exactly-once external payment execution, and has not passed the Holochain concurrency suite or provider sandbox qualification.
