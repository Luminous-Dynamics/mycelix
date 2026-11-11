# Provider contract and proof-carrying retry profile V1

Status: effects-disabled contract parser/reference model. The Adyen profile is a public-documentation baseline only and is explicitly not production-qualified.

## Why this exists

An idempotency key is not a universal exactly-once abstraction. The provider owns its deduplication namespace, retention, regional topology and status vocabulary. A local payment coordinator must not silently widen those guarantees.

Mycelix turns provider behavior into a versioned contract subject. The exact contract digest is embedded into each provider operation key; a changed contract therefore changes the key's accepted scope.

## Paradigm shifts

1. **Provider idempotency as a versioned contract, not a boolean.** Model key scope, region scope, character limit, retention, concurrent in-progress behavior, changed-payload behavior, partial-refund support, refund ceiling and absence-query support explicitly.
2. **Retry decisions as evidence-carrying permits.** A retry is not an informal `true`; V1 returns a permit bound to provider, profile, API version, account, region, key, exact request digest, contract digest, operation-key digest, attempt class and exclusive expiry.
3. **Unknown is not absent.** Provider "no result", timeout and replica lag do not prove that no effect occurred. Absence has its own evidence type and is accepted only when a contract explicitly provides a complete lookup horizon.
4. **Accepted is not settled.** The provider request response may only mean accepted for processing. Provider webhook status, refund settlement and the beneficiary bank credit are different facts and can occur at different times.
5. **Provider refund ceiling is independent from owner authority and liquidity.** A provider may prevent refunds above captured amount, but that does not prove the owner authorized a particular refund or the funding account still has reservable liquidity.

## V1 provider contract fields

- Provider ID and profile ID.
- API version and provider account/merchant scope.
- Idempotency key namespace scope.
- Regional deduplication scope (global or per region).
- Maximum key character count.
- Whether POST idempotency is supported.
- Guaranteed minimum and documented upper retention bounds.
- In-progress duplicate retry semantics.
- Changed-payload semantics.
- Partial-refund policy: unsupported, supported, or payment-method-dependent.
- Refund ceiling rule.
- Authoritative-absence policy and visibility horizon.
- Profile maturity: documentation baseline, sandbox qualified, or production qualified.
- Webhook authentication profile.

The canonical contract digest uses a domain-separated binary encoding and fixed field order. Rust and Go share the same KATs for the contract, provider operation key, ordinary observation, dedicated absence subject and retry permit.

## Adyen documentation baseline

The baseline currently encodes these published facts:

| Provider fact | Modelled V1 value | Consequence |
| --- | --- | --- |
| Idempotency is supported on POST | enabled | Same-key retry may be considered while its guarantee remains live |
| Maximum key size | 64 characters | Reject longer keys; count Unicode characters, not UTF-8 bytes |
| Key uniqueness scope | company account | Account is in the immutable key scope |
| Validity | 7–14 days after first submission | Use 7 days as the conservative guaranteed-retention cutoff |
| Simultaneous regional endpoints | no cross-region duplicate checking | A key is bound to the original region |
| In-progress duplicate | retry only when `transient-error: true` | Header absent/false means no retry |
| Changed-payload behavior | unspecified | Keep exact request digest immutable |
| Multiple/partial refunds | payment-method-dependent | Provider-specific payment-method capability must be checked |
| Aggregate refund ceiling | total refund value does not exceed captured amount by default | Enforce provider capture ceiling alongside Mycelix plan ceiling |
| Refund initiation | only after capture; use cancel/reversal paths when not captured or capture state is unknown | A forward payment status must not be inferred from refund eligibility |
| Refund result | listen to REFUND / REFUND_FAILED / REFUNDED_REVERSED events | Acceptance response is not a final settlement receipt |

Sources (reviewed 2026-11-11):
- API idempotency: https://docs.adyen.com/development-resources/api-idempotency/
- Refunds: https://docs.adyen.com/online-payments/refund/
- Checkout refund endpoint: https://docs.adyen.com/api-explorer/Checkout/latest/post/payments/_paymentPspReference_/refunds

This is not a production Adyen profile. The API version and webhook signature verification profile remain unpinned, and the public documentation baseline does not claim authoritative lookup/absence semantics. `require_live_dispatch` rejects it. No live request should be created from this profile.

## Retry permit semantics

The retry permit is derived only after checking:

- contract digest and provider/account/profile/API match;
- original region is unchanged for per-region deduplication;
- exact canonical request digest is unchanged;
- current time has not moved backward;
- the minimum guaranteed idempotency retention has not elapsed;
- attempt class permits retry (transport timeout unknown, or provider's explicit transient in-progress signal).

The permit binds the full immutable operation scope and expires at:

`created_at + retention_minimum`

At that exact boundary, retry is rejected. The 7–14 day published range is not interpreted as a 14-day guarantee; V1 stops at day 7 because key presence after the minimum is uncertain.

The permit is not signed and is not a provider authorization. It is a local decision object for later scope revalidation. Production dispatch still requires a durable, owner-authorized intent, a provider-specific request signer, an authenticated provider response and a real funding-account reservation.

## Dedicated absence evidence

A regular provider observation cannot have status `AuthoritativelyAbsent`. This avoids treating a generic status update or a no-result query as proof of non-execution.

A provider may enable `CompleteLookupAfterHorizon` only when the public or contracted provider semantics define:
- the lookup namespace (account + region + exact key/request);
- completeness and visibility horizon;
- retention and replica consistency;
- a verifiable response/evidence mechanism;
- the maximum time at which a delayed request could still materialize.

The reference validator checks subject equality and that the lookup's `complete_through` is at least `created_at + horizon`. This is only a time/scope rule; it does not authenticate the lookup response. Until an adapter verifies the source and its signatures, the evidence cannot qualify a production retry.

## Status monotonicity

V1 distinguishes Unknown, InProgress, Accepted, Settled, Rejected, Returned and Reversed. A status cannot regress (for example, Settled → InProgress). A duplicate observation at the same status is idempotent only if its digest matches. Refund-reversal/return is represented as a new post-settlement condition, not as erasure of the original settlement event.

Status semantics are provider-specific. V1 is a conservative common reference vocabulary, not an assertion that every PSP exposes every state.

## Industry comparison

**Adyen:** already provides request idempotency, retained keys, and aggregate refund ceiling rules. Mycelix must not claim to replace these. The contribution here is representing the provider's scope, region, retention and transient retry constraints in the same evidence model as owner authority, funding liquidity and independent replay.

**Temporal:** recommends idempotent compensation activities and registering compensations before forward execution where side effects may occur before a result is returned. The Mycelix payment model is stricter at a different boundary: a cash compensation is a new effect and must not dispatch until the forward effect is proven settled and the compensating grant and funding reservation are verified. These approaches are complementary; they should not be collapsed into one generic Saga rollback rule.

**AWS Saga guidance:** notes eventual consistency, missing transaction isolation and stale-data risks from concurrent orchestration, recommending semantic locking. Mycelix's funding-account reservation and exact provider idempotency scope are intended to provide explicit business-level locks. The in-memory V1/V2 models do not establish a distributed lock or transaction isolation.

Official references:
- Adyen API idempotency: https://docs.adyen.com/development-resources/api-idempotency/
- Adyen refund semantics: https://docs.adyen.com/online-payments/refund/
- Temporal Saga pattern: https://github.com/temporalio/documentation/blob/main/docs/design-patterns/saga-pattern.mdx
- AWS Saga orchestration: https://docs.aws.amazon.com/prescriptive-guidance/latest/cloud-design-patterns/saga-orchestration.html

## Tests and shared KATs

The Rust and Go implementations share:
- contract digest;
- operation-key digest;
- provider-observation digest;
- dedicated-absence subject digest;
- proof-carrying retry-permit digest.

Adversarial cases include wrong account, wrong region, changed request digest, changed provider contract, retention boundary, backwards clock, Unicode key length, transient-error true/false, absent lookup policy, incomplete lookup horizon, changed evidence under same status, invalid terminal regression, tampered retry permit and permit expiry.

The Go package includes microbenchmarks for key validation, permit qualification, permit revalidation and parallel permit qualification. The Rust benchmark records 30 raw timing samples for the same operations. These are hashing/decision microbenchmarks, not end-to-end payment latency.

## Benchmark protocol and targets

Record exact commit SHA, Rust/Go version, OS/architecture, CPU model, build mode, fixture digest, raw samples and median/p95/p99.

Run:
- 1k / 10k / 100k / 1m provider operation keys;
- 1 / 2 / 4 / 8 / 16 / 32 / 64 concurrent callers;
- same key/same payload, same key/changed payload and wrong account/region;
- retry immediately before, exactly at and immediately after retention cutoff;
- duplicated/reordered observations and terminal reversals;
- 0 / 1 / 10 / 50% timeout, in-progress and delayed webhook fault injection.

Hard acceptance:
- zero cross-account or cross-region key reuse;
- zero payload mutation under an existing key;
- zero retries at/after the minimum guarantee cutoff;
- zero in-progress retry without the transient signal;
- zero unknown/no-result → authoritative-absence promotion;
- zero status regression or changed digest accepted as an exact replay;
- 100% Rust/Go canonical digest and disposition agreement.

Do not define universal latency goals or claim SOTA from these microbenchmarks. First publish matched same-hardware baselines and an adversarial correctness envelope, then optimize p95/p99 and cost per qualified retry.

## Claim ceiling and next stages

This model does not verify webhook HMAC signatures, mTLS, provider response signatures, provider account credentials, payment method eligibility at live time, an actual refund, bank settlement, account holds, legal finality or multi-agent Holochain serialization.

Next:
1. Pin Checkout API version and the exact account/region endpoints.
2. Add a verified webhook contract (HMAC key lifecycle, rotation, replay protection, event IDs, duplicate/reordered delivery).
3. Add provider lookup and absence only where an authoritative completeness contract is documented.
4. Add sandbox adapter tests without live funds.
5. Bind provider profile digest and permit into compensation V2 dispatch events.
6. Verify the funding-account hold and owner grant at the Holochain integrity boundary.
