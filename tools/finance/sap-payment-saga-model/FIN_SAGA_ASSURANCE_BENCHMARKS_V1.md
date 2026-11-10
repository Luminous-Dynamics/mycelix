# Finance Saga Assurance Benchmarks v1

**Status:** benchmark specification and target envelope; no performance qualification is claimed by this document. Safety thresholds are non-compensatory: throughput cannot offset a duplicate, unauthorized or incorrectly completed monetary effect.

## 1. The paradigm shift being measured

A payment should not be represented by one mutable status field or one provider response. The assurance object is a chain of independently checkable claims:

`immutable intent → owner-local effects → typed source evidence → replayable journal → owner-scoped checkpoint → independent reconciliation`

The claims remain separate:

- A request was accepted.
- A local source-chain transaction committed.
- A Treasury effect was committed by its owner.
- A source record and receipt are authentic and authorized.
- A provider/bank reports a settlement state.
- The required operation is reconciled as complete.

A timeout is not automatically failure or success. It is `Indeterminate` until the original effect identity is reconciled. A Payments checkpoint cannot attest for Treasury, and neither can attest for a bank.

## 2. Non-negotiable safety qualification

| Invariant | Acceptance target |
|---|---:|
| Duplicate debit, recipient credit or Treasury fee under retry/race/fault injection | **0** |
| Unauthorized or wrong-owner checkpoint accepted | **0** |
| Same idempotency key with changed immutable intent accepted | **0** |
| Operation marked complete with a required effect or receipt missing/unknown | **0** |
| Unexplained integer-unit conservation delta | **0** |
| Cross-language known-answer vectors that disagree | **0** |
| Unknown, unavailable, stale, revoked or conflicting evidence silently upgraded to verified/settled | **0** |
| Frozen positive/negative conformance cases with expected disposition | **100%** |
| Recovery using the original stable effect identity | **100%** |

Run at least 1,000,000 deterministic, seeded state-machine sequences before production consideration. Preserve failing seeds and minimized counterexamples as fixtures. The current Rust/Go tests and KATs are a starting corpus, not this qualification.

## 3. Fault-injection matrix

Inject a crash, timeout, lost response, duplicate delivery, source-chain-head movement or unavailable dependency at every boundary:

1. Claim creation and authorization.
2. Local debit/credit/Payment/Receipt atomic transaction.
3. Durable Treasury outbox creation.
4. Treasury dispatch before acknowledgement.
5. Treasury commit with lost acknowledgement.
6. Payment/Receipt indexing and link repair.
7. Event append and checkpoint creation.
8. Source ActionHash lookup and owner-authority resolution.
9. DHT partial visibility, delayed validation and competing checkpoint histories.
10. Provider idempotency expiry, regional routing and ambiguous callback delivery.

Required assertions: no new logical operation is created to escape an indeterminate prior attempt; no owner can advance another owner's frontier; replays do not mutate; compensations are new explicitly authorized events; every outbox item remains in exactly one observable state.

Run concurrency at 1, 10, 100 and 1,000 contenders for (a) the same sender/key/intent, (b) the same sender/key with changed intent, and (c) distinct senders. Same-intent attempts must converge on one operation identity; changed-intent attempts must conflict; distinct operations must not serialize unnecessarily.

## 4. Replay and checkpoint performance

### Dataset sizes

- Journal replay: 100, 1,000, 10,000, then 100,000 frames.
- Owner checkpoints: 1, 10, 100, 1,000 and 10,000 per owner.
- Recovery patterns: clean completion, repeated same-effect indeterminate/retry, long pending tail, multiple owners, and partial owner evidence.

### Required reporting

Capture at least 30 raw samples per point after warm-up. Publish p50/p95/p99, min/max, throughput, serialized bytes, allocations/operation, peak RSS, and the full raw samples. Record exact source SHA, fixture digests, OS, CPU model, toolchain, compiler flags, build profile and concurrency. Keep replay, hashing, signature verification, source-record retrieval, DHT visibility and external provider latency as separate measurements.

Initial scaling guardrail for pure in-memory replay: a tenfold increase in event count should not exceed a 15× p95 latency increase on the same controlled runner. This is a provisional regression budget, not a measured result or a substitute for a matched baseline. Report the actual curve; do not smooth or drop outliers.

Checkpoint verification must be decomposed into:
- canonical subject encoding and digest;
- native Holochain action/signature and action-author binding;
- owner grant and operation-scope authorization;
- ActionHash/entry-type/source-chain predecessor verification;
- DHT retrieval and partial-visibility reconciliation;
- complete offline archive requalification.

Do not present an aggregate “verification latency” without those component measurements. Network and DHT costs must not be hidden inside the cryptographic microbenchmark.

## 5. Industry baselines and fair comparison

### Conventional database + outbox

Build a matched baseline using a transactional database, a unique constraint over sender-scoped operation identity, immutable intent fingerprint, an outbox written in the same database transaction, a leased worker, bounded retries and reconciliation. Use the same synthetic workloads, effect semantics, fault schedule and hardware. Compare:
- duplicate/false-completion outcomes;
- recovery after lost acknowledgement and process restart;
- throughput and p50/p95/p99 end-to-end latency;
- write amplification, storage per operation and oldest-pending age;
- operator interventions per 10,000 operations.

Do not compare a Rust in-memory reducer against a networked provider and call the result an industry win. Compare like with like.

### Provider idempotency

Adyen documents idempotency keys scoped to a company account, a 7–14 day retention period, and a caveat that simultaneous regional endpoints do not cross-check key duplication. Therefore provider idempotency is an adapter-specific, time-bounded defense, not the global identity of a Mycelix effect. The local stable effect ID and reconciliation record must outlive the provider's key-retention window; before any new provider request after expiry, reconcile the original external effect or return an explicit unresolved state.

Reference: https://docs.adyen.com/development-resources/api-idempotency/

### Holochain transaction boundary

Holochain documents that writes within one zome-function transaction are atomic, while a concurrent transaction can cause a chain-head-moved error and be retried by the caller. This supports an atomic local write set, not atomicity across independent agents or external rails. Model Payments and Treasury as separate owners with separate frontiers and evidence.

References: https://developer.holochain.org/build/zome-functions/ and https://developer.holochain.org/build/validation/

### Credential interoperability

Use W3C Verifiable Credentials Data Model 2.0 and the applicable Data Integrity cryptosuite when portable credentials are needed. VCDM 2.0 is a Recommendation; VCDM 2.1 remains a Working Draft in the October 2026 publication history. Do not claim a custom Mycelix checkpoint digest is VC-conformant. The native same-DNA Holochain action remains the intended author binding; a VC wrapper is an interoperability/export profile, not a substitute for owner authorization or source-action verification.

References:
- https://www.w3.org/TR/vc-data-model/
- https://www.w3.org/TR/vc-data-integrity/
- https://www.w3.org/standards/history/vc-data-model-2.1/

## 6. How Mycelix should beat the baseline

The claim to earn is not “exactly once.” It is:

1. **Effect identity outlives provider idempotency.** Provider keys are bounded adapters; the internal operation/effect identity and immutable intent remain stable.
2. **Unknown is preserved.** An unavailable DHT dependency or ambiguous external response stays partial/indeterminate, rather than becoming false absence or success.
3. **Owners attest only to their own effects.** Payments cannot speak for Treasury; neither speaks for the bank.
4. **Replay is independently reproducible.** Rust and Go consume the same frozen vectors, while a third implementation independently verifies the published canonical bytes.
5. **Evidence has a claim ceiling.** Cryptographic integrity, authority, source-record validity, ledger booking, rail settlement and legal finality are separate dispositions.
6. **Recovery is observable and bounded.** Stable effects, leases, backoff, query-before-retry, dead-letter state and operator-visible age replace recursive or silent retries.

Do not say “better than SOTA” until the matched database/outbox baseline, named-provider sandbox, real multi-agent Holochain fault suite and independent review have been run on recorded exact heads. Publish wins and losses.

## 7. Qualification progression

- **Reference:** canonical Rust/Go subjects, deterministic replay and frozen hostile vectors.
- **Runtime:** actual native action author, owner grant, entry type, ActionHash and predecessor/frontier verification.
- **Distributed:** multi-agent Holochain tests with delayed/partial DHT visibility and competing checkpoints.
- **Operational:** autonomous bounded outbox recovery, process restart and operator alerting.
- **External:** provider-specific idempotency/query/callback/finality adapters and statement reconciliation.
- **Independent:** third-party conformance rerun, reproducible artifacts and a public benchmark report.

A workflow that is queued, skipped, uninitialized or run on a merge ref is not an exact-head pass.
