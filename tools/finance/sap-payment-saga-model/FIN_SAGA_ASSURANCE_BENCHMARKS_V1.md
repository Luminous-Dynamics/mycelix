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


## 8. Current implementation coverage snapshot (2026-11-10)

| Area | Current evidence | Still required |
|---|---|---|
| Typed effect evidence | Rust/Go canonical digest reference and shared known-answer vectors in #6020 | Exact-head qualification and native Holochain source-record verification |
| Journal replay | Rust/Go in-memory reducers, prefix reconstruction and 100/1,000/10,000-frame synthetic benchmarks in #6021 | Exact-head qualification; disk durability, conductor restart and DHT replay |
| Checkpoint subject | Rust/Go owner-scoped canonical subject, byte/digest KAT and mutation tests in #6041 | Native action-author binding, owner grant authorization, source-action/frontier resolution and competing-history policy |
| Receipt authenticity | Draft implementation #5755 | Real-agent/DHT qualification and Holochain 0.7 migration qualification |
| Autonomous recovery | Design/requirements in #4965 | Durable scheduler/lease/worker, bounded retry and queue-age operations |
| Provider/rail outcome | Adapter requirements and bounded provider profiles in #4957/#4958 | Named-provider sandbox, callback/query authentication and independently reconciled settlement |
| Database comparison | Defined here | Implement and run the matched transactional-outbox baseline |

This table is a status boundary, not a green dashboard. Draft PRs, queued workflows and reference-only tests do not qualify their pending rows.


## 8. Industry baseline extension: saga/outbox and durable workflow engines

The fair comparison must include more than provider idempotency:

### AWS saga and transactional-outbox patterns

AWS's Prescriptive Guidance describes sagas as sequences of local transactions with forward recovery and compensating transactions. It distinguishes choreography and orchestration, and calls out idempotency, observability, eventual consistency and isolation as operational concerns. Its transactional-outbox guidance addresses the database/message dual-write problem, but explicitly warns that duplicate messages can still occur; consumers must be idempotent.

References:
- [AWS saga patterns](https://docs.aws.amazon.com/prescriptive-guidance/latest/cloud-design-patterns/saga-patterns.html)
- [AWS saga choreography and its dual-write/idempotency considerations](https://docs.aws.amazon.com/prescriptive-guidance/latest/cloud-design-patterns/saga-choreography.html)
- [AWS transactional outbox pattern](https://docs.aws.amazon.com/prescriptive-guidance/latest/cloud-design-patterns/transactional-outbox.html)

This is the minimum operational baseline for recovery. Mycelix should not claim novelty for “saga”, “outbox”, retries, compensation or replay alone. It must demonstrate stronger owner/source evidence and cross-domain reconciliation while retaining the familiar outbox safety properties.

### Temporal durable execution and event-history replay

Temporal documents a complete ordered Workflow Event History as the source of truth for workflow reconstruction. On resume, workflow code is replayed against that event history rather than relying solely on a persisted in-memory snapshot.

Reference: [Temporal Workflow replay](https://docs.temporal.io/workflows).

This is a direct comparator for deterministic workflow replay and crash recovery. The current Mycelix Rust/Go replay reducer is a small in-memory reference model; it is not a Temporal replacement and has not been compared with Temporal under equivalent workloads. The target should be to compose with or complement durable execution where appropriate, not to assert that an eight/ten-thousand-event synthetic reducer beats a production workflow platform.

### What Mycelix must add beyond those baselines

1. **Evidence is owner-authenticated, not merely workflow-recorded.** Replay histories and outbox rows show what a workflow recorded; Mycelix must additionally bind each attestation to the correct owner-native Holochain action, actual action author, source entry type and authorized owner scope.
2. **Separate authorities remain separate.** Payments may attest its local transaction; Treasury attests its fee receipt; the bank/provider attests its rail outcome. No single orchestrator can promote another owner's missing evidence to success.
3. **Unknown remains explicit.** Partial DHT visibility, lost provider acknowledgements, stale or revoked credentials, and absent owner checkpoints remain unavailable/indeterminate until reconciled.
4. **Evidence outlives workflow and provider retention.** Immutable effect IDs and independently retained owner checkpoints must remain reconcilable after provider idempotency keys expire and workflow execution history is archived or compacted.
5. **Every assertion has a claim ceiling.** A valid journal, source-chain action, credential or provider message proves only the claim defined by that evidence class; it does not by itself prove cross-agent finality or legal settlement.

### Matched competitor matrix

| Capability | PostgreSQL + transactional outbox | AWS-style saga/outbox | Temporal durable workflow | Mycelix target (not yet qualified) |
|---|---|---|---|---|
| Local atomic transaction | Database transaction | Participant's local transaction | Activity/application transaction | Owner-local source-chain transaction |
| Retry identity | Unique DB key + consumer dedupe | Participant idempotency | Activity/workflow idempotency still required | Sender/key intent + stable per-owner effect ID |
| Recovery history | Outbox/status/event rows | Saga events/compensation | Ordered workflow event history | Typed journal replay + owner-scoped checkpoints |
| Cross-owner authorship | Application-specific | Service identities | Workflow/activity identities | Native owner action author + explicit authority grant |
| Unknown remote outcome | Reconcile/query adapter | Retry/compensate per participant | Activity retry/timeout policy | Explicit indeterminate + query original effect |
| External finality | Provider/bank-specific | Provider/bank-specific | Provider/bank-specific | Separately reconciled owner/rail evidence |
| Publicly reproducible evidence | Depends on implementation | Depends on implementation | Platform history plus application evidence | Canonical KATs, source refs, owner checkpoints and independent verifier |

Do not interpret the matrix as a claim that the first three platforms lack security controls. It identifies the specific Mycelix target that needs independent testing. A fair benchmark must compare the same local effects, workflow history retention, external idempotency profile, retry schedule, crash points, fault rates, storage durability and hardware.

### Additional benchmark targets

- Replay a matched Temporal workflow/event-history implementation and the Mycelix reducer at 100, 1k, 10k and 100k transitions; report startup/replay time, history bytes, cache effects, compaction/continue-as-new behavior, and deterministic nondeterminism failures separately.
- Compare AWS-style choreography and orchestration baselines with 2, 5, 10 and 25 participants. Measure operational complexity: number of retry/compensation branches, correlated failures, mean time to reconcile and operator interventions.
- For outbox consumers, inject duplicate and reordered delivery at 1×, 2×, 10× and burst fan-out; assert zero duplicate economic effects and retain all delivery attempts.
- Run archive/requalification tests after history compaction, workflow export and key rotation. An archived history must either requalify from retained evidence or explicitly return incomplete; missing history is never interpreted as a successful terminal state.

The new Mycelix claim is therefore not “better workflow engine” or “exactly-once payments.” It is **owner-authenticated, evidence-carrying recovery across independently controlled economic domains**. That claim is only earned after native Holochain, multi-agent, outbox, named-provider and external reconciliation tests pass.
