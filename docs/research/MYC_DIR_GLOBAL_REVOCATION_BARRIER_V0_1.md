# Mycelix Directory Fabric: Global Revocation Barrier and Effect Linearizability v0.1

**Profile:** MYC-DIR-GLOBAL-REVOCATION-BARRIER-0.1  
**Status:** research specification; preregistration only; not runtime-qualified  
**Tracking:** [MYC-DIR-REVOCATION-BARRIER-001 / #6476](https://github.com/Luminous-Dynamics/mycelix/issues/6476)  
**Composes with:** [PR #6443](https://github.com/Luminous-Dynamics/mycelix/pull/6443), [PostgreSQL qualification #6459](https://github.com/Luminous-Dynamics/mycelix/issues/6459), [queue-worker qualification #6463](https://github.com/Luminous-Dynamics/mycelix/issues/6463), [effect-surface inventory #6454](https://github.com/Luminous-Dynamics/mycelix/issues/6454)

## 1. Executive thesis

A broker acknowledgement proves a transport event. An authorization decision proves that a request was allowed under a particular authority snapshot and validity interval. A worker claim identifies the current execution claimant. None of those facts, independently or together, proves that every downstream resource owner has stopped accepting work authorized under an older resource state.

The next abstraction should therefore be a **Global Revocation Barrier (GRB)**: a provider-independent protocol that issues a strict revocation acknowledgement only after every independently executable effect owner in a frozen, verified route inventory has installed, committed, and durably read back its new fence.

The barrier is not a new message broker and is not a distributed transaction pretending to atomically commit across heterogeneous services. It is a monotonic coordinator protocol over owner-local atomic fences. Partial progress is permitted; a global acknowledgement is not.

The central claim is:

> After a strict barrier acknowledgement for revocation R, no command authorized under a generation preceding R can create a new protected effect at any effect owner covered by the acknowledged inventory.

That claim is meaningful only when every route to the protected effect is inventoried, every owner enforces its own fence at the mutation boundary, and the evidence includes the exact owner set and durable owner acknowledgements.

## 2. Terms and authority boundaries

- **Broker:** stores and delivers commands. It does not authorize a protected effect.
- **Worker:** attempts work under a current claim. It is not the authority for the resource generation.
- **Policy decision point (PDP):** evaluates an authorization request and issues a bounded decision receipt.
- **Policy enforcement point (PEP):** authenticates the decision and claim evidence, then invokes the owner interface. It must not bypass the owner fence.
- **Effect owner:** the final independently executable system that can commit the protected mutation. Examples include a PostgreSQL database, payment service, cloud control plane or device controller.
- **Effect route:** an independently executable path to an effect owner. A queue consumer, direct API, scheduled job, admin path and replay processor may be different routes to the same owner.
- **Owner fence:** an owner-local monotonic generation checked atomically with the protected mutation.
- **Owner acknowledgement:** evidence that one exact owner committed and durably read back the requested fence generation.
- **Global barrier acknowledgement:** evidence that every required owner in one frozen inventory has acknowledged the same revocation and that the inventory remained current at finalization.

The broker, PDP, PEP, worker, queue offset, publisher confirm, consumer acknowledgement, message ID, delivery count and CAEP/SSF event are not substitutes for the effect owner's atomic mutation check.

## 3. Why the broker layer is insufficient

The transport products solve different, valuable problems:

- NATS JetStream supports persistent streams and at-least-once consumer delivery. Its publisher de-duplication uses the client-supplied Nats-Msg-Id within a configured duplicate window; the message ID, not the body, is compared. The window is transport state and cannot replace durable effect-owner idempotency.
- RabbitMQ quorum queues support replicated queue state, publisher confirms and manual consumer acknowledgements. At-least-once dead-letter transfer can produce duplicates when target confirmation is unavailable and retried.
- Kafka transactions atomically coordinate Kafka-produced records and consumed offsets. The Kafka design documentation explicitly distinguishes this from writes to an external destination; stronger semantics there require cooperation from the destination.

Therefore a publish acknowledgement, queue acknowledgement, offset commit or broker-side de-duplication result is not evidence that the protected external effect occurred exactly once, remains authorized, or is impossible after revocation.

Mycelix should retain broker-native reliability and compete on the stronger owner-level property rather than trying to replace broker throughput or replication.

## 4. Global barrier protocol

### 4.1 Freeze the scope

Construct a verified effect-surface inventory for the exact realm/resource/action and revocation digest. It must bind:

- inventory epoch and topology snapshot;
- canonical route-set and manifest digests;
- all independently executable routes;
- the effect owner for each route;
- each route's enforcement mode;
- the expected owner set;
- resolver/verifier identity and build;
- the closure proof commitment and verified-complete result.

A missing, unknown or newly added route is not silently ignored. An inventory change during barrier execution invalidates the pending closure. Route admission must itself be gated: a new effect route cannot become executable until it has installed the current owner generation and appears in a newly verified inventory.

### 4.2 Install each owner-local fence

For every owner, request an idempotent installation identified by the revocation digest and exact scope. The owner atomically advances its generation and returns a receipt binding at least:

- revocation digest and exact resource/action/owner scope;
- previous and installed generation;
- owner identity and owner epoch;
- inventory and route-set digest;
- durable commit identifier or equivalent state commitment;
- durability/replication profile;
- post-commit read-back result;
- verifier identity and authenticated receipt.

The owner generation is monotonic. A repeated request for the same revocation returns the same installed generation. An older or substituted revocation cannot lower the generation.

### 4.3 Define the local linearization point

The owner must serialize protected mutations against fence installation. For a PostgreSQL reference implementation, a mutation acquires a shared row lock on the exact fence record and holds it through commit. Fence installation acquires a conflicting lock and holds it through commit.

There are two valid orders:

1. **Effect lock wins:** the old-generation transaction already holds the shared fence lock. Fence installation waits. The effect may commit before the fence install commits. Only then may the owner acknowledge the newer generation.
2. **Fence lock wins:** fence installation commits the newer generation first. A delayed old-generation effect waits, reads the updated row under the selected isolation contract, compares its supplied generation and rejects before mutation.

The owner acknowledgement must be emitted only after the fence transaction commits and a durable read-back confirms the exact installed generation. Function return, lock acquisition, broker notification or an uncommitted row update is insufficient.

The reference PostgreSQL path should pin its tested server version, isolation level, replication mode, pooler and failover contract. The current reference uses READ COMMITTED and row-level FOR SHARE/FOR UPDATE locks; behavior at other isolation levels must be tested and documented rather than assumed.

### 4.4 Finalize the global barrier

The coordinator may issue the global acknowledgement only when all of the following hold:

1. The current topology epoch and inventory digest still match the frozen inventory.
2. The expected owner set exactly matches the set of verified owner acknowledgements.
3. Every acknowledgement matches the same revocation digest, scope, inventory digest and owner epoch.
4. Every owner has committed its fence and completed durable read-back.
5. No owner or route is best-effort, non-fenceable, missing, unknown or awaiting reconciliation.
6. The coordinator's own barrier record is durably committed.

The global receipt should bind the ordered owner-acknowledgement digest set, not merely a count or quorum. A quorum is insufficient for strict revocation when any omitted owner can still mutate the protected resource.

A partial barrier is PENDING. It is never described as globally revoked. The coordinator may crash and resume by retrieving exact owner receipts and safely retrying idempotent installs. Previously fenced owners remain fenced; the protocol does not roll them back to make the barrier appear atomic.

### 4.5 Handle outcomes that remain unknown

Unknown is not no-effect. An effect attempt with an ambiguous outcome blocks a new physical attempt until the exact attempt is reconciled.

There is an important distinction between an unknown client response and an unfenced delayed external effect:

- **Owner-local atomic transaction:** if the effect transaction already holds the same owner fence lock, fence installation waits for that transaction to commit or roll back. The owner can acknowledge only after the exclusive fence installation commits and the new generation is durably read back. A lost client response still requires exact reconciliation before retry, but it does not leave a pre-fence transaction capable of committing after the owner acknowledgement.
- **External/non-atomic call:** if a worker can send a remote request outside the transaction that installs the local fence, the remote effect may arrive after the local acknowledgement. Unknown remote outcomes therefore block strict global completion unless the receiver independently enforces the same monotonic owner-fence rule at its own mutation boundary, or an independently trusted reconciliation proves the exact attempt terminal and prevents a later effect.

A local database transaction plus an outbox does not fence a later HTTP request, payment rail or device command. The receiving system must participate by checking its own monotonic fence and idempotency key atomically with its mutation. Otherwise the route is explicitly best-effort and strict global completion is unavailable.

For a strict barrier, a remote sink that cannot enforce the owner fence itself cannot be included as a strong owner. A local database transaction plus an outbox does not fence a later HTTP request, payment rail or device command. The receiving system must participate by checking its own monotonic fence and idempotency key at its own mutation boundary. Otherwise the route is explicitly best-effort and strict global completion is unavailable.

## 5. Owner epochs, restart and durability

A generation alone is insufficient if a database restore or failover can roll it back. The owner fencing coordinate should be treated as a tuple:

**(owner epoch, resource/action generation)**

The owner epoch is governed by the owner's durable recovery/failover protocol and must never be reused for a stale primary or stale restored snapshot. Generation advances within that epoch. If the owner cannot prove that the recovered state is at least as new as the highest acknowledged state, it must fail closed: no protected writes and no new owner acknowledgement until the high-water state is reconciled.

For PostgreSQL, local WAL commit configuration, synchronous replication requirements, eligible failover nodes, backup/restore recovery and primary fencing are part of the owner contract. An acknowledgement that can be lost on promotion is not a durable revocation acknowledgement.

The full semantics must also cover split-brain. A former primary that can still accept writes is an independent effect owner until it is fenced from the write path. A database epoch label alone does not prevent stale-primary writes unless the actual write authority enforces it.

## 6. Time and validity

Decision expiry and owner fencing solve different problems. A decision is usable only within its signed validity interval under an authenticated clock profile and conservative uncertainty bound. A resource generation is checked atomically at the mutation boundary even when a decision remains time-valid.

The SQL reference uses the database clock and a configured maximum uncertainty. That configuration is not, by itself, proof that the host clock is healthy or that its uncertainty bound is true. Runtime qualification must include clock rollback, step adjustments, pause/resume, clock-profile mismatch, time-source loss, restart and failover. If clock health or its bound is unknown, new mutations fail closed.

Exact idempotent replay of a prior committed operation is read-only and can remain available after decision expiry to resolve a lost response. It must never authorize a new physical effect.

## 7. Formal invariants

The formal model and runtime oracle must test at least:

- **No stale post-ack effect:** after a global barrier acknowledgement, no earlier-generation protected mutation commits at any owner in the acknowledged inventory.
- **No stale post-owner-ack effect:** after an owner acknowledgement, that owner rejects every old-generation mutation.
- **Owner-set equality:** global acknowledgement implies exact equality between the frozen required owner set and the verified owner-ack set.
- **Durable acknowledgement:** owner acknowledgement implies committed fence state and successful post-commit read-back under the declared durability profile.
- **Monotonic owner state:** owner epoch/generation never regresses across duplicate revocations, restart, failover, restore or concurrent installs.
- **In-flight order:** an effect already holding the owner fence lock either commits before fence acknowledgement or is rejected; it cannot commit afterward.
- **No route bypass:** direct SQL, alternate APIs, schedulers, replay tools and administrative paths cannot mutate without the owner fence.
- **No false no-effect:** Unknown is never converted to ConfirmedNoEffect without exact independent reconciliation.
- **No strict claim for weak sinks:** any best-effort or non-fenceable route blocks strict global acknowledgement.
- **Topology closure:** any route-set/topology epoch change invalidates the pending inventory closure or is prevented from becoming executable before inheriting the current fence.
- **Revocation idempotency:** replay of the same revocation returns the exact prior generation/receipt; out-of-order revocations cannot regress state.
- **Evidence binding:** each receipt binds the revocation, scope, inventory, owner epoch, generation, durability mode and exact predecessor state.

The model must represent the lock acquisition/commit point, not only a boolean "fence is current". It must allow an effect to acquire the shared owner lock before revocation, hold it while the revoker waits, commit, and only then let the owner acknowledge. The reverse ordering must reject the stale effect. This avoids both an unsafe model and an unnecessarily restrictive one that rejects all in-flight work without representing the correct serialization order.

### Current finite-state model source and scope

The first finite-state model is [DirectoryFabricGlobalRevocationBarrier.tla](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricGlobalRevocationBarrier.tla). Its safe configuration is [DirectoryFabricGlobalRevocationBarrier.cfg](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricGlobalRevocationBarrier.cfg). Four negative-control configurations independently enable [stale owner-fence acceptance](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricGlobalRevocationBarrier_no_owner_fence.cfg), [acknowledgement before durable read-back](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricGlobalRevocationBarrier_ack_before_readback.cfg), [missing-owner global acknowledgement](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricGlobalRevocationBarrier_missing_owner.cfg) and [topology-drift bypass](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricGlobalRevocationBarrier_topology_drift.cfg).

The model represents per-owner in-flight work, fence-lock acquisition, fence commit, post-commit read-back, owner acknowledgement, worker claim epochs, frozen topology epoch and global acknowledgement. It models an effect that entered before revocation as draining before the exclusive fence lock can be acquired. A new stale-generation effect after fence commit is rejected in the safe configuration.

The current finite-state model is intentionally narrower than the full specification. It does **not yet** model cryptographic receipt verification, real broker/client behavior, clock uncertainty, owner-epoch/failover/backup-restore recovery, or remote unknown-outcome reconciliation. Unknown-attempt handling remains separately represented in [DirectoryFabricQueueWorker.tla](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricQueueWorker.tla). Owner-epoch recovery and split-brain fencing are explicit next formal extensions, not already-proven properties of this model. The model, safe configuration and negative controls are source-present only; TLC has **not** been executed.

## 8. Benchmark design

The machine-readable companion corpus is MYC_DIR_REVOCATION_BARRIER_V0_1.json. It is synthetic and preregistered; none of its outcomes are measured.

### 8.1 Workload matrix

- Effect owners: 1, 4, 16, 64 and 256.
- Concurrent owner transactions: 0, 1, 32, 256 and 1,024 per owner.
- Queue backlog: 1,000; 100,000; 1,000,000; 10,000,000 commands.
- Offered rate: 100, 10,000 and 100,000 messages/second.
- Workers: 1, 32 and 256.
- Command age at delivery: 0 seconds, 1 minute, 1 hour and 24 hours.
- Fault schedule: owner/network partition, worker pause/resume, broker redelivery, lost acknowledgement, coordinator crash, DB crash, leader promotion, restore, clock drift and topology mutation.

### 8.2 Correctness gates

These are hard gates, not optimization targets:

- stale effects after any owner acknowledgement: 0;
- stale effects after global acknowledgement: 0;
- incomplete owner-set acknowledgements: 0;
- duplicate logical effects: 0;
- owner epoch/generation regressions: 0;
- false no-effect classifications: 0;
- successful bypass mutations: 0;
- global acknowledgements with an unknown/non-fenceable route: 0.

A single safety violation fails the configuration. Averaging across runs is prohibited.

### 8.3 Performance and recovery measurements

Record per-run raw samples for:

- owner fence-install latency;
- global barrier p50/p95/p99/max;
- time from revocation request to final owner acknowledgement;
- time spent waiting for pre-existing in-flight transactions;
- shared/exclusive fence lock wait and contention;
- number of effects completed before the barrier and rejected after it;
- time to resume a partial barrier after coordinator/owner restart;
- queue age, backlog drain rate and redelivery rate;
- CPU, memory, storage/WAL and network cost;
- owner count, owner health, topology digest and failure schedule.

Use at least ten independent open-loop repetitions per measured configuration. Publish warm/cold profiles, offered and accepted load, all raw samples, full version/configuration digests, and fault event timelines. Report both median and tail values; do not hide timeouts or failed barriers from the denominator.

The existing provisional latency objectives (including 250 microseconds for pure reference binding, 5 milliseconds delivery-to-owner revalidation, and 5 seconds p95 / 30 seconds p99 healthy-network revocation completion) remain hypotheses. Global barrier latency must be measured separately from local binding and queue delivery. Do not market them as achieved performance.

## 9. Fair industry comparison

Use the same message payload, resource mutation, replication/durability class, worker count, backlog, owner topology and fault schedule. Preserve each product's recommended configuration and report the exact settings.

| Comparator | Strong native boundary | What it does not establish by itself |
|---|---|---|
| NATS JetStream | Persistent streams, consumer state, ack/redelivery, message-ID de-duplication within a configured window | Current authorization or one external effect after revocation |
| RabbitMQ quorum queues | Replicated queue, publisher confirms, manual consumer acknowledgement, optional at-least-once DLX | Exactly one downstream mutation or a global owner fence |
| Apache Kafka | Atomic Kafka output records and consumer offsets under Kafka transactions | An arbitrary external database/API mutation without destination cooperation |
| PostgreSQL row-lock fence | Atomic generation check plus local protected mutation under a shared/exclusive lock protocol | Other databases, payment rails, HTTP APIs or device routes |
| Lease-only worker coordination | Worker ownership/lease and re-assignment semantics | A paused former worker being unable to mutate an unfenced resource |
| CAEP/SSF event propagation | Interoperable security-event signaling between transmitters and receivers | Atomic owner-local fence commit or global route-set completeness |

Do not report broker-only throughput as an end-to-end Mycelix result. Do not compare ephemeral transport against replicated durable transport. Do not count a queue acknowledgement as an effect. The external destination must be instrumented and independently queried by the correctness oracle.

## 10. Standards relationship

CAEP/SSF should be used as interoperable signal transport and a revocation trigger, not as the owner-fencing primitive. CAEP 1.0 is the stable profile baseline. The CAEP 1.1 working-group draft published in November 2026 is a forward-looking interoperability target and must be labelled draft. Neither event delivery nor receiver processing should be equated with a completed owner fence.

The adapter should carry a stable revocation identity through signal, owner installation, owner acknowledgement and global barrier receipt. Duplicate or reordered signals must be harmless and monotonic.

## 11. Implementation sequence

1. **Contract and model:** formalize owner lock order, in-flight effect ordering, owner epoch, barrier state and exact owner-set matching. Add safe and intentionally broken model configurations.
2. **PostgreSQL qualification:** compile both migrations on a pinned server; run all SQL self-tests; execute two-session effect-first/revocation-first races; exercise claim reassignment; test role bypass, clock faults, restart, failover and stale restore.
3. **NATS adapter:** implement one real JetStream worker path with explicit ack, double-ack where appropriate, AckWait redelivery, duplicate publication outside the dedup window, and worker pause/resume. Keep the resource owner authoritative.
4. **Global coordinator harness:** add multiple heterogeneous owner instances, partial failure, coordinator restart, exact owner acknowledgement matching, topology drift and global receipt verification.
5. **RabbitMQ and Kafka adapters:** test quorum leader change, at-least-once dead-letter duplicates, Kafka rebalance and external database commit/offset ambiguity.
6. **Independent oracle:** reconstruct histories from owner commits, generation changes, worker claims, broker delivery/ack traces and acknowledgements. Validate the linearization order independently of the implementation under test.
7. **Public results:** publish source SHA, image/configuration digests, raw histories, failed schedules, checker results and reproducible scripts. Mark every unexecuted gate as not run.

## 12. Explicit non-claims

This specification does not establish:
- a novel broker or a replacement for NATS, RabbitMQ or Kafka;
- an atomic heterogeneous distributed transaction;
- a production-ready multi-owner coordinator;
- a cryptographic trust root or signature verifier;
- completeness of the existing route inventory;
- PostgreSQL, broker, failover or TLA+ runtime qualification;
- zero violations in any real workload;
- superiority over industry SOTA.

The research contribution is the composable, independently checkable end-to-end contract: exact authority scope, owner-local linearization, monotonic generation, full route coverage, durable acknowledgements and a global barrier that cannot complete on missing evidence.
