# FIN-EFFECT-002 — Claim Admission Authority, Uniqueness and Spendability

**Version:** 0.1  
**Status:** Research contract / authority-profile specification  
**Issue:** [#6593](https://github.com/Luminous-Dynamics/mycelix/issues/6593)  
**Composes:** FIN-EFFECT-001 / #6588, PCOE payment model / #6556, FIN-SAFE-020 / #6575, FIN-SAFE-021 / #6576  
**Date:** 24 November 2026

## Executive thesis

A durable operation ID identifies the obligation. It does not authorize a first effect, serialize competing claims, prove a complete observation view, or establish spendability.

FIN-EFFECT-001's pure kernel deliberately returns `NoBindingObserved` and `NoEffectObserved` as observations, **not an admission grant** and not mutation permissions. This contract defines the next boundary: **which authority may reserve a singleton effect role, under which fault assumptions, and what further evidence is required before that effect can be considered spendable.**

The design must never confuse:
- a stable identifier with an authorization;
- a certificate with a value mutation;
- a local commit with a remote owner's commit;
- a quorum observation with external settlement;
- an absent DHT link with proof of absence;
- durable workflow progress with exactly-once external side effects.

## 1. Four questions, four proofs

| Question | Required evidence | Does not prove |
|---|---|---|
| Identity | operation ID + immutable intent fingerprint | authority or uniqueness |
| Admission | valid claim under a named authority profile | effect committed |
| Effect | authenticated owner commit and predecessor binding | other owner legs committed |
| Spendability/finality | all profile-required claims, effects and conflict checks | external/legal settlement unless separately specified |

Every certificate and API response must state which row it establishes.

## 2. Profile A — single-writer owner-chain admission

A single effect owner may use its own authoritative source-chain transaction to bind the stable `(operation_id, effect_role, effect_owner)` identity to one intent fingerprint and local effect.

The candidate transaction binds:
- FIN-EFFECT-001 operation identity and exact immutable intent fingerprint;
- effect identity, role and authenticated owner;
- owner authority/key epoch and protocol version;
- exact predecessor and resulting action references;
- local effect facts and any same-chain outbox/receipt facts;
- schema and policy revision.

Expected concurrency behavior: same-owner same-chain requests race from one head; at most one transaction commits. The loser refreshes and resolves the existing claim or returns `Indeterminate`. A retry never generates a replacement identity.

This profile has a bounded claim ceiling:
- it relies on one authoritative owner history and the runtime's actual transaction boundary;
- it does not serialize independent owner chains;
- it does not prove that a restored/forked history is globally unique;
- if fork evidence is unavailable, stale or incomplete, affected claims remain provisional/non-spendable;
- local Holochain zome-call atomicity is not cross-cell atomicity or global consensus.

This profile should be preferred where one owner already has legitimate authority and a globally shared registry would add unnecessary coordination.

## 3. Profile B — Byzantine quorum witness admission

Research an optional independently operated claim registry for effect roles that need cross-replica uniqueness beyond one owner chain.

A candidate baseline is:
- (n=3f+1) authenticated witnesses;
- a certificate requires (q=2f+1) distinct eligible witnesses;
- each honest witness durably records at most one fingerprint per `(operation_id, effect_role, effect_owner, authority_epoch)`;
- a witness refuses to sign a second, conflicting fingerprint for that singleton claim;
- certificates bind the full claim and signer-set configuration.

### 3.1 Conditional quorum-intersection argument

For two quorums of size (2f+1) selected from (3f+1) members, their intersection has at least:

[
(2f+1)+(2f+1)-(3f+1)=f+1
]

If at most (f) witnesses are Byzantine, at least one intersecting witness is honest. If every honest witness durably refuses to certify a conflicting fingerprint for the same singleton claim, two conflicting certificates cannot both be formed.

This is an assumption-bound argument, not a product feature. It is invalidated by an incorrect signer set, duplicate identities, key compromise beyond (f), non-durable witness journals, unsafe epoch transitions, signing before durable admission, or a client accepting certificates outside the declared profile.

### 3.2 Required certificate binding

A candidate certificate binds:
- operation ID;
- immutable intent fingerprint;
- effect ID, role and owner;
- authority profile and protocol version;
- signer-set epoch and signer-set digest;
- canonical claim bytes and domain;
- per-witness monotonic sequence / journal reference;
- exact predecessor or authority checkpoint root;
- distinct authenticated signer identities and signatures;
- certificate digest and verification profile.

A certificate is an admission decision. The effect owner still authenticates it and commits its own effect under its own local rules. A witness must not mutate a payer, beneficiary, Treasury or Hearth account by assertion.

### 3.3 Reconfiguration and key lifecycle

No certificate may be accepted merely because its signature is valid under a stale signer set.

A signer-set transition must bind the old epoch, new epoch, old log/checkpoint root, new initial root and a transition barrier. The transition procedure must prevent the old and new sets from certifying incompatible histories for the same operation/effect namespace.

Model and test:
- old-set authorization closure;
- new-set admission activation;
- joint or otherwise justified transition evidence;
- witness key rotation and revocation;
- lost/replaced witness recovery;
- durable sequence continuity;
- stale certificate replay;
- equivocation and signed-conflict evidence.

Do not use wall-clock lease expiry as a conflict-resolution shortcut unless clock skew, monotonicity, revocation and partition assumptions are explicit and tested. Prefer durable sequence/epoch claims. No quorum means `Pending` / `Indeterminate`, not a weaker certificate threshold.

### 3.4 Fault assumptions

The theorem must name:
- maximum Byzantine witness count (f);
- authentication and identity-enrollment assumptions;
- network timing / synchrony assumptions for liveness;
- durable journal and recovery assumptions;
- signature/key compromise assumptions;
- signer-set transition protocol;
- what happens when the assumptions are exceeded.

At (f+1) Byzantine witnesses, the stated (f)-fault safety theorem no longer applies. Do not claim it still holds. The implementation must expose degraded/compromised status and freeze affected claims where detected.

## 4. Profile C — escrowed or partitioned capabilities

A third option is to avoid a global claim round for every operation by pre-allocating disjoint, owner-bound spending capabilities or value-note partitions.

A member, agent or service may spend only value/capabilities assigned to its partition. This can trade liquidity utilization, flexibility and rebalancing complexity for local serialization.

Required invariants:
- allocation partitions are disjoint and value-conserving;
- one capability/note cannot authorize two effects;
- allocations are bound to one authority epoch and owner;
- reallocation requires consuming/revoking the old capability under a qualified transition;
- stale partitions cannot spend after revocation;
- cloned/forked owners cannot make both histories spendable;
- unavailability causes a hold, not implicit reallocation.

Benchmark this against per-operation quorum claims. It may be cheaper and safer for predictable pools, but must not be represented as a universal solution.

## 5. Spendability is a separate state machine

Use a typed projection rather than a Boolean. The initial vocabulary is:

- `Unclaimed`: no admission evidence observed; never an admission grant.
- `ClaimPending`: request exists but the authority decision is incomplete.
- `Provisional`: a candidate or locally observed claim exists, but the declared finality profile is incomplete.
- `Certified`: the exact singleton claim is valid under the named authority profile.
- `EffectCommitted`: the authorized owner has committed its local effect.
- `Spendable`: all evidence required by the instrument/profile is complete and no unresolved conflict blocks use.
- `ConflictFrozen`: incompatible claims, forks, equivocation or lineage contradictions are observed.
- `Indeterminate`: authority, evidence completeness, runtime capability or history cannot be established.
- `RevokedForFutureUse`: the authority is revoked under a valid transition; historical effects are not erased.

`Certified` is not automatically `Spendable`. A payer claim can be certified while the beneficiary claim is still pending. A local owner effect can commit while Treasury remains outstanding. External provider acceptance remains distinct from external settlement and legal finality.

## 6. Formal-methods plan — extend PCOE, do not fork the theorem

Extend the existing PCOE two-owner TLA+ model from [PR #6556](https://github.com/Luminous-Dynamics/mycelix/pull/6556). Do not create an independent payment state machine that repeats payer, beneficiary and Treasury semantics.

Add state for:
- operation identity and intent fingerprint;
- singleton effect role and effect owner;
- authority profile;
- signer set, epoch, signer identities and Byzantine set;
- durable witness vote/journal state;
- claim certificate and predecessor/checkpoint root;
- owner local effect commit;
- stale/revoked authority epochs;
- network partition / quorum availability;
- fork and evidence completeness;
- provisional/certified/spendable projection.

Required safe invariants:
1. At most one certified fingerprint per singleton effect identity within one qualified authority history.
2. No local value effect commits without a valid admission decision under that profile.
3. Two conflicting claims never both become spendable under the declared fault assumptions.
4. Minority, stale, revoked or wrong-epoch certificates never authorize an effect.
5. Quorum loss may reduce liveness but never lowers the certificate threshold.
6. An honest witness never durably signs two conflicting fingerprints for one singleton claim.
7. Incomplete DHT/history observations never promote to spendable.
8. Owner forks or equivocation freeze affected spendability.
9. Epoch transition cannot allow conflicting old/new certificates for the same claim.
10. An unsafe mutant (minority acceptance, volatile vote loss, or overlapping epochs) violates its intended invariant.

The PCOE baseline currently has a real bounded two-owner model-check result, but its coverage map distinguishes bounded, partial and not-in-model scenarios. This contract adds a new authority dimension; it does not retroactively extend the previous proof.

## 7. Adversarial corpus and benchmark targets

The machine-readable v0.1 corpus is in [claim-authority-corpus.v0.1.json](../../mycelix-finance/benchmarks/finance/claim-authority-corpus.v0.1.json), with [Draft 2020-12 schema](../../mycelix-finance/benchmarks/finance/claim-authority-corpus.v0.1.schema.json).

Minimum safety targets (research goals, not measured results):
- 100 concurrent same-ID/same-fingerprint claims: zero duplicate certified singleton effects.
- 100 same-ID/changed-fingerprint races: 100% conflict/hold; zero second effect.
- zero conflicting spendable certificates across all modeled schedules with no more than (f) Byzantine witnesses.
- zero effects accepted from minority, stale, revoked or wrong-epoch certificates.
- zero promotions from incomplete views, unresolved forks or equivocation.
- quorum loss must return pending/indeterminate; never silently lower threshold.
- at (f+1) Byzantine witnesses, mark guarantee out-of-profile and freeze/detect where possible; do not advertise the (f)-fault theorem.

Measure safety and liveness separately:
- certificate p50/p95/p99 latency;
- throughput under healthy, partitioned and degraded quorum;
- percentage/time of operations held Indeterminate;
- time to detect witness equivocation;
- epoch-transition duration and recovery time;
- witness storage/journal growth;
- signature verification and network cost per qualified effect;
- capability partition utilization, stranded liquidity and reallocation time;
- operator minutes per 1,000 held operations.

A system that safely refuses spending during a partition can have lower availability than a system that guesses. Do not conceal the trade-off in one score. Compare performance only after the same safety assumptions and workloads pass.

## 8. Industry comparison — avoid false novelty claims

- **Stripe API v1/v2:** strong request idempotency and partial-retry behavior within documented scopes. Mycelix must preserve the same-key/same-intent retry behavior, then test durable obligation identity beyond provider replay windows and across separate owner effects.
- **Adyen:** company-account scoped keys with 7–14 day validity and documented regional scope limitations.
- **PayPal:** `PayPal-Request-Id` is retained for up to 45 days where the API supports it; retention must be checked per endpoint.
- **Temporal:** durable Workflow Event History preserves progress across failures; Activities are retried and may execute more than once, so external side effects still need idempotency. Durable orchestration is not a global effect-uniqueness certificate.
- **AWS transactional outbox:** couples local state and publication; duplicates remain possible and consumers need idempotent admission.
- **Holochain:** local source-chain transactions provide useful atomicity within their boundary; DHT observations are eventually consistent, and fork/countersigning capabilities must be qualified on the exact runtime.
- **Classical BFT:** quorum intersection provides safety only under a named fault bound, authenticated membership, durable non-equivocation and safe reconfiguration. A home-grown witness list is not automatically BFT.

Mycelix's credible differentiator is a pluggable, evidence-carrying authority profile with explicit assumptions, auditable conflict handling and owner-specific effect boundaries—not an unsupported claim to outperform all incumbent processors or consensus systems.

## 9. Exit criteria and claim ceiling

Promote a profile only when:
- its authority and fault assumptions are written and independently reviewed;
- the safe TLA model and intended unsafe mutants are checked at exact head;
- canonical certificate bytes and cryptographic verification are independently reviewed;
- concurrency, recovery, partition, equivocation and epoch-transition tests pass;
- real owner effects are authenticated and remain owner-authored;
- all unknown or incomplete states remain non-spendable;
- an exact runtime capability matrix exists for the deployed conductor/HDK/HDI;
- benchmark results include all failures, unknowns, sample sizes and cost/availability trade-offs.

This v0.1 contract does not claim Mycelix currently implements a Byzantine quorum, global uniqueness, production fork finality, cross-cell atomicity, countersigning-backed double-spend prevention, external settlement or legal finality.
