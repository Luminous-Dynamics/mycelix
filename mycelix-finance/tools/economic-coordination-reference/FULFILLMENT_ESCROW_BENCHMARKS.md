# Escrowed partial-fulfilment experiment: adversarial benchmarks

**Status:** proposed benchmark contract; no benchmark result is claimed.  
**Scope:** the experimental Rust `fulfillment_escrow` module only.  
**Authority ceiling:** in-memory reference accounting; no production capacity, economic, or settlement authority.

## Hypothesis

Disjoint, preallocated quantity rights let each replica reserve locally during a network partition without globally over-allocating a finite request. This trades dynamic utilization for bounded offline autonomy. It does not imply that rights can be transferred safely while disconnected.

## Safety gates (hard pass/fail)

| Gate | Proposed campaign | Acceptance target |
|---|---:|---|
| Quantity conservation | 100,000 seeded traces × 64 transitions | Zero states where available + pending + accepted differs from requested quantity |
| Global accepted bound | Same traces | Zero states where accepted quantity exceeds request quantity |
| Local authorization | 100,000 claim attempts across 2–32 replicas | Zero reservations exceeding that replica's available escrow rights |
| Transfer conservation | 100,000 transfer attempts, including overflow/boundary values | Zero rights created, duplicated, or lost |
| Replay resistance | 10,000 repeated claim IDs and 10,000 repeated transfer IDs | Zero replay accepted in one authoritative history |
| Finalization | 10,000 invalid/finalized replay attempts | Zero accepted quantity changed by invalid or repeated finalization |
| Unknown outcome | Simulated lost replies and delayed finalization | Pending rights remain reserved; timeout alone never releases rights |
| Deterministic replay | 100 identical operation traces | Identical state and ordered journal entries |

A failure of any hard invariant is a stop condition. Never average a safety failure into throughput, fairness, or utilization scores.

## Distributed counterexamples required before a protocol claim

The serial model cannot validate these claims. A future replicated prototype must explicitly test and publish minimized counterexamples for:

1. Two replicas receiving the same allocation after a fork or stale snapshot.
2. A rights transfer acknowledged by the sender but not observed by the receiver.
3. A transfer replay after recovery or replica restart.
4. Duplicate, reordered, and delayed reservation/finalization messages.
5. Partition healing with divergent journals and concurrent local reservations.
6. A verifier or coordinator becoming Byzantine, unavailable, or equivocating.
7. A node restoring from a stale backup and attempting to reuse spent rights.

The implementation should fail closed on ambiguous transfer ownership; it must not “fix” divergence by inventing capacity. Tests must distinguish a hard guarantee from eventual reconciliation.

## Utilization and liveness (reported separately from safety)

Run common-seed comparisons against a serial global-reservation baseline, fixed escrow with no transfer, and escrow with transfers only after connectivity returns. Vary replica count (2, 4, 8, 16, 32), request-size skew, demand locality, partition duration, and failure rate.

Report:
- accepted quantity / requested quantity;
- unserved demand while another replica holds idle rights;
- stranded-rights ratio at partition end and after recovery;
- p50/p95 reservation latency and reconciliation latency;
- rights-transfer frequency and bytes/messages per accepted unit;
- tail latency and rejected claims by replica, not only global means;
- every hard-invariant violation and minimized counterexample.

Provisional *research* targets after safety gates pass: zero invariant violations; at least 90% of feasible demand served within 10 reconciliation rounds after connectivity returns; and median reservation latency no more than 1.5× the global baseline during connected operation. These are falsifiable starting hypotheses, not established performance facts; publish baseline measurements before ratifying thresholds.

## Measurement protocol

Freeze source SHA, toolchain, allocation policy, workloads, seed list, transition generator, and oracle versions before a campaign. Preserve raw traces and journals. Include seed-level distributions, failed/censored runs, and benchmark harness digests. Never report a queued, skipped, unexecuted, or locally simulated campaign as a pass.

## Claim boundary

Even if all serial safety tests pass, that only qualifies the reference state machine. A distributed safety claim requires an independently implemented replicated model with explicit message, crash/restart, partition, replay, and equivocation semantics, plus exact-head execution records. A real-world fairness or economic-benefit claim requires separate empirical evidence.


## Research grounding and novelty boundary

The allocation idea has established prior art in escrow techniques and bounded-counter CRDTs: rights are partitioned so replicas can perform invariant-preserving local operations until their own rights are exhausted. See [Conflict-free Replicated Data Types](https://arxiv.org/abs/1805.06358). The relevant design question is not whether every operation can be coordination-free; it is which invariant-preserving operations may proceed with local rights and which operations (such as rights transfer or reconciliation) require coordination.

The complementary framework of [invariant confluence / coordination avoidance](https://arxiv.org/abs/1402.2237) asks whether independently valid states remain valid after allowed operations and merges. That is the correct lens for a future replicated Mycelix protocol: define the quantity, authority, evidence and finality invariants first, then identify which transitions need coordination.

The novelty claim here is deliberately narrower than inventing escrow or CRDTs. The experiment applies finite escrow rights to staged economic fulfilment while keeping accepted physical quantity, pending reservations, evidence decisions and external settlement authority explicitly separate. It is not yet a CRDT: there is no defined merge operator, causal metadata, or proof that merged replica states preserve the invariants.
