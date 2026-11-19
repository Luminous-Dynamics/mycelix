# Kernel observer accounting and independent evidence design

**Status:** design proposal and PEAB test plan. No BPF program is implemented or attached by this document. No kernel enforcement, loss-free observation, or production event correlation is claimed.

## 1. The observer must account for its own failures

A kernel event stream is not trustworthy merely because it came from a kernel hook. A full ring buffer can refuse a reservation without blocking the hook. The Linux BPF ring-buffer documentation explicitly describes non-blocking reservation failure when space is unavailable; it also describes failure in NMI context when the reservation lock cannot be acquired even if the ring is not full.

This creates a second failure channel: the observer can miss the event it was supposed to report. An event-loss signal written into the same saturated ring buffer is not a sufficient solution: the failure record can be lost for the same reason as the event. Prism therefore needs independent accounting of attempted, published, lost, ingested, and reconciled events.

## 2. Proposed event identity and completeness contract

Each record should carry:

- observer boot/session identity and monotonically increasing attach generation;
- hook/program identity and protocol family (connect4/connect6, UDP sendmsg, packet egress);
- monotonically assigned attempt sequence within its observer generation (unique per event; socket cookies may repeat across lifecycle records);
- monotonic kernel timestamp;
- cgroup/workload identity and an identity-generation or equivalent anti-reuse context;
- socket identity where the hook exposes it, destination address/port, family, protocol, and kernel verdict;
- policy/map generation and a compact reason code.

The event sequence is an attempt sequence, not just a ring-buffer publication sequence. Sequence uniqueness is scoped by observer generation; socket cookies are socket-lifecycle identifiers and must not be treated as unique event IDs, because one socket can appear in more than one observation. Allocate it before the ring reservation attempt. If reservation fails, a later published record will expose the hole; a separately readable loss counter records the failure reason. The snapshot must also publish the attempt high-watermark so loss of the final event is detectable even when no later event exists.

Sequence allocation before reservation can differ from reservation/publication order under concurrency. Consumers must sort/validate by attempt sequence, not infer causal order from ring-buffer delivery order. If global atomic sequence contention is too expensive, per-CPU or sharded sequences can be benchmarked, but the evidence model must then preserve per-shard monotonicity and refuse unsupported cross-shard ordering claims.

## 3. Out-of-band producer and consumer accounting

Keep at least the following counters outside the event ring:

| Counter | Updated by | Purpose |
|---|---|---|
| attempted hooks | BPF map | Number of connect/sendmsg/packet events entering observation |
| failed reservations | BPF per-CPU map | Ring reservation failures, including non-capacity failure reasons where distinguishable |
| published records | BPF/user-space reconciliation | Records committed to the ring |
| consumer callback failures | user-space observer | Decode/copy/dispatch failures after ring publication |
| journal-ingest failures | user-space observer | Bounded queue or durable-write failures |
| unmatched records | reconciler | Kernel events that have no unique application action |
| ambiguous joins | reconciler | Events inside overlapping same-workload action windows |

The producer loss counter must not be emitted only as another ring-buffer record. Read it through a separate map snapshot/control channel. Consumer and journal failures also require independent process-side counters; a record that was read from the BPF ring but dropped before durable storage is still missing evidence.

At minimum, the snapshot includes: generation, attach/detach monotonic times, attempted-event high-watermark, per-CPU loss counters, published/drained counts, sequence holes, consumer-ingest failures, and the event stream's first/last sequence.

## 4. Stable snapshot is a protocol, not a best-effort read

A counter read concurrent with producers is only a momentary sample. To claim a complete window, the observer needs a generation barrier:

1. Mark the old generation closing and atomically publish a new generation for subsequent hooks.
2. Track in-flight producers by generation so userspace can wait until old-generation hooks have exited.
3. Drain all committed records for the old generation.
4. Read the final attempt high-watermark and out-of-band loss counters only after the old generation is quiescent.
5. Reconcile the exact expected sequence range against records, producer loss, consumer failures, and durable journal records.
6. Persist both the snapshot and the attach/detach interval; any gap remains permanently unknown.

This is a proposed protocol requiring a verifier-safe BPF implementation and race testing. A simple pair of reads around a ring-buffer drain is not sufficient if producers can still publish late records.

The kernel documentation exposes ring-buffer producer/consumer positions and available-data snapshots as momentary observations. Those values are useful diagnostics and possible heuristics, but the documentation warns they can be stale by the time they are read. They must not be the only proof of evidence completeness.

## 5. Ring topology: benchmark the trade-off

The Linux shared ring buffer was designed to use memory efficiently across CPUs and preserve reservation ordering across CPUs. The same documentation notes that reservation is serialized, and that later committed records can wait behind an earlier reservation that has not yet been committed. It also permits more complex topologies such as per-CPU or sharded ring buffers.

PEAB should compare, on the same kernel/workload:

1. One global ring: simplest cross-CPU reservation order; potential shared contention.
2. Per-CPU rings: isolate producer pressure; require per-CPU sequence and explicit merge policy.
3. Sharded rings keyed by workload/cgroup hash: preserve local order while balancing contention; require shard-generation provenance.

Measure attempted events/s, published events/s, lost count by reason, p50/p95/p99 hook cost, CPU cost, ring occupancy, consumer lag, memory, event reordering, and time to detect an overflow. A zero-loss claim is invalid unless the induced slow-consumer and saturation controls actually force the system through pressure.

## 6. Enforcement requires two separate hook planes

Do not equate socket admission with ongoing packet enforcement:

- Cgroup socket-address hooks (IPv4/IPv6 connect, plus UDP sendmsg where applicable) can qualify new socket-operation admission.
- Cgroup skb egress hooks operate on outgoing packets and are the relevant additional plane to investigate for established flows and passed descriptors.

They answer different questions. PEAB reports new-flow connect/sendmsg admission separately from packet treatment on an already-connected socket. Fork/exec, SCM_RIGHTS descriptor transfer, cgroup migration, TCP established flow, UDP, raw sockets, IPv4/IPv6, and policy expiry must each be tested on the supported kernel matrix. Do not claim passed-socket revocation from a connect-hook test.

The kernel hook family is a proposed reference boundary, not proof that an attached program currently blocks every path or receives all required identity fields. Validate program attachment, hook invocation, return verdict, and packet outcome independently.

## 7. Reconciliation and fail-closed outcomes

The test-only Rust reference model in evidence_reconciliation_model.rs defines the current minimum application/kernel join: same workload plus a bounded monotonic action interval, a complete observer window, unique socket identity, and an allowed observed peer inside the capability's authorized set. Overlapping action windows remain ambiguous.

The reconciliation output must retain separate states:

- reconciled connected / denied-before-connect;
- incomplete due to loss, sequence gap, observer generation gap, missing event or ambiguous join;
- contradictory due to unauthorized peer, kernel denial or unexpected activity;
- unmatched kernel event or application receipt.

Only a complete and unambiguous join can become independently corroborated. No missing event may be interpreted as proof of no socket activity.

## 8. PEAB work items

- OBS-001..008 qualify incomplete evidence, ambiguity, generation-scoped event-sequence uniqueness, and socket-cookie lifecycle semantics at the reference-model layer.
- OBS-009 is planned: force interior and terminal ring-buffer reservation failures, prove the out-of-band counters and high-watermark reveal the loss, then verify the result remains incomplete.
- KERNEL-008..010 separate connect/sendmsg policy from packet-egress behavior and established-flow treatment.
- Compare global, per-CPU and sharded ring topologies under identical workloads before choosing a production design.

The reference model and this design are source-level work only. They do not constitute an eBPF observer, independent runtime attestation, a signed receipt, or a passing PEAB run.
