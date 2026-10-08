# Partitioned and delayed observer gossip simulation research v1

Status: research-only, deterministic event model.

This layer moves gossip validation from isolated pairs of observations to a deterministic event trace. The simulator replays signed observations through partitions, delayed delivery, message reordering, duplicate delivery, and stale replays.

## Model

- Two relying observers maintain local sets of authenticated monitor observations.
- An observation is admitted only after verifying the monitor signature and the referenced witness tree-head signature.
- A partition blocks message delivery; healing releases queued messages in a specified order.
- Observation IDs are idempotent. Duplicate deliveries do not create extra independent evidence.
- A stale head is retained as historical evidence but cannot lower the locally computed high-water tree size.
- No global result is inferred merely because a central simulator can inspect two isolated local states.

## Distinct evidence classes

The model separates:

- `monitor-equivocation`: one monitor has signed two observations of conflicting roots for the same tree size;
- `split-view-detected`: authenticated observations from distinct monitors expose conflicting roots for the same tree size;
- `converged-consistent`: both observers have the same authenticated observation set and every different-size pair is linked by a valid consistency proof;
- `unresolved-local-only`: the observers have not exchanged enough evidence to reach a shared result.

The distinction matters when a monitor signs conflicting claims: monitor self-equivocation is direct evidence even though it is not independent-monitor corroboration. Conversely, a partitioned pair of clients cannot claim shared agreement simply because an evaluator can see both local states.

A valid contradictory signed head remains evidence even where a separate honest head set has a 4-of-4 quorum. Quorum is not used to erase explicit same-size contradictory-root evidence.

## Trace corpus

Eight deterministic traces cover:

1. delayed but append-only-compatible heads that converge after healing;
2. a split view revealed only after the partition heals;
3. a permanent partition that remains unresolved;
4. reordered delivery plus duplicate replay;
5. a single monitor's self-equivocation;
6. a fork observation that is not hidden by a separate 4-of-4 honest head quorum;
7. a tampered in-transit gossip observation that is rejected;
8. stale historical replay after a newer tree head without high-water rollback.

The Python and Node simulators independently validate the signed evidence, replay the same event traces, and produce byte-identical reports.

## Claim ceiling

This is a deterministic discrete-event model, not a live network test. It demonstrates the classification behavior of the modeled delivery schedules. It does not establish real-world convergence, message-delivery guarantees, censorship resistance, monitor independence, or behavior under arbitrary Byzantine network control.

A permanently partitioned network is expected to remain unresolved; the model deliberately makes no availability claim where messages cannot cross the partition.

References:
- RFC 9162, Sections 2.1.4 and 5.4 (consistency and gossip): https://www.rfc-editor.org/rfc/rfc9162.html
- RFC 9942, Sections 5.2.1 and 5.3.1 (inclusion and consistency receipts): https://www.rfc-editor.org/rfc/rfc9942.html
