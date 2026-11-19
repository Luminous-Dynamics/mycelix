# Prism evidence reconciliation reference model v0.1

**Status:** executable test-only reference model; not a production observer, kernel program, signed receipt format, or runtime assurance claim.

## Why this exists

Prism produces an application-side EgressReceipt. A future independent observer will see kernel connection events. These records describe different facts: the application receipt says what Prism authorized and believed it completed; the kernel event says which workload attempted a socket connection and to which peer. Neither record alone proves that the other plane agrees.

The model makes reconciliation conservative and executable before a production eBPF observer or transport-to-socket identity bridge is selected.

## Correlation contract

The model uses workload identity, a bounded monotonic action interval, observer generation and event sequence, kernel socket cookie and actual peer, plus the application-reported peer and authorized peer set.

It deliberately does not invent a shared capability ID in a kernel event. Until a transport or broker can produce a trustworthy shared socket identity, an event falling inside multiple same-workload action intervals is ambiguous. It is never assigned by nearest timestamp, event order, or best-effort guessing.

One action may produce multiple connect attempts (for example, address racing). Those attempts may reconcile only when the action is the unique candidate for every event, each event has a unique generation-scoped sequence (socket cookies identify sockets and may legitimately repeat across lifecycle observations), every attempted peer is in the authorized peer set, and the peer selected by the application appears as an allowed kernel event. A socket event cannot be consumed by two application receipts.

## Evidence outcomes

| Outcome | Meaning | Qualification effect |
|---|---|---|
| ReconciledConnected | A complete observer window accounts for the application-reported peer and all observed attempts remain inside the authorized set | Eligible for independent corroboration |
| ReconciledDeniedNoSocket | Prism denied before opening a socket and the complete window contains no matching attempt | Eligible for independent corroboration |
| Incomplete | Event loss, sequence gap, generation mismatch, detach/reattach gap, missing event, or ambiguous join | Never report as a pass |
| Contradictory | An observed attempt violates authority or the kernel verdict disagrees with the application claim | Security failure; never average into a score |
| Unmatched kernel event | An event has no unique application receipt in the scoped workload/time window | Preserve it; do not fabricate HTTP/TLS completion facts |

A report with any unmatched kernel event is not independently qualified, even if another action reconciles successfully.

## Observer completeness is a prerequisite

The observer snapshot must cover the entire action interval in one continuous generation. It must report zero lost events, and the event stream must be contiguous from first_sequence through last_sequence. A generation change or sequence discontinuity invalidates the snapshot. Reattachment cannot retroactively close the interval in which the observer was absent.

This distinguishes:
- **No event observed:** there is no matching record.
- **No event occurred:** a complete observer window with an intact sequence proves no event occurred in that interval.
- **Unknown:** the observer lost events, changed generation, detached, or could not uniquely correlate the event.

Only the second can corroborate a denied-before-connect receipt.

## Executable coverage

The dependency-free test file prism-egress/tests/evidence_reconciliation_model.rs runs as part of cargo test -p prism-egress. It covers a valid positive control, event-buffer loss (including a missing terminal event exposed by the attempt high-watermark), overlapping actions, an orphan kernel event, a missing kernel event, detach/reattach gaps, sequence gaps, an unauthorized peer, and pre-connect denial.

PEAB entries OBS-001 through OBS-007 classify this as **partial source coverage**. These tests do not establish that a kernel observer, cgroup policy, or production receipt integration exists or works.

## Critical limitation and next experiment

A workload ID plus timestamp window cannot identify one request when requests overlap. That is a feature of the model: choosing one candidate would create false causal evidence.

Compare two mechanisms under identical concurrent workloads:

1. **Conservative correlation:** retain workload/time correlation and mark overlapping candidates incomplete.
2. **Socket-identity bridge:** expose the exact socket cookie to the application or broker for the transport connection that generated the receipt, then bind it to the one-use capability/action ID without allowing userspace to self-assert the kernel observation.

The second option likely requires a transport connector or local broker boundary; Reqwest's high-level response API does not currently expose the underlying socket cookie. Test any bridge against descriptor passing, socket replacement, connection racing, pooling, cancellation, and cgroup movement. A capability ID copied into a userspace log is not an independent kernel identity.

## Explicit non-goals

This model does not attach eBPF programs, claim enforcement, prove packet-level egress filtering or established-flow revocation, establish cryptographic signatures or trusted timestamps, infer HTTP/TLS/body completion from a connect event, or replace Nixward's workload-level enforcement work.

Application authorization, kernel observation, and their reconciliation remain separate evidence claims.
