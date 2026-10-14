# AC-073: Deterministic Current Decision-Version Reads

## Purpose

Holochain's default `get` returns the live creation record and does not impose an application-specific update conflict policy. When an application needs a single current view, it must resolve update conflicts itself. urlHolochain Entries documentationhttps://developer.holochain.org/build/entries/

AC-073 makes that rule explicit for Hearth Decision lifecycle reads.

## Resolver

`get_current_decision_record` starts from the root Decision action and traverses the reachable valid Update graph.

Each revision is classified as either:

- `Open`;
- terminal: `Closed` or `Finalized`.

Terminal revisions dominate Open revisions. This prevents a concurrent status-preserving `Open -> Open` branch from resurrecting a Decision after a terminal update exists.

Within the selected class, the resolver chooses the greatest:

```text
(action timestamp, raw action hash)
```

All revisions remain preserved in the DHT.

## Lifecycle consumers

The Decision-specific resolver is used for:

- voting status checks;
- finalization status checks;
- close status checks;
- vote-amendment status checks;
- single-decision reads;
- hearth decision listings;
- pending-decision reads.

Immutable Vote and Outcome reads retain their existing paths.

## Why this complements AC-071/072

AC-071 resolves concurrent `DecisionOutcome` candidates deterministically.
AC-072 records the exact Decision action/version used as the finalization basis.
AC-073 ensures that lifecycle checks themselves operate on a deterministic current Decision revision.

Together:

```text
Decision revision
    -> lifecycle state
    -> exact finalization basis
    -> concurrent outcome candidates
    -> canonical outcome view
```

## Boundary

AC-073 does not make concurrent finalization exactly-once. Two agents can still race after observing the same Open revision. The purpose here is to eliminate stale root-record lifecycle reads and to make observable Decision revision selection explicit and deterministic.