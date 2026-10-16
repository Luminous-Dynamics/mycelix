# Hearth Publication Evidence UI V1

**Status:** Leptos client integration authored; tests and browser/WASM qualification remain outstanding.
**Parent:** [HTH-UX-007 / PR #5084](https://github.com/Luminous-Dynamics/mycelix/pull/5084).

## Design rule

Keep the mutation result and replication evidence as separate timelines. A successful mutation returns immediately to the resident as source-chain saved. One best-effort publication-evidence query follows against the exact action hash from that returned Record. Query failure does not alter the earlier mutation result and does not cause a retry.

The view stores only one latest operation status. It does not poll continuously, block the write button on receipt collection, infer household-wide synchronization, or use publication status as an authorization signal.

## Concurrent query ordering (HTH-UX-009)

Multiple successful actions can start publication-evidence queries whose responses arrive out of order. Because this view intentionally stores one latest snapshot, every query reserves a monotonically advancing generation before awaiting the conductor. A response may update the shared status only when its generation still matches the current generation; stale responses are discarded without retry. The generation protects the subject, action reference, and evidence as one tuple.

This is a client-side latest-snapshot ordering rule. It does not cancel conductor work, prove mutation ordering, create durable history, or turn publication evidence into an action outcome.

## Flow

1. A live mutation returns a decodable WireRecord.
2. The handler immediately emits the resident-facing "saved; peer sharing may still be in progress" message.
3. The handler calls the shared publication-evidence helper once using the action hash directly from the returned record. The short action-hash prefix is display-only.
4. The visible status becomes Checking, then maps the API result to NotAvailableToCaller, NoOperationData, Observed receipt counts, or Unavailable.
5. The mapper checks that the response action hash equals the requested hash and verifies all operation/receipt aggregate counts against per-operation values. Any mismatch fails closed to Unavailable.
6. The component renders the exact observation timestamp in ISO form and states that the evidence is a snapshot that may become stale.
7. The action's source-chain-saved message is not rewritten by receipt query failure, rejection evidence, no operation data, or a later conductor error.

## UI semantics

- **Checking:** one follow-up query is in progress.
- **Unavailable:** call failed, timestamp/counts were invalid, or returned evidence was internally inconsistent. Peer status is unknown.
- **Not available to caller:** the API intentionally supplied no evidence to this caller. This is distinct from a checked-but-empty query.
- **No operation data:** a valid response said no DHT operation receipt sets were available. This is not complete, failure, or proof no operation exists.
- **Receipts observed:** show operation count, observed receipt records, valid/rejected/abandoned record counts, the number of operations that met Holochain's configured receipt threshold, the checked-at timestamp, and the explicit caveat that this does not prove every peer currently has the update.
- **Rejected/abandoned:** use the attention treatment if any operation has those receipt statuses, even if threshold completion is also true.

Threshold completion uses the host-supplied `receipts_complete` per operation. Do not hardcode five. No operation set means no completion (never vacuous truth). The short action hash is a display reference, not an operation key for new dispatch.

## Scope and privacy

- Query only the action hash returned by this user's successful write.
- Query runs through the Hearth role and `hearth_bridge.get_action_publication_evidence`.
- No API retry or repeated poll in this version.
- The API narrows HDK receipt visibility to the calling agent's own action and does not disclose validator keys or raw DHT operation hashes.
- Client-side DTO mapping fails closed on action-hash mismatch, negative timestamp, count mismatch, an Observed result with zero operation sets, or a malformed NoOperationData/NotAvailableToCaller state.
- The UI has one shared latest-evidence status, not a durable multi-operation journal. This does not replace HTH-UX-005's journal adapter work.

## Verification authored

The frontend now has tests for:
- DTO MessagePack round-trip;
- empty operation sets do not imply completion;
- per-operation valid/rejected/abandoned counts map correctly;
- mismatched action hash fails closed;
- inconsistent aggregate counts fail closed;
- Observed with zero operations fails closed;
- caller-unavailable evidence does not reveal counts;
- invalid timestamps fail closed;
- status styling distinguishes unknown, progress, threshold-complete, and attention states.

These tests are authored but not yet executed. They test the local decoder/presentation contract and latest-request generation guard, not actual browser scheduling or a WebSocket call.

## Benchmarks (targets, not results)

1. 100% of successful mutation responses surface source-chain-saved feedback without awaiting the publication query.
2. Zero publication-query failures reported as action failures.
3. Zero global-sync claims.
4. 100% of publication reports bound to the requested action hash and containing a valid observation timestamp.
5. Zero malformed/inconsistent DTOs shown as receipt-threshold complete.
6. At most one post-action publication query per successful mutation in V1; zero continuous polling.
7. 100% of checked-but-empty results distinguished from not-checked and unavailable.
8. At least 90% of study participants distinguish "saved" from "receipt thresholds observed"; at least 90% understand neither guarantees every peer has the latest state.
9. 100% of stale evidence-query completions are prevented from replacing the latest requested evidence snapshot.
10. Qualify the exact head with Leptos 0.8 WASM compilation, frontend unit tests, and browser-side call mock before merge. Queued/skipped/absent is not PASS.

## Nonclaims

The status is one local, authoring-conductor observation. It is not global DHT health, a current peer-availability guarantee, a propagation SLA, durable history, a delivery guarantee, or an authorization decision.
