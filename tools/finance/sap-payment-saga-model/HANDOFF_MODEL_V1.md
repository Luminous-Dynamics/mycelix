# Delegated grant handoff reference model v1

**Status:** effects-disabled Rust reference state machine and independent Go verifier. Models separate parent and child owner-local histories. Not a production Holochain coordinator, signature verifier, distributed transaction or payment rail.

## Problem

A parent-escrow grant is not one transaction across two owners. The parent source chain may commit an allocation while the child is offline; a grant/acceptance/revocation/closure message may be duplicated, delayed or lost; a process may restart after either side commits. A lost acknowledgement is not evidence that the action failed.

The reference therefore separates:
- parent grant commitment and budget encumbrance;
- child grant acceptance;
- parent revocation request;
- child-local revocation observation;
- child exposure resolution and closure;
- parent-owned escrow release.

Each side preserves its own action identity. The parent never releases a delegated budget because a delivery timed out or an acknowledgement is missing.

## Authority and handoff identities

GrantTermsV1 binds:
- stable grant ID;
- parent owner, pool and currency;
- child owner, pool, destination and currency;
- integer minor-unit budget;
- parent-local grant sequence;
- parent policy sequence and policy digest.

The canonical grant digest is independent of either implementation and domain-separated by MXSAP-GRANT-TERMS-V1 plus a zero byte.

Parent grant issuance binds the parent action hash. Replaying the same terms with the same parent action hash is idempotent. Reusing the grant ID with changed terms or a different parent action hash conflicts. A replay must recover the already-committed action; it must not create a second budget allocation.

The reference checks digest syntax and exact subjects. It does not verify Holochain action signatures, source-chain authorship or policy authorization. Those are adapter responsibilities.

## Parent grant lifecycle

| State | Parent escrow encumbered? | New child risk |
|---|---:|---|
| Issued | Yes | Child cannot use until exact grant acceptance |
| RevocationRequested | Yes | Child may continue within the grant until it observes revocation |
| Released | No | No further risk; release action is final |

The parent commits the whole budget against its own cap at grant issuance. The same source-chain transaction must durably add the grant and the delivery/outbox obligation. A failed local commit creates neither. A post-commit worker sends the committed grant using stable delivery ID grant:<grant_id>.

An acceptance acknowledgement only tells the parent that the child accepted the grant. It never releases or reduces parent encumbrance.

## Child grant lifecycle

A child book is bound to exact grant terms and the parent action hash.

- Before acceptance, child reservations and dispatch are rejected.
- Exact duplicate acceptance returns the original child acceptance action hash.
- A different grant, parent action or changed grant terms conflicts.
- After acceptance, child reservations are bounded by the delegated amount.
- Once the child commits its local revocation-observation barrier, no new reservations or dispatch authorization are allowed.
- Effects already dispatch-authorized before the barrier remain eligible for evidence retrieval, reconciliation, settlement or authoritative-absence resolution.
- Never-dispatched reservations may be cancelled; they are not treated as having produced an external effect.

## Revocation is not instantaneous across a partition

The parent appends a RevokeGrantV1 equivalent on its own source chain. This records the parent's intent and creates a durable delivery obligation, but it does not make a remote child's source chain change atomically.

Until the child observes the exact parent revoke action and commits its local barrier, the child may still be unaware of revocation and can continue using its already-delegated budget. This is an explicit asynchronous semantic, not a correctness defect.

The safety mechanism is the parent escrow: the entire sublimit remains encumbered while delivery, acknowledgement or child visibility is delayed. The parent cannot give that same budget to another child.

The child records a revocation receipt binding:
- grant ID and grant digest;
- parent revoke action hash;
- child owner;
- child revocation-observation action hash.

Duplicate delivery returns the same receipt. A different revoke action conflicts.

## Child closure and parent release

A child may close only after:
1. it has observed the exact parent revoke action;
2. its exposure book has zero outstanding reserved, dispatch-authorized, indeterminate or committed-unsettled exposure;
3. it has generated a canonical child exposure snapshot digest;
4. it has a child source-chain frontier and child close action identity.

GrantClosureReceiptV1 binds:
- grant and parent action hashes;
- parent revoke action hash;
- child owner;
- child acceptance action hash if the child accepted the grant;
- child revocation-observation action hash;
- child source-chain frontier;
- child exposure snapshot digest;
- child close action hash;
- zero-unresolved disposition.

A never-accepted grant may be revoked and safely closed with no acceptance action hash, provided the child has the exact grant terms and observes the parent revoke action. This avoids permanently stranding a parent budget allocated to an undelivered child grant.

The parent accepts closure only for the exact grant, child owner, parent action and revoke action. If it already observed acceptance/revocation receipts, the closure's corresponding child action hashes must match those receipts. If an intermediate acknowledgement was lost, the closure receipt carries the action identities needed to recover the parent view.

Only the parent-owned ParentGrantReleasedV1 equivalent frees the sublimit. Duplicate release with the same closure is idempotent; a changed closure conflicts. Missing, stale, ambiguous or unverifiable child closure evidence leaves the budget encumbered.

The in-memory reference does not authenticate the child action. Production must verify the exact child-authored source-chain action, signature, author, predecessor/frontier, grant subject and canonical snapshot before appending a parent-owned release action.

## Outbox and crash recovery

The parent book retains deterministic pending deliveries:
- grant:<grant_id> until acceptance is observed or a closure proves safe cancellation/closure;
- revoke:<grant_id> until the child revocation receipt is observed or a closure proves it.

Retries reuse the same stable delivery IDs and parent action hashes. A lost response after child acceptance, child revocation observation, child closure or parent release is recovered by querying/replaying the existing action identity, not inventing a new grant or release.

Rust and Go Clone helpers model process restart from a persisted snapshot. They are not durable storage. Production recovery must read owner-local source-chain actions and rebuild the state from verified evidence.

## Canonical snapshots and shared KAT

- Grant terms domain: MXSAP-GRANT-TERMS-V1 plus zero byte.
- Parent snapshot domain: MXSAP-HANDOFF-PARENT-V1 plus zero byte.
- Child snapshot domain: MXSAP-HANDOFF-CHILD-V1 plus zero byte.
- Integers are unsigned big-endian.
- Text is UTF-8 with u32 big-endian byte length, bounded at 4,096 bytes.
- Maps are serialized in sorted grant-ID order.
- Optional receipts have explicit presence markers.
- The shared fixture sap-payment-handoff-v1.kat pins grant, parent and child snapshot digests. Rust and Go load the same fixture independently.

## Adversarial and recovery coverage

The reference tests cover:
- child use before acceptance;
- parent escrow following issue, acceptance and lost acknowledgement;
- repeated grant delivery and child restart;
- delayed revocation while the child remains within its delegated cap;
- child-local revoke barrier blocks fresh risk;
- already-dispatched effects reconcile after revocation;
- unresolved exposure prevents child closure;
- authoritative absence permits closure only for the exact subject;
- wrong child owner cannot release parent escrow;
- unaccepted grant revocation and safe closure;
- duplicate parent release;
- late acceptance/revocation acknowledgements after release;
- restart of parent and child book snapshots;
- changed parent action hash on grant replay;
- cross-language known-answer agreement.

The benchmark programme should stress 1 / 10 / 100 / 1,000 grants per parent, 1k / 10k / 100k child effects, 0–24h delivery and visibility delays, 1–10,000 duplicate/reordered messages, and restart at every transition. Measure commit-to-accept, revoke-to-child-barrier, close-to-parent-release, recovery convergence, duplicate-work load and encumbered-but-unused budget time. Safety failures are non-compensatory; throughput never offsets unsafe dispatch or release.

## Holochain integration boundary

This reference follows the documented substrate boundary:
- actions are authored to one agent's append-only source chain;
- writes inside one zome invocation commit atomically on that owner-local chain;
- action hashes become available after commit;
- remote delivery belongs in post-commit/durable outbox work;
- DHT integration and visibility are separate from local commit;
- stale strict source-chain snapshots can return chain-head-moved and must be retried from a fresh head.

Therefore the real sequence is parent-owned atomic grant + outbox, then child-owned acceptance, then parent-owned revoke + outbox, then child-owned observation and closure, then parent-owned release. No cross-owner atomic transaction or instantaneous partition-wide revocation is assumed.

## Claim ceiling

A passing Rust/Go reference suite establishes only the deterministic modeled state transitions and shared canonicalization. It does not establish:
- authentic parent/child Holochain action signatures or source-chain frontiers;
- action validity or DHT convergence;
- post-commit outbox durability across real conductor crashes;
- source-chain fork exclusion or concurrent parent grant serialization;
- provider settlement finality or authoritative absence;
- bank liquidity, collateral sufficiency, solvency, legal netting or regulatory compliance.

Production qualification requires real parent/child agents, native source-chain writes, strict chain-head concurrency, post-commit delivery recovery, delayed DHT visibility, malicious/forged receipt tests and independent verification of parent release actions.
