# Transaction Conflict Resolution Protocol v1

## Scope

This protocol resolves only transaction update conflicts that the automatic safety
policy cannot project. It does not delete, rewrite, merge, or synthesize transaction
history. A resolution selects one already-authored branch and preserves every other
branch as superseded evidence.

The protocol has two authority paths:

1. **Bilateral agreement** — the buyer and seller independently approve the same
   selected branch against the same exact conflict-head set.
2. **Arbitration award** — a conflict-bound dispute is finalized by the guarded
   arbitration protocol, and exactly one conflict head was authored by the winning
   transaction party.

Neither party can unilaterally canonicalize an unsafe conflict.

## Stable identity and branch binding

Every approval and resolution binds:

- the original Transaction Create action;
- protocol version `1`;
- a sorted, duplicate-free set of at least two transaction heads;
- one selected head that is a member of that set;
- immutable buyer, seller, listing, quantity, price, creation time, and epistemic
  identity inherited from the transaction tree.

The coordinator creates bindings from the currently observed raw leaf set. Integrity
validation proves that every bound action belongs to the declared transaction root and
shares the immutable transaction identity. The reducer applies a resolution only when
every current leaf descends from exactly one bound head and the selected branch has
exactly one current descendant.

This lets the selected branch progress without concealing later descendants of a
superseded branch. A new fork on the selected branch remains a new explicit conflict.
Fresh authority bound to that exact later head set may resolve the new conflict; older
records remain immutable historical evidence and cannot silently decide the new fork.

## Bilateral agreement

A `TransactionConflictApproval` is immutable and must be authored by its declared
approver. The approver must be the authoritative buyer or seller from the transaction
root.

A bilateral `TransactionConflictResolution` is valid only when:

- one bound approval is authored by the buyer;
- one bound approval is authored by the seller;
- both approvals bind the same root, policy version, exact head set, and selected head;
- the final resolution repeats that exact binding;
- the resolution author is one of the transaction parties.

Multiple equivalent approvals or resolutions may exist as audit evidence. If valid
resolution entries authorize different selected branches, the transaction remains
conflicted.

## Arbitration award

A conflict dispute binds the exact unsafe head set in addition to the stable
transaction root. Ordinary lifecycle disputes continue to bind one authored Disputed
revision and have an empty conflict-head set.

An arbitration-authorized transaction resolution is valid only when:

- the dispute is bound to the same transaction root and exact conflict heads;
- the arbitration result belongs to that dispute and names the authoritative buyer or
  seller as winner;
- exactly one bound conflict head was authored by the winner;
- that unique winner-authored head is the selected head;
- an assigned arbitrator authors the resolution entry.

The arbitration result decides which party's existing branch prevails. It does not
create a new transaction state or silently choose between multiple branches authored by
the same winner.

## Reducer states

The transaction reducer exposes:

- `resolved` — one live raw head;
- `auto_resolved` — the narrow safety policy projects an existing authored head;
- `authorized_resolved` — bilateral or arbitration authority projects an existing
  authored branch;
- `conflicted` — no unique safe or authorized projection exists.

Authorized projections include the resolution action, authority kind, exact bound
heads, selected head, and authority evidence hashes. All raw current heads remain in
the response.

## Fail-closed conditions

The reducer must remain conflicted when any of the following is true:

- approvals disagree on the selected head or conflict-head set;
- a required approval is missing or authored by the wrong party;
- multiple valid resolution entries authorize different branches;
- the selected branch has forked into multiple current descendants;
- a current head cannot be traced to one of the resolution's bound heads;
- an arbitration result is unrelated to the conflict-bound dispute;
- the winner authored zero or multiple bound conflict heads;
- immutable transaction identity differs between heads;
- evidence is incomplete or unavailable.

Action-hash ordering, timestamps, arrival order, and coordinator preference are never
material authority.

## Claim boundary

A passing implementation demonstrates an explicit, auditable projection protocol for
a specific transaction update tree. It does not demonstrate automatic semantic merge,
rollback of side effects, global consensus, Byzantine tolerance, or legal enforceability.


## Controlled network evidence

The version-3 network promotion profile exercises the bilateral path after a real
`shipped` versus `cancelled` partition conflict. It requires independent approvals from
the buyer and seller, DHT visibility of both approval actions, an authority entry bound
to the original two heads, and the same `authorized_resolved` shipped projection on both
conductors. The cancelled branch remains visible as superseded evidence.

The arbitration authority path is integrity-validated, typed, and covered by static and
wire contracts, but multi-conductor arbitration-authorized convergence remains a
separate promotion boundary.
