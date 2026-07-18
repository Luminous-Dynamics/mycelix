# Transaction conflict policy v2

## Purpose

Holochain transaction updates form a tree. Network partitions can expose more than one valid leaf after reconnection. Marketplace must preserve that evidence and must never pick a material winner by timestamp, arrival order, or action-hash ordering.

Policy v2 first permits two narrow automatic projections where one **existing authored head** is safety-dominant:

1. **Cancellation dominates pre-shipment progression.** Exactly one `Cancelled` head may supersede only `Pending` or `Confirmed` heads. No shipping or delivery evidence may be present.
2. **Dispute dominates ordinary lifecycle progression.** Exactly one `Disputed` head may supersede `Pending`, `Confirmed`, `Shipped`, or `Delivered` heads. This halts ordinary lifecycle actions but does not decide fault or compensation.

Every other multi-head state initially remains `Conflicted`, including `Shipped` versus `Cancelled`. It may become `AuthorizedResolved` only through one of two immutable authority paths:

3. **Bilateral authority.** Buyer and seller independently approve the same existing head while binding the exact stable root and complete conflict-head set. Neither approval alone is sufficient.
4. **Arbitration authority.** A conflict-bound dispute and final arbitration result may select only the unique conflict head authored by the adjudicated winner.

The resolver returns every raw head, every safely superseded head, and every applied authority record. No DHT record is deleted or rewritten.

## Output contract

A transaction resolution contains:

- stable original Create action;
- policy version and machine-readable reason;
- `Resolved`, `AutoResolved`, `AuthorizedResolved`, or `Conflicted` state;
- an existing authored canonical head only when safe;
- every locally visible raw head;
- explicit superseded heads;
- immutable applied bilateral or arbitration authority records;
- number of traversed revisions.

`AutoResolved` and `AuthorizedResolved` are projections, not synthesized transaction actions. Authority selects an existing authored branch and carries forward only along that branch; a later fork requires fresh authority. Terminal cancellation cannot be mutated further. A dispute projection may be passed into the arbitration path while all competing lifecycle evidence remains inspectable.

## Explicit exclusions

The policy does not:

- synthesize a new transaction state;
- merge tracking metadata;
- use timestamps as authority;
- resolve two cancellation heads or two dispute heads;
- choose between shipped and cancelled branches without explicit authority;
- permit one party to canonicalize an unsafe conflict unilaterally;
- treat bilateral approval as fault attribution or compensation;
- assign blame or economic compensation without arbitration;
- prove network-scale convergence or Byzantine tolerance.
