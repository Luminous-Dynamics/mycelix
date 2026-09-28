# D6E — deterministic heterogeneous federation model

D6E adds a transport-neutral federation boundary to the Integral reference node.

The model distinguishes:

- **logical delivery identity** from retry/attempt identity;
- **evidence origin** from the node exercising authority;
- **schema generation** from presentation state;
- **partition** from successful delivery;
- **privacy-minimized projections** from source observations;
- **active authorization** from reconnect/reconciliation.

A foreign evidence envelope may be accepted under explicit local authority while retaining `origin = Foreign`. A foreign node cannot acquire local authority merely by projecting or reconnecting an envelope.

## Deterministic oracle

`accept_delivery` checks schema generation, authority origin, authorization/expiry, partition state, privacy projection status, and finally replay identity/payload.

`reconcile` does not grant a new authority or expiry. It re-enters the same acceptance boundary with `Reconciled` delivery state.

## Delivery replay

Retries may change `attempt_id`, but must retain `logical_delivery_id`, payload digest, origin, and authority origin. The replay oracle canonicalizes delivery ordering before evaluating attempts. A mutation is rejected even if the mutated retry sorts before the original attempt.

This makes the replay property independent of transport arrival order rather than merely dependent on the current vector order.

## Event-sourced observation state

D6E now has a separate append-only observation history. Exact duplicate observation events are replayable; reusing an observation identity with changed quantity, origin, work, or evidence is rejected as a duplicate mutation.

`FederationObservationState::replay` canonicalizes observations by observation identity and reconstructs **all** pairwise conflicts. It therefore separates:

1. event history;
2. reconstructed evidence state;
3. conflict preservation;
4. later governance resolution.

Replaying the same observations in different delivery orders, or with exact duplicate events, produces the same canonical evidence state. No replay step selects a winner.

## Competing observations

Two observations of the same work with different quantities are retained as distinct records, including their independent origins and evidence references. The reference model produces an `ObservationConflict`; it does not choose a winner or rewrite either observation.

That conflict can feed the existing FRS conflict/assessment seam, where a later human decision remains separate from the observations themselves.

## Reconciliation is not resolution

The observation reconciler is explicitly order-invariant. Reversing delivery order cannot change whether the reference model sees agreement or conflict.

When quantities disagree, the result is `ConflictPreserved`; there is no implicit winner. A later FRS or governance process must make any resolution explicit rather than having federation silently choose one observation.

## Resolution boundary

A preserved conflict is not a resolved conflict. `resolve_conflict` returns `AwaitingHumanDecision` until an explicit decision reference is supplied. The federation layer therefore records disagreement without selecting an outcome; a subsequent decision is a separate artifact.

## FRS handoff

A preserved federation conflict can now be handed to the FRS seam without selecting a quantity. FRS may recognize the disagreement and require an explicit `Accepted` or `Rejected` CDS decision; `Draft` is not treated as resolution. The reference model therefore keeps observation, assessment, recommendation, and governance decision as distinct artifacts.

## Claim ceiling

`ReferenceModelOnly`.

D6E does not establish network reliability, cryptographic authenticity, privacy compliance, scalability, economic correctness, governance legitimacy, or human outcomes.
