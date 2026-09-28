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

## Replay

Retries may change `attempt_id`, but must retain `logical_delivery_id`, payload digest, and origin. An existing receipt with a changed payload or origin rejects the replay.

## Claim ceiling

`ReferenceModelOnly`.

D6E does not establish network reliability, cryptographic authenticity, privacy compliance, scalability, economic correctness, governance legitimacy, or human outcomes.


## Competing observations

Federation reconciliation also has an explicit observation boundary. Two observations of the same work with different quantities are retained as distinct records, including their independent origins and evidence references. The reference model produces an ObservationConflict; it does not choose a winner or rewrite either observation.

That conflict can feed the existing FRS conflict/assessment seam, where a later human decision remains separate from the observations themselves.
