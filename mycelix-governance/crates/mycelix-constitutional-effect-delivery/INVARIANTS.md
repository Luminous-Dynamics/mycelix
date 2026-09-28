# MYC-CONST-003D1E implementation invariants

This maps the implementation to the frozen delivery-core requirements in #3397. Exact-head qualification remains the separate responsibility of #3405.

| Invariant | Enforcement |
| --- | --- |
| Intent-before-dispatch | PreparedDeliveryIntent has no dispatch API. A DispatchPermit can only originate from a record created from CommittedDeliveryIntent. |
| Stable semantic identity | EffectBinding is retained by the record. Retry and observation paths require exact equality, covering instance root, contract root, payload commitment, sink root, and sink epoch. |
| Unknown is first-class | OutcomeUnknown is a distinct state and observation. Transport/provider acknowledgement never maps to KnownNoEffect. |
| Evidence provenance | TransportAccepted, ProviderAcknowledged, SemanticSuccess, OutcomeUnknown, and reconciliation observations are separate kinds. |
| Monotonicity / fail-closed | Unknown never downgrades known success/no-effect. Success vs no-effect contradiction halts. IntegrityHalted is absorbing. |
| Replay profile | NoAutomaticRetry blocks unknown retry. IdempotentByEffectInstance reuses the exact binding. Retry after no-effect is explicit and profile-gated. |
| Reconciliation | Reconciliation must bind to an existing observation from the same attempt; unknown resolution must target the exact unknown observation. |
| Completion | Local durable completion requires KnownSuccess and remains recorded if later contradictory evidence forces IntegrityHalted. |
| Caller acknowledgement | Acknowledgement changes only volatile state and is omitted from snapshots. |
| Commit truth | Only Committed produces delivery authority; DefinitelyNotCommitted and CommitOutcomeUnknown do not. |
| Recovery | DeliverySnapshot retains semantic state, attempts, observations, and completion. recover() validates schema, identity stability, ordering, terminal evidence shape, and completion evidence. |

## Negative corpus

The unit tests cover:

- no committed intent -> no attempt;
- identity drift on initial attempt, retry, and reconciliation;
- stale dispatch permit after late success;
- provider acknowledgement mistaken for effect success;
- timeout mistaken for no-effect;
- retry while unknown under NoAutomaticRetry;
- identity-preserving idempotent retry;
- reconciliation against a different identity/attempt;
- completion without success;
- retry after success;
- contradictory success/no-effect evidence;
- duplicate observation replay;
- caller acknowledgement changing durable state;
- snapshot recovery after retry;
- malformed conflicting snapshot state.

## Qualification boundary

The unit corpus establishes the finite Rust transition contract. #3405 must additionally execute the complete crash-cut matrix and named mutation corpus. Neither tranche claims physical exactly-once behavior from a provider.
