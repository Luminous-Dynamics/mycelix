# SAP execution profile finite-state model v1

**Status:** finite-state design model and mutant suite. Exact-head TLC results are pending; this is not a Holochain conductor test or a production payment proof.

## Purpose

Formalize the semantic boundary between:

1. a single-author, one-zome-call atomic local principal transfer;
2. the same local transaction plus a separately committed Treasury fee effect; and
3. a payer-authenticated debit and a beneficiary-authenticated claim committed by distinct owner chains.

The model makes these separate configurations. It does not claim that the local source-chain transaction is cross-agent atomic or that two independent owner chains share one global transaction.

## State variables and transitions

- `LocalAtomicCommit`: atomically sets payer debit, beneficiary credit, Payment, Receipt and the fee outbox intent in one transition.
- `TreasuryFeeCommit`: a separate transition, enabled only after the local transaction and outbox intent exist.
- `BilateralPayerCommit` and `BilateralBeneficiaryCommit`: independent owner-chain effects.
- Owner-view observations can be complete or incomplete; incomplete views do not imply absence.
- `ProjectBilateral` applies conflict precedence, then Indeterminate for incomplete views, then Supported only when both effects exist in complete views, otherwise Pending.
- `MarkOperationComplete` is enabled only after the exact selected profile's required effects are satisfied.

## Safety invariants

- `LocalAtomicNoPartial`: local debit, credit, Payment, Receipt and outbox preparation cannot be observed as separately committed local effects.
- `TreasuryRequiresOutbox`: the fee effect cannot commit before the local transaction and exact outbox intent.
- `ExecutionModelIsolation`: a local profile cannot use payer/beneﬁciary independent owner legs; a bilateral profile cannot use the local atomic transaction.
- `BilateralOwnerSeparation`: payer and beneficiary owners are distinct in the bilateral profile.
- `BilateralProjectionSound`: Supported requires both complete owner views and both observed effects; Pending requires complete views and a missing effect; Indeterminate requires an incomplete view; Conflict requires conflicting evidence.
- `CompletionRequiresEffects`: overall completion cannot precede local atomic/required Treasury effects or bilateral projection support.

## Negative controls

| Mutant | Expected invariant | Defect represented |
|---|---|---|
| Split local atomic write | `LocalAtomicNoPartial` | Debit appears without the rest of the one-transaction write set |
| Complete immediately after outbox preparation | `CompletionRequiresEffects` | Outbox intent is mistaken for Treasury effect commit |
| Same payer and beneficiary owner | `BilateralOwnerSeparation` | One owner is incorrectly used for both independent authority domains |

The workflow must fail when each mutant does not produce its named counterexample. A generic TLC failure is not sufficient; the expected invariant name must be present in the trace.

## TLC configurations

- `SAPExecutionProfiles-local-atomic.cfg`
- `SAPExecutionProfiles-local-treasury.cfg`
- `SAPExecutionProfiles-owner-bilateral.cfg`
- `SAPExecutionProfiles-split-local-mutant.cfg`
- `SAPExecutionProfiles-early-treasury-mutant.cfg`
- `SAPExecutionProfiles-same-owner-mutant.cfg`

The workflow pins TLC 1.7.4 by SHA-1 and checks the three safe profiles for invariant preservation, then requires the three mutant counterexamples.

## Limitations

The model treats `ProfileMode`, `PayerOwner` and `BeneficiaryOwner` as input constants. It does not model DID signature verification, Holochain ActionHash resolution, actual source-chain author/predecessor validation, conductor commit/rollback, forks, DHT gossip, source-chain frontier discovery, cryptographic verification, the Treasury zome implementation, external bank settlement, compensation, legal discharge or physical delivery.

The DHT completeness booleans are adversarial inputs to the projection model. The model proves only that an incomplete view cannot support a successful bilateral claim under the specified reducer; it does not prove an adapter can establish that its view is complete.

## Next gates

1. Run the model on the exact PR head and retain TLC logs/artifacts.
2. Add multi-agent Sweettest tests for local source-chain transaction atomicity and chain-head movement.
3. Add a two-conductor bilateral test with concurrent owner writes, lost responses, incomplete DHT frontiers, stale checkpoints and fork/conflict observations.
4. Bind real source-action authors and predecessors into profile-qualified evidence before projecting support.

## Claim ceiling

A TLC pass proves the finite TLA+ model satisfies the enumerated invariants under its constants and transition system. It does not prove the production Rust implementation, Holochain runtime, cryptographic authority, DHT convergence, global uniqueness, Treasury delivery or external settlement.
