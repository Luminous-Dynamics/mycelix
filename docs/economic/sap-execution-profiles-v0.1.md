# SAP execution profiles v1: local atomic versus owner-authenticated bilateral

**Status:** pure profile constructors and synthetic semantic-boundary corpus. This is not a Holochain integrity validator, conductor test, payment adapter or settlement proof.

## Why two execution profiles exist

SAP payment execution has two different ownership/atomicity models in the public architecture. They must not share a generic `Completed` interpretation.

### 1. Single-author local atomic principal transfer

The local Finance coordinator performs the payer debit and beneficiary credit as direct calls inside one zome invocation, with Payment, Receipt and a durable Treasury fee outbox intent in the same local source-chain write set. Holochain source-chain commit atomicity means those local writes commit or roll back together; the model must not invent a debit-committed/credit-failed interleaving inside that one transaction.

The typed Economic OS effect is therefore one owner-specific role: `finance.local_atomic_payment_transaction`. It represents the local atomic source-chain transaction as one indivisible effect. It does not split into independent payer-debit and beneficiary-claim effects.

Treasury fee distribution is a different boundary. The local transaction commits a stable outbox intent; Treasury delivery occurs as a separate owner effect. A failed or unknown Treasury call must remain Pending/Indeterminate until reconciled against that same outbox-derived effect ID. A local Payment record being complete is not proof that the Treasury fee has arrived.

### 2. Owner-authenticated bilateral transfer

A separate profile represents independent payer and beneficiary source-chain effects:

- payer owner authors the immutable intent and payer debit effect;
- beneficiary owner independently authors the beneficiary claim/credit effect;
- the two roles bind distinct owners;
- each owner effect is validated against its own source-chain author, predecessor and frontier;
- the two source chains are not one atomic transaction.

This model must not use the local-atomic role as a substitute for either owner-authored leg. Missing or incomplete DHT evidence is Indeterminate, not global absence. Conflicting owner histories, stale checkpoints or forks cannot be resolved by selecting whichever result appears newest.

## Explicit execution profiles

The Rust crate exposes `ExecutionProfileV1` with three versioned profiles:

| Profile | Required roles | Execution boundary |
|---|---|---|
| `LocalAtomicPrincipalTransferV1` | `finance.local_atomic_payment_transaction` | One author-owned atomic local transaction; Treasury effect is outside this claim |
| `LocalAtomicTransferTreasuryReconciledV1` | `finance.local_atomic_payment_transaction` + `treasury_fee` | Atomic local transaction plus a separately owned Treasury fee receipt |
| `OwnerAuthenticatedBilateralTransferV1` | `payer_debit` + `beneficiary_claim` | Distinct payer and beneficiary source-chain effects |

The three profiles have unique profile IDs and claim names, which are included in the existing FIN-EFFECT-004 canonical profile digest. They reuse the existing FIN-EFFECT-001 operation and effect identities; no new operation/effect hash domain is introduced. The legacy generic SAP bilateral template remains byte-for-byte unchanged for compatibility but is marked unsuitable for new production adapter selection.

The builder requires the exact role set, rejects missing/extra/duplicate bindings, and rejects a bilateral profile when payer and beneficiary owner strings are identical. Local-atomic owner binding has one local transaction owner slot. The Treasury-reconciled profile adds a distinct role slot and receipt dependency; it does not assume the same or different author unless the selected deployment profile makes that an explicit authority constraint.

## Aggregation and recovery

These execution profiles still use the same profile-bound projection precedence:

`Conflict > Indeterminate > Pending > SupportedWithinDeclaredScope`

- **Local effect missing or incomplete:** Indeterminate.
- **Local effect not observed in a complete declared local-author view:** Pending.
- **Local atomic effect observed consistently:** supported within the local atomic profile only.
- **Treasury batch absent or incomplete:** Indeterminate for the Treasury-reconciled profile.
- **Treasury complete empty view:** Pending.
- **Treasury commit with lost acknowledgement:** reconcile the exact outbox/effect ID and identical evidence; do not mint a new ID.
- **Bilateral payer leg only or beneficiary claim only:** Pending only when the other owner view is declared complete; otherwise Indeterminate.
- **Conflicting receipt/digest or owner history:** Conflict.

## What this pure layer does not prove

The profile constructor binds names, required roles and owner identifiers into a content digest. It does not authenticate a DID, verify a signature, prove that an Holochain action was authored by the bound owner, load a predecessor record, validate the source-chain sequence/frontier, prove local zome-call atomicity in a deployed DNA, prove DHT completeness or cross-agent uniqueness, verify a Treasury balance mutation, or establish external bank settlement or legal discharge.

Those claims require the production integrity validators and actual multi-agent Sweettest/conductor qualification. A profile-selected `SupportedWithinDeclaredScope` result is only a projection over the observations supplied by the adapter.

## Adversarial corpus and targets

`sap-execution-profile-adversarial-corpus.v0.1.json` contains 15 public synthetic cases. It tests local-atomic versus bilateral role substitution, Treasury missing/unknown/duplicate acknowledgements, one-sided bilateral effects, incomplete DHT frontiers, owner fork conflicts, same-owner bilateral binding and continued use of the ambiguous legacy profile. It is a development conformance suite, not a hidden benchmark, runtime test or standards certification.

Targets (not achieved results):

- zero cross-profile promotions;
- zero payer-only/beneficiary-only effects satisfying a local-atomic role;
- zero missing/unknown Treasury outcomes promoted to reconciled;
- zero one-sided owner effects promoted to bilateral support;
- zero same-owner payer/beneficiary bilateral profiles;
- 100% digest determinism for the exact selected profile and role-owner set;
- separately measure runtime author/role/source binding false positives, fork/conflict detection, recovery correctness and p50/p95/p99 projection cost.

## Next runtime gate

Use the existing Sweettest harness to exercise (a) local source-chain atomic transaction commit/rollback and chain-head movement, (b) lost local response and same-key recovery, (c) Treasury fee outbox delivery with lost acknowledgement and destination-side exact-effect reconciliation, and (d) two distinct agents committing payer and beneficiary effects with forks, incomplete DHT views and stale checkpoint cases. Never treat a reference model or profile-constructor test as a substitute for those runtime proofs.
