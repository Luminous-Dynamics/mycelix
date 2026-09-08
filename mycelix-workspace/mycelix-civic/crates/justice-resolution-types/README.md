# Justice Resolution Types v0.1

`justice-resolution-types` is the transport-neutral semantic vocabulary for final outcomes owned by Mycelix Civic Justice.

It exists because the active Justice arbitration zome already models decisions, remedies, appeals, and enforcement, but its current runtime records are not yet strong enough to be treated directly as a Business terminal expectation. In particular, downstream code must not strengthen a caller-visible `finalized` flag, optional monetary fields, or raw runtime representation into institutional finality.

## Ownership boundary

Justice owns adjudicated dispute/remedy meaning.

Governance may own institutional policy, jurisdictional authority, delegation, or genuinely Governance-native resolutions.

Finance owns execution, payment/refund/settlement, and reconciliation.

Business owns orchestration and terminal equality/closure qualification.

The intended ordinary dispute chain is:

```text
Justice case / evidence / arbitration
        ↓
Justice decision + remedy
        ↓
Justice-owned finality verification
        ↓
verified JusticeResolutionOutcomeV1
        ↓ versioned cross-domain bridge
Finance execution expectation / retained exception expectation
        ↓
actual domain result
        ↓
Business terminal equality + closure qualification
```

This crate defines only the semantic result vocabulary. It does not prove that a record exists, that arbitrators were valid, that quorum was satisfied, that an appeal is absent/resolved, or that a decision is institutionally authoritative.

## Monetary remedy, not Finance refund

`FinalMonetaryRemedyV1` binds:

- exact case reference;
- exact decision reference;
- exact remedy reference;
- Justice-owned monetary remedy kind;
- exact institutional subject reference;
- opaque expected logical effect identity;
- responsible-party reference;
- beneficiary-party reference;
- exact semantic unit;
- integer `u128` amount;
- explicit finality basis.

The v0.1 monetary remedy kinds are deliberately narrow:

- `Compensation`;
- `Restitution`.

Neither is itself a Finance operation. In particular, Justice `Restitution` does **not** automatically mean `finance.refund`. A future cross-domain mapping must prove that the exact remedy kind, subject, parties, unit, amount, and applicable bridge profile justify that Finance operation.

The `subject_ref` is part of material semantics. It identifies the exact order, agreement, obligation, prior economic effect, or other institutional subject to which the remedy applies. A downstream bridge must not infer this from free-form reasoning or remedy description.

The amount is an integer quantity in an exact semantic unit. The vocabulary does not assume currency decimal rules. A unit may therefore be something explicit such as `USD-cent`, `credits`, or another domain-qualified unit.

## Explicit finality

A bare `finalized: bool` is insufficient for downstream institutional qualification.

`JusticeFinalityBasisV1` distinguishes:

- `NoAppealPermitted { policy_ref }`;
- `AppealWindowExpired { appeal_deadline_unix_ms, qualified_at_unix_ms, no_live_appeal_evidence_ref }`;
- `AppealResolved { appeal_ref, appeal_resolution_ref }`;
- `ConsentFinal { settlement_ref }`.

The appeal-window form fails structurally when qualification precedes the deadline and always requires an exact no-live-appeal evidence reference. A future Justice verifier must still prove the referenced policy/evidence/appeal/settlement is authentic and applicable.

## Retained exception disposition

`FinalRetainedExceptionDispositionV1` binds the exact institutional subject plus exact domain-scoped exception identity that a final Justice disposition permits to remain unresolved.

Retaining an exception is not satisfaction. Business closure must preserve that distinction, and the subject binding prevents an exception from one dispute/order from being substituted into another.

## Deterministic material

The monetary result exposes two deterministic v1 representations:

- `canonical_effect_material_v1()` — exact remedy kind, subject, parties, unit, amount, and effect identity;
- `canonical_resolution_material_v1()` — effect semantics plus the exact finality basis.

All variable text uses UTF-8 byte-length prefixes, preventing delimiter ambiguity.

Changing amount, remedy kind, subject, party, unit, or effect identity changes effect material. Changing only finality evidence leaves the economic/remedy effect unchanged but changes the full resolution material.

## Semantic profiles

The v0.1 profiles are:

- `justice.final-monetary-remedy@1`
- `justice.final-retained-exception-disposition@1`

Material semantic changes require a new version. Historical v1 values retain their original interpretation.

## Runtime gap

The current Justice arbitration runtime still needs an owning verifier/adoption layer that proves, among other things:

- exact decision/remedy correspondence;
- adjudicator/panel membership and decision rule/quorum;
- exact remedy kind;
- exact subject rather than free-form interpretation;
- required monetary fields and beneficiary semantics;
- appeal/finality state from authoritative Justice history rather than a self-asserted flag;
- immutable/superseding historical lineage.

Only after that verifier exists should a production bridge treat a `JusticeResolutionOutcomeV1` as an authoritative Justice result.
