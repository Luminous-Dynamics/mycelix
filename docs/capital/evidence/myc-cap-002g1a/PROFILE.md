# MYC-CAP-002G1A — Exact chamber ballot, eligibility, and recusal reconstruction v1

Status: candidate executable profile; local preflight only until hosted exact-head qualification executes.

## Purpose

Close MYC-CAP-002G1's deliberate aggregate-count limitation by reconstructing chamber quorum and approval inputs from one exact pseudonymous eligibility snapshot, one exact ballot set, and one exact conflict/recusal set.

Core separation:

```text
self-reported chamber totals
!= reconstructed chamber authority

conflict disclosed
!= conflicted ballot removed from authority
```

This theorem feeds the G1 stewardship decision gate. It does not replace G1's typed action classes, constitutional/enforcer requirements, emergency scoping, or prohibited-action rules.

## Conservative v1 membership model

v1 intentionally supports a narrow authority model:

```text
one pseudonymous participant
-> exactly one chamber
-> at most one ballot
-> one authority unit
```

Weighted voting, delegated voting, liquid democracy, transferable votes, token weighting, and multi-chamber membership are not supported by this profile.

The eligibility snapshot is semantically set-like: chamber/member ordering is normalized before the snapshot commitment is computed. Reordering the same membership does not create a new semantic eligibility state.

Duplicate members inside one chamber or the same participant appearing in multiple chambers fail closed.

## Exact eligibility currentness

The tally profile binds:

- exact project;
- exact G1 governance-profile SHA-256;
- exact eligibility-registry ID;
- one designated integer registry epoch;
- exact recusal semantics;
- bounded participant count.

The supplied eligibility snapshot must match that exact registry and epoch. A stale or substituted snapshot fails closed.

The theorem contains no wall-clock semantics. Registry/epoch currentness is supplied by the frozen profile; authenticity of the registry source belongs to a separate evidence/identity layer.

## Ballot reconstruction

Every ballot binds:

- unique ballot ID;
- project;
- G1 governance-profile digest;
- exact decision ID and action code;
- exact eligibility registry ID/epoch;
- chamber;
- pseudonymous participant reference;
- vote value;
- evidence reference.

Allowed vote values are exactly:

```text
APPROVE
REJECT
ABSTAIN
```

Unknown values and injected weighting/authority fields fail closed.

A participant must be eligible in exactly the supplied chamber and may contribute at most one ballot.

## Required recusal semantics

The frozen v1 recusal rule is:

```text
EXCLUDE_FROM_ELIGIBLE_AND_TALLY
```

For any conflict code that the parent G1 action marks as requiring recusal:

- the conflict record must be disclosed;
- the participant is excluded from the effective eligible denominator;
- the participant may not submit a ballot;
- therefore the participant cannot contribute to participation or approval/rejection/abstention counts.

This semantics is profile-defined rather than inferred from prose.

A supplied conflict record is evidence input only. Absence of a supplied conflict record is **not** proof that no real-world conflict exists.

## Reconstructed threshold inputs

For each G1-required chamber, v1 derives:

- effective eligible count;
- participating count;
- approval count;
- rejection count;
- abstention count;
- recused count;
- quorum result;
- approval-threshold result.

Threshold arithmetic reuses G1's integer parts-per-million rules. No floating-point authority math is used.

The resulting receipt says only whether all required chamber thresholds pass:

```text
chamber_threshold_state = PASS | BLOCKED
```

It deliberately fixes:

```text
governance_authorization_established = false
identity_authenticity_established = false
democratic_legitimacy_established = false
```

Overall governance authorization still belongs to G1 after consuming qualified tally evidence.

## Deterministic commitments

G1 governance profile semantic SHA-256:

`945b558049d7570cd47b52d7d822b2dff142a46b3d5fa117ed24dadbcd1059cb`

Tally profile semantic SHA-256:

`559999d245e395b4d143b7a36d883499109965db276f93142faf6f0324e75bf4`

Normalized eligibility snapshot semantic SHA-256:

`a280c7449b293d080e3a86af2cfd84a2580dafd64dc95d5ed205748827f02426`

Canonical fixture file SHA-256:

`be1470664b70078c7d3e090dcb41dc671af87f2c157c24b510572c887bf47446`

Frozen receipt file SHA-256:

`bd06c9beeec8532c3bd3d2010d14ca251772dfcff8009703e35bf04240643eea`

Verifier source SHA-256:

`ee5203544258bbe43b78738b8d67149e8edc921deada96fc9a45d1ecd85a48ea`

Regression-suite SHA-256:

`3d4f79e0e020a516dfcd096210d9f4cd0bd14a5031a8a7f9b7ca1b6844fcddf1`

## Local preflight

The stdlib suite passes **22/22** locally in a repository-shaped layout. It covers positive reconstruction, required-recusal denominator removal, recused-ballot rejection, duplicate participant/ballot rejection, ineligible and cross-chamber ballots, stale eligibility/ballot epochs, multi-chamber membership, duplicate eligibility entries, wrong-chamber/undisclosed/duplicate conflicts, reconstructed quorum and approval failures, unknown vote values, project/profile/decision substitution, authority-field injection, and deterministic/order-independent reconstruction.

Local PASS is not hosted qualification.

## Privacy boundary

This v1 uses pseudonymous participant references. It does not require publication of real-world identities.

Future privacy-preserving profiles may replace explicit membership/ballot records with commitments, selective disclosure, threshold tally receipts, or ZK membership/one-vote proofs. Those mechanisms should strengthen the privacy surface without changing G1's authority semantics.

## Nonclaims

Even a hosted PASS would establish only deterministic chamber-tally reconstruction over one frozen eligibility/ballot/recusal profile and supplied evidence. It would not establish real-world identity authenticity, ballot secrecy, coercion resistance, absence of undisclosed conflicts, democratic legitimacy, legal validity, social consensus, or wisdom of the decision.
