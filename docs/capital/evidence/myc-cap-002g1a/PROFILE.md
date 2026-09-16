# MYC-CAP-002G1A — Exact chamber ballot, eligibility, and recusal reconstruction v1

Status: candidate executable profile; hosted qualification binds the exact Git subject, exact parent, exact six-file scope, deterministic receipt bytes, and strong nonclaims.

## Purpose

Close G1's deliberate aggregate-count limitation by reconstructing chamber quorum and approval inputs from one exact pseudonymous eligibility snapshot, ballot set, and conflict/recusal set.

Core separation:

```text
self-reported chamber totals != reconstructed chamber authority
conflict disclosed != conflicted ballot removed from authority
```

G1A feeds G1; it does not replace G1's action classes, constitutional/enforcer rules, emergency scope, or prohibited-action policy.

## Conservative v1 authority model

```text
one pseudonymous participant
-> exactly one chamber
-> at most one ballot
-> one authority unit
```

Weighted/delegated/token voting, transferable votes, and multi-chamber membership are unsupported and fail closed where represented.

Eligibility membership and other set-like inputs are canonicalized so harmless ordering changes do not alter semantic commitments.

## Currentness and recusal

The tally profile binds the exact project, G1 profile digest, eligibility registry ID/epoch, recusal semantics, and bounded participant count.

The v1 recusal rule is:

```text
EXCLUDE_FROM_ELIGIBLE_AND_TALLY
```

A required conflicted participant is removed from the effective eligible denominator and may not submit a ballot. Absence of a supplied conflict record is not proof that no real-world conflict exists.

## Output boundary

For each G1-required chamber the verifier reconstructs eligible, participating, approval, rejection, abstention, and recused counts plus quorum/approval results.

The receipt exposes:

```text
chamber_threshold_state = PASS | BLOCKED
governance_authorization_established = false
identity_authenticity_established = false
democratic_legitimacy_established = false
```

Overall governance authorization still belongs to G1.

## Semantic commitments

G1 governance profile semantic SHA-256:

`945b558049d7570cd47b52d7d822b2dff142a46b3d5fa117ed24dadbcd1059cb`

Tally profile semantic SHA-256:

`559999d245e395b4d143b7a36d883499109965db276f93142faf6f0324e75bf4`

Normalized eligibility snapshot semantic SHA-256:

`a280c7449b293d080e3a86af2cfd84a2580dafd64dc95d5ed205748827f02426`

Reviewed implementation/test/fixture/receipt bytes are committed by the exact Git subject and exact six-file diff scope. The generated receipt must reproduce byte-for-byte from the checked-in fixture; manually duplicated source-file hash literals are not a separate authority plane.

## Test surface

The stdlib suite contains **22** fail-closed regressions covering positive reconstruction, required-recusal denominator removal, duplicate/recused/ineligible/cross-chamber ballots, stale registry epochs, membership duplication, conflict substitution/disclosure, reconstructed threshold failures, unknown vote values, project/profile/decision substitution, authority-field injection, and deterministic/order-independent reconstruction.

The hosted runner previously exposed malformed `assertRaisesRegexht` calls during `py_compile`; this candidate repairs those test-harness typos without changing verifier semantics.

## Privacy and nonclaims

Pseudonymous participant references do not authenticate real-world identities. Even hosted PASS would establish only deterministic tally reconstruction over the supplied frozen evidence/profile. It would not establish ballot secrecy, coercion resistance, absence of undisclosed conflicts, democratic legitimacy, legal validity, social consensus, governance authorization, or wisdom.
