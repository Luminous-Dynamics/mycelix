# ADMIN-001 — Administrative Procedure Core v0.1

Status: **pure semantic qualification kernel**

ADMIN-001 adds the first executable administrative-case layer above `mycelix-institutional-core`.

## Reuse, do not duplicate

The parent institutional core already owns:

- `AuthorityGrant` / `AuthorityRequirement` / `evaluate_authority`;
- institution / jurisdiction / rulebook / principal identifiers;
- `EvidenceRef`;
- `Decision`;
- `Challenge`;
- `Appeal`;
- `Remedy`; and
- advisory-signal separation.

ADMIN-001 therefore does **not** introduce another generic decision, evidence, authority, challenge, appeal, or remedy vocabulary.

Its only new semantic responsibilities are administrative case identity, procedure profile, pre-decision structural state, exact case-to-decision binding, and competent-decision qualification.

## v0.1 lifecycle

```text
Filed
  -> EvidenceOpen
  -> ReadyForDecision
  -> DecisionIssued
```

`qualify_pre_decision_transition` can produce only the first two edges. It has no representation for `ReadyForDecision -> DecisionIssued`.

The final edge exists only through:

```text
ReadyForDecision
+ exact AdministrativeDecisionEnvelope
+ exact AuthorityGrant
+ AdministrativeDecisionPolicy
+ authority evidence
-> QualifiedAdministrativeDecision
-> issue_qualified_decision(...)
-> DecisionIssued
```

A state enum value is not authority.

## Exact case context

Every case binds:

- one case ID;
- one institution;
- optional jurisdiction;
- one exact rulebook;
- one exact procedure profile;
- one subject reference;
- filer identity;
- filing time; and
- current structural state.

Institution, jurisdiction, rulebook, procedure profile, subject, filer, and filing time cannot drift during pre-decision progression.

## Competent decision theorem

A decision can qualify only when all of the following hold:

1. the case is exactly `ReadyForDecision`;
2. the envelope names the exact case;
3. envelope jurisdiction equals case jurisdiction;
4. institutional `Decision.institution` equals case institution;
5. `Decision.rulebook` equals case rulebook;
6. `Decision.subject_ref` equals case subject;
7. decision policy procedure profile equals case procedure profile;
8. decision time is not earlier than readiness;
9. envelope authority-grant ID equals the supplied grant;
10. envelope decider equals the actual grant holder;
11. `evaluate_authority` permits the exact capability/role/evidence requirement at decision time; and
12. decision evidence is structurally valid and contains no duplicate evidence identity.

No `authorized=true`, `official=true`, reputation score, Phi value, model recommendation, stake value, or caller-chosen positive enum can substitute for the grant evaluation.

## Deliberate v0.1 limits

ADMIN-001 does not yet prove:

- notice was required or served;
- opportunity to respond/hearing requirements were satisfied;
- reasons are substantively adequate or mandatory;
- evidence completeness under a domain rulebook;
- service of the decision;
- reconsideration;
- appeal eligibility/timeliness;
- stays;
- administrative or judicial finality;
- remedy authority;
- current generation-bound grant freshness beyond what the supplied parent grant/evidence establish;
- cryptographic authenticity of `decision_proof_ref`;
- runtime persistence;
- Holochain admission;
- frontend behavior; or
- any external/physical effect.

Those remain ADMIN-002/003 and later runtime boundaries.

## Effect boundary

`QualifiedAdministrativeDecision` and `IssuedAdministrativeDecision` permanently report that they do **not** grant external-effect authority.

A qualified permit decision, benefit decision, tax decision, procurement-protest decision, licensing decision, or other administrative result may become an input to later effect authority. It is not itself the actuator capability.

## GOVSYS relationship

Semantically this crate is governed by GOVSYS-002 / PR #784, particularly PI-001 through PI-008, PI-011, PI-016, PI-017, PI-019, and PI-020.

Its current executable ancestry is intentionally the authority stack ending at PR #75, because that is where `mycelix-institutional-core` and current grant identity/freshness work live today. A later promotion/convergence gate must preserve both authority ancestry and GOVSYS-002 constitutional ancestry; this PR does not pretend that convergence has already happened.
