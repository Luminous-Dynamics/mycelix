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

Its new semantic responsibilities are administrative case identity, procedure profile, replayed pre-decision lineage, exact case-to-decision binding, and competent-decision qualification.

## Public-surface hardening

The crate root is `src/lib_hardened.rs`.

The original ADMIN-001 implementation remains in `src/lib.rs` as a **private module**. Its lower-level structural functions are used internally and retain their focused tests, but they are not directly exported as the public decision-qualification API.

The public theorem is:

```text
deserializable AdministrativeCase snapshot
!=
qualified administrative case lineage
```

A caller cannot present an arbitrary syntactically valid `ReadyForDecision` snapshot and call the raw decision qualifier.

Instead the public path requires:

```text
exact Filed root
+ bounded ordered PreDecisionTransition sequence
+ one decision policy captured at lineage qualification
-> QualifiedAdministrativeCaseLineage
```

`QualifiedAdministrativeCaseLineage` is opaque and is intentionally not `Clone`, `Serialize`, or `Deserialize`.

Persisted state must therefore be requalified from the exact `Filed` root and transition lineage before it can contribute to a consequential decision.

## v0.1 lifecycle

```text
Filed
  -> EvidenceOpen
  -> ReadyForDecision
  -> DecisionIssued
```

The private structural transition kernel can produce only the first two edges. It has no representation for `ReadyForDecision -> DecisionIssued`.

The public final edge exists only through:

```text
QualifiedAdministrativeCaseLineage(current = ReadyForDecision)
+ exact AdministrativeDecisionEnvelope
+ exact AuthorityGrant
+ authority evidence
-> QualifiedAdministrativeDecision
-> issue_qualified_decision(...)
-> DecisionIssued
```

A state enum value is not authority, and a deserialized snapshot is not a lineage proof.

## Exact case context

Every case binds:

- one case ID;
- one institution;
- optional jurisdiction;
- one exact rulebook;
- one procedure-profile ID;
- one subject reference;
- filer identity;
- filing time; and
- current structural state.

Institution, jurisdiction, rulebook, procedure profile, subject, filer, and filing time cannot drift during pre-decision progression.

## Decision-policy binding

The exact `AdministrativeDecisionPolicy` supplied when the lineage root is qualified is retained inside the opaque lineage token.

The later public `qualify_administrative_decision` function does **not** accept a replacement decision policy. This prevents a caller from replaying the same ready case while substituting a weaker same-profile policy at decision time.

ADMIN-001 does **not yet** establish that the initially supplied policy is the institutionally authoritative semantic definition for that procedure-profile ID. A runtime/profile-provider layer must eventually bind the profile ID to an immutable policy/version/content identity. Until that exists, policy-source authenticity remains an explicit non-claim rather than being hidden behind the profile name.

## Competent decision theorem

A decision can qualify only when all of the following hold:

1. the public case token was replayed from an exact `Filed` root;
2. its current case is exactly `ReadyForDecision`;
3. the envelope names the exact case;
4. envelope jurisdiction equals case jurisdiction;
5. institutional `Decision.institution` equals case institution;
6. `Decision.rulebook` equals case rulebook;
7. `Decision.subject_ref` equals case subject;
8. the retained decision-policy procedure profile equals case procedure profile;
9. decision time is not earlier than readiness;
10. envelope authority-grant ID equals the supplied grant;
11. envelope decider equals the actual grant holder;
12. `evaluate_authority` permits the exact capability/role/evidence requirement at decision time; and
13. decision evidence is structurally valid and contains no duplicate evidence identity.

No `authorized=true`, `official=true`, reputation score, Phi value, model recommendation, stake value, caller-chosen positive enum, or deserialized ready snapshot can substitute for the grant and lineage qualification.

## Bounded replay

ADMIN-001 has exactly two possible pre-decision structural transitions, so public lineage replay accepts at most two transition records.

This gives the pure kernel a closed fan-in bound and prevents a duplicate-transition flood from becoming a hidden qualification resource path.

Later ADMIN revisions may add richer procedural events, but they should retain explicit bounds appropriate to their evidence model.

## Deliberate v0.1 limits

ADMIN-001 does not yet prove:

- the initial procedure-policy source is authoritative for the named profile;
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

Those remain ADMIN-002/003 and later runtime/profile-provider boundaries.

## Effect boundary

`QualifiedAdministrativeCaseLineage`, `QualifiedAdministrativeDecision`, and `IssuedAdministrativeDecision` do not grant external-effect authority.

A qualified permit decision, benefit decision, tax decision, procurement-protest decision, licensing decision, or other administrative result may become an input to later effect authority. It is not itself the actuator capability.

## GOVSYS relationship

Semantically this crate is governed by GOVSYS-002 / PR #784, particularly PI-001 through PI-008, PI-011, PI-016, PI-017, PI-019, and PI-020.

Its current executable ancestry is intentionally the authority stack ending at PR #75, because that is where `mycelix-institutional-core` and current grant identity/freshness work live today. A later promotion/convergence gate must preserve both authority ancestry and GOVSYS-002 constitutional ancestry; this PR does not pretend that convergence has already happened.
