# ADMIN-003 — Administrative Review & Finality v0.1

Status: **pure review/finality qualification layer; no runtime or external-effect authority**

ADMIN-003 is additive above ADMIN-002 and is available only through the Cargo feature:

`administrative-review = ["procedural-completeness"]`

There is intentionally no configuration in which ADMIN-003 can bypass procedural completeness.

## Central separation

```text
issued administrative decision
!= admissible challenge
!= appeal/review case
!= competent review disposition
!= administrative finality
!= judicial finality
!= remedy
!= external effect authority
```

The original decision is never rewritten. Review produces a new immutable lineage around it.

## Review-policy context

`AdministrativeReviewPolicy` binds both sides of the review relationship:

- exact source procedure profile;
- exact source institution / jurisdiction / rulebook;
- exact review forum / jurisdiction / review rulebook;
- immutable policy reference + content digest/profile;
- separate review, stay, and remedy capabilities;
- accepted review roles;
- challenge and appeal windows;
- post-disposition finality delay;
- reviewer-independence requirement; and
- bounded allowed remedy-type set.

A source-case context mismatch fails before review can begin.

The pure kernel does not prove that the supplied policy reference/digest is the currently authoritative policy. Runtime/provider qualification must establish policy provenance/currentness. That remains an explicit non-claim.

## Immutable source decision

`qualify_reviewable_decision` consumes one `IssuedAdministrativeDecision` and verifies that its successor case is `DecisionIssued` with the same exact decision identity and decision time.

The resulting `QualifiedReviewableDecision` is opaque and not serializable/clonable. Review cannot detach from the exact issued decision and float onto another record.

## Challenge admissibility

`ChallengeSubmission` reuses institutional-core `Challenge` and adds only the missing provider references for standing and service.

A challenge qualifies only when:

- institutional-core protocol is exact;
- the exact challenged decision matches;
- grounds are present and bounded;
- filing does not predate the decision;
- filing is within the exact overflow-safe challenge window;
- challenge evidence is structurally valid, bounded and identity-unique;
- no challenge evidence is observed after filing; and
- standing/service proof references are present and bounded.

Standing proof authenticity and the substantive legal meaning of standing remain provider/profile responsibilities.

## Appeal/review lineage

ADMIN-003 v0.1 requires the appellant to equal the challenger. Representation, guardianship, successor-interest and counsel authority are deliberately deferred rather than represented by an untyped alternate principal.

An appeal qualifies only when:

- it references the exact qualified challenge;
- its forum and rulebook equal the review policy;
- it does not predate the challenge; and
- it is within the exact overflow-safe appeal window.

## Separate stay authority

A stay is not inferred from appeal filing.

`StayDirective` requires the exact appeal + decision and an independent authority evaluation for the policy's `stay_capability`.

The stay state is typed as:

- `NotStayed`;
- `StayedUntil(t)`; or
- `LiftedAt(t)`.

A non-expired stay cannot be silently overwritten. A lift requires an active stay. Re-imposition after expiration or a valid lift remains possible through a new directive and new authority evaluation.

A stay token still grants no external-effect authority by itself.

## Independent competent review

`AdministrativeReviewDisposition` binds:

- exact appeal and original decision;
- exact reviewer principal;
- exact authority grant;
- typed outcome (`Affirm | Reverse | Vacate | Remand | Modify`);
- bounded non-empty reasons;
- bounded exact review evidence;
- decision time;
- immutable disposition/proof references; and
- exact remedy types authorized by that disposition.

The reviewer must hold the configured `review_capability` in the review forum/jurisdiction/rulebook under institutional-core `evaluate_authority()`.

When independence is required, the reviewer may be neither the original decider nor the appellant/challenger.

Review evidence observed after the review decision fails closed.

Affirmance cannot smuggle a corrective remedy authorization in v0.1.

## Administrative finality

Administrative finality is a separate opaque type.

`AdministrativeFinalityReceipt` must bind the exact original decision, appeal, and review-disposition reference. Finality cannot close before the review disposition plus the configured finality delay.

The receipt proof is externally supplied. The pure kernel therefore proves scope/timing/type separation, not the authenticity of an authoritative no-further-internal-review registry. Runtime/provider work must prove that source.

ADMIN-003 v0.1 deliberately does **not** claim unchallenged-decision finality from local absence. Proving “no timely challenge exists” requires an authoritative closed-world challenge registry/closure receipt and is deferred rather than inferred from an empty local vector.

## Judicial finality is external

Mycelix administrative code does not mint judicial authority.

`ExternalJudicialFinalityReference` may be recorded only after administrative finality and must bind:

- exact original decision;
- exact administrative disposition;
- court/forum reference;
- exact judgment reference + non-zero content digest/profile;
- finality time; and
- proof reference.

The result is named `RecordedExternalJudicialFinality`, not `QualifiedJudicialAuthority`. It explicitly returns `grants_authority() == false` and `grants_external_effect_authority() == false`.

Court competence, judgment authenticity, hierarchy, appeal exhaustion and jurisdiction are provider/interface responsibilities.

## Remedy separation

ADMIN-003 reuses institutional-core `Remedy`.

A remedy qualifies only when:

- it targets the exact administratively final decision;
- its type was explicitly authorized by the exact review disposition;
- it does not predate administrative finality;
- its lifetime is coherent;
- its `authorized_by` grant matches the supplied grant; and
- that grant independently satisfies the policy's `remedy_capability` in the review authority context.

Review authority therefore does not automatically imply stay authority or remedy authority.

`QualifiedRemedy` is evidence for a later effect boundary; it is not itself an actuator capability.

## Resource / ambiguity bounds

v0.1 bounds:

- review reasons: 64;
- review evidence: 256;
- allowed/authorized remedy types: 32;
- references/text/profile lengths; and
- all time-window arithmetic via checked addition.

Duplicate evidence identities, duplicate review roles and duplicate remedy-type encodings fail closed.

## Deliberate non-claims

ADMIN-003 does not establish:

- authoritative review-policy sourcing/currentness;
- substantive standing;
- representative/counsel authority;
- challenge or appeal fee rules;
- equitable tolling / excusable delay;
- multi-tier administrative appeals;
- en banc review;
- discovery or evidentiary admissibility doctrine;
- substantive correctness of review reasons;
- authoritative proof that no further internal appeal exists;
- court competence or judicial hierarchy;
- cryptographic proof authenticity;
- runtime persistence;
- Holochain admission;
- service/delivery execution; or
- physical/external effects.

Those belong to profile/provider work, later multi-tier review work, judicial interfaces, and explicit effect adapters.

## Promotion rule

ADMIN-003 must remain draft/non-promotable until all of the following are true:

1. ADMIN-002 exact head is executable-qualified in default and completeness modes;
2. ADMIN-003 exact head passes default + ADMIN-002 + ADMIN-003 feature qualification;
3. current authority ancestry (#74/#75 and their qualified parent chain) is green at the ancestry actually used by the administrative branch; and
4. the administrative stack is explicitly converged with GOVSYS-002 constitutional ancestry rather than merely referencing it in prose.
