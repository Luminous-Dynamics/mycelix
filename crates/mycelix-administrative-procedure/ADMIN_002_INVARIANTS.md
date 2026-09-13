# ADMIN-002 — Procedural Completeness v0.1

Status: **pure procedural-completeness qualification layer**

ADMIN-002 strengthens ADMIN-001 without replacing its authority theorem.

## Central theorem

```text
qualified case lineage
!=
procedurally complete case
!=
competent administrative decision
!=
external effect authority
```

ADMIN-001 remains responsible for exact filing lineage, case context, retained decision policy, and competent authority. ADMIN-002 adds the evidence required to say that the configured procedural opportunities existed before a decision was qualified.

## Public API gating

The crate root is `src/lib_admin_002.rs`.

ADMIN-001 is imported privately through:

```text
#[path = "lib_hardened.rs"]
mod admin001;
```

The public ADMIN-002 root re-exports safe case/lineage types and `qualify_case_lineage`, but it does **not** re-export ADMIN-001's raw consequential decision qualifier or issuance function.

The only public consequential path is:

```text
QualifiedAdministrativeCaseLineage
+ exact ProceduralCompletenessPolicy
+ exact notice receipts
+ exact response-opportunity receipts
+ exact EvidenceClosureReceipt
-> QualifiedProcedurallyCompleteCase

QualifiedProcedurallyCompleteCase
+ AdministrativeDecisionEnvelope
+ AuthorityGrant
+ authority evidence
-> QualifiedProceduralDecision
-> IssuedAdministrativeDecision
```

Both ADMIN-002 positive types are opaque and intentionally not `Clone`, `Serialize`, or `Deserialize`.

## Exact policy capture

`ProceduralCompletenessPolicy` binds:

- one procedure-profile ID;
- immutable policy reference;
- non-zero externally supplied policy content digest;
- exact digest profile;
- exact required notice-recipient set;
- exact required response-recipient set;
- required response mode;
- minimum response window; and
- reasons requirement.

The policy object is retained inside the opaque completeness token and cannot be replaced at decision qualification.

The pure kernel does **not** prove that this policy object is the institution's current authoritative policy. Runtime/provider work must eventually bind the policy reference/content identity to an authoritative source and currentness generation. This remains an explicit non-claim.

## Notice semantics

Notice qualification is exact-set based.

- every required recipient must have exactly one notice receipt;
- unrequested extra recipients fail the exact-set comparison;
- duplicate recipient receipts fail closed;
- every receipt must bind the exact case;
- notice must be served no earlier than case filing and no later than readiness;
- notice content identity must be a non-zero digest; and
- receipt proof references must be non-empty and bounded.

ADMIN-002 proves structural/provenance completeness. It does not interpret whether the notice text was legally adequate; the owning procedure profile/provider owns that substantive interpretation.

## Opportunity-to-respond semantics

A policy may require written response, hearing, either, or no response opportunity.

When response is required:

- response recipients must be a subset of notice recipients;
- every required recipient must have exactly one opportunity receipt;
- the opportunity cannot open before filing or before that recipient's notice;
- the window must be at least the policy minimum;
- the mode must satisfy the policy; and
- the opportunity must close no later than the evidence cut and readiness.

An opportunity receipt proves an opportunity was qualified. It does not claim the recipient actually responded, waived participation, or that a hearing was substantively fair.

## Evidence closure

`EvidenceClosureReceipt` binds one exact case, closure time, proof reference, and exact decision-evidence cut.

The cut is order-independent but identity/content exact:

- evidence IDs must be unique;
- every evidence record must pass institutional structural validation;
- evidence count is bounded; and
- evidence observed after the closure instant is rejected.

The eventual `Decision.evidence` set must equal this closed cut exactly. A decision cannot silently omit evidence from the qualified cut, add later evidence, or substitute changed evidence under the same case.

## Reasons

v0.1 supports two explicit policy states:

- reasons not required by this profile; or
- at least one reason required.

When required, an empty decision-reasons set fails closed before ADMIN-001 authority qualification.

This is a completeness theorem only. ADMIN-002 does not establish that reasons are persuasive, legally sufficient, truthful, unbiased, or substantively correct.

## Authority preservation

ADMIN-002 does not evaluate governmental legitimacy or invent authority.

After completeness qualification, the exact decision is still passed through private ADMIN-001, which requires its replayed lineage, retained decision policy, exact case context, exact grant holder, exact institutional capability/role/evidence requirement, and decision-time `evaluate_authority()` result.

Thus:

```text
procedural completeness != authority
```

and:

```text
procedural completeness + authority != successful external effect
```

## Resource bounds

v0.1 bounds:

- required notice recipients: 64;
- required response recipients: 64;
- notice receipts: 64;
- response-opportunity receipts: 64; and
- decision evidence records: 256.

Exact duplicate semantic inputs fail rather than amplifying qualification.

## Deliberate non-claims

ADMIN-002 does not establish:

- authoritative procedure-policy source/currentness;
- legal adequacy of notice content;
- actual participation or waiver;
- hearing impartiality or evidentiary admissibility rules;
- discovery rights;
- substantive sufficiency of evidence;
- substantive adequacy of reasons;
- service of the final decision;
- reconsideration;
- appeal eligibility/timeliness;
- stays;
- typed administrative or judicial finality;
- remedy authority;
- runtime persistence;
- cryptographic proof authenticity;
- Holochain admission; or
- external/physical effects.

Those remain profile/provider work, ADMIN-003, and later runtime/effect adapters.

## Next

ADMIN-003 should reuse institutional-core `Challenge`, `Appeal`, and `Remedy` rather than inventing replacements. Its theorem should preserve the original decision immutably while adding reconsideration/review lineage, stay state, independent review authority, and typed administrative/judicial finality.
