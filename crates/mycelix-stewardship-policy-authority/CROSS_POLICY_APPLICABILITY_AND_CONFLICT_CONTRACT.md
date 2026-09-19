# STEW-012C — Cross-Policy Applicability and Conflict Contract v0.1

## Purpose

Freeze the rules that must exist before multiple STEW-003 policies or STEW-012B policy-authority candidates can be reduced into one consequential runtime authorization decision.

The core boundary is:

```text
multiple policy candidates exist
!= all are applicable
!= all have equal authority
!= newest wins
!= any prohibition globally wins
!= majority wins
!= most restrictive policy automatically wins
!= runtime authorization resolved
```

STEW-003 prohibition dominance is intentionally **local to one policy object**. It must not be generalized into an implicit global precedence rule across independently authored policies.

## Four distinct questions

A later evaluator must keep these questions separate:

1. **Applicability** — does this policy govern this exact request, actor, target, action, purpose, context, jurisdiction, time, and representation?
2. **Authority** — is the policy issuer actually authorized for the relevant scope and domain?
3. **Precedence / composition** — if multiple applicable authoritative policies exist, how are they combined or ordered under an explicit profile?
4. **Runtime decision** — after constraints, duties, currentness, conflicts, and evidence are evaluated, may the requested action proceed?

```text
applicable != authoritative
authoritative != globally controlling
precedence rule exists != precedence rule legitimate
policy-set evaluation != runtime capability issuance
```

## No hidden universal precedence

V1 freezes the following as forbidden implicit reducers:

- newest-record-wins;
- oldest-record-wins;
- any-prohibition-wins across distinct policy authorities;
- majority vote;
- reputation/MATL weighting;
- token/stake weighting;
- wealth or payment weighting;
- government primacy;
- archive/institution primacy;
- community primacy;
- copyright-holder primacy;
- platform-operator primacy;
- most-restrictive-wins;
- least-restrictive-wins;
- first-writer-wins;
- last-writer-wins.

Any precedence or composition semantics must come from an explicit, versioned, evidence-bearing conflict/composition profile whose own authority and scope are evaluated separately.

## Same-policy versus cross-policy prohibition

Inside one STEW-003 policy:

```text
Prohibit(Action) + Permit(Action)
-> Prohibited
```

That is a property of the exact policy object.

Across independently authored policies:

```text
Policy A: Permit(View)
Policy B: Prohibit(View)
```

must initially become:

```text
CrossPolicyConflict
```

unless an explicit admitted composition/precedence profile resolves their relationship.

The system must not silently reuse STEW-003's local prohibition rule as a global policy reducer.

## Applicability must be request-specific

A future authorization request should bind at least:

- exact target identity;
- requested action;
- requesting principal / delegated principal;
- purpose or use-context reference where relevant;
- time/currentness context;
- execution/disclosure context;
- applicable jurisdiction or governance-profile references when used;
- policy-set snapshot / evidence lineage.

A policy-authority candidate that structurally covers a subject is not automatically applicable to every request concerning that subject.

## Distinct authority domains

Rights, cultural protocol, access/use stewardship, institutional custody, preservation, legal restrictions, and technical capability can coexist without collapsing into one scalar authority.

```text
copyright permission
!= cultural permission
!= archive custody
!= decryption capability
!= community mandate
!= legal compulsion
```

A later composition profile may require several independent authorities to be satisfied, but the generic theorem must not hard-code which real-world authority dominates every other one.

## Currentness and supersession

Two records from the same asserted authority do not resolve by timestamp alone.

A later evaluator must represent explicit relationships such as:

```text
activates
supersedes
revokes
expires
amends
coexists
conflicts
```

and carry evidence for those relationships.

```text
newer != superseding
revoked now != historical record erased
historically valid != currently applicable
```

## Conflict status vocabulary direction

A future executable child should represent, at minimum:

```text
NoApplicablePolicy
SingleApplicableCandidate
CompatiblePolicySet
CrossPolicyConflict
CompositionProfileRequired
IndeterminateAuthority
IndeterminateCurrentness
```

These are structural/evaluation states, not final moral or legal judgments.

## Fail-closed runtime boundary

For consequential protected-content, disclosure, AI-training, derivative-generation, redistribution, or commercialization actions:

```text
unresolved policy authority/conflict
-> no positive runtime authorization candidate
```

This is **not** equivalent to declaring one policy the winner. It means the system lacks sufficient resolved authority to issue a positive capability.

Historical records, disputes, and rejected candidates remain auditable rather than being erased.

## Constraints and duties

Constraint and duty references from multiple policies must remain provenance-bound to their source policy until an explicit composition profile says how they combine.

Do not silently:

- union all duties;
- discard duplicate-looking duties;
- intersect all constraints;
- treat similar labels as semantic equality;
- treat one satisfied duty as satisfying another authority's duty.

Semantic equivalence itself requires an explicit mapping/profile/evidence boundary.

## Emergency and compelled-access cases

Emergency access, court/legal compulsion, preservation rescue, or other exceptional regimes must be explicit policy/composition profiles with independent audit evidence.

They must not be implemented as hidden bypasses such as:

```text
if admin { allow }
if emergency { ignore cultural policy }
if government_request { bypass normal authority }
```

The generic layer records the asserted exceptional basis and its consequences; it does not decide that the basis is legitimate.

## Synthetic qualification corpus for an executable child

A later typed child should include at least:

1. one applicable admitted policy and one unrelated policy -> unrelated policy excluded;
2. two compatible permissions from independently admitted authorities -> compatible-set candidate, not automatic authorization;
3. permit/prohibit conflict across independent authorities -> `CrossPolicyConflict`;
4. permit/prohibit inside one STEW-003 policy -> local prohibition remains dominant;
5. newer policy without explicit supersession evidence -> both remain present;
6. explicitly superseded policy -> historical record retained but excluded from current candidate set under the applicable profile;
7. authority candidate with indeterminate currentness -> unresolved, not silently ignored or accepted;
8. same-looking duties from two authorities -> remain distinct without an explicit semantic mapping;
9. emergency override reference without admitted composition authority -> fail closed;
10. unresolved conflict on `TrainAi`, `Disclose`, or `Redistribute` -> no positive runtime authorization candidate.

## Relationship to later Symthaea authorization

Symthaea must consume the resolved authority state, not improvise precedence from retrieved text or model judgment.

```text
model sees two policies
!= model chooses the more persuasive one
```

and:

```text
model can reason about dispute
!= model may resolve dispute as authority
```

## Deliberate non-claims

This contract establishes no policy applicability, issuer authority, legal precedence, community primacy, state primacy, copyright ownership, cultural legitimacy, emergency legitimacy, currentness, conflict resolution, runtime authorization, access permission, AI-training permission, or legal compliance.
