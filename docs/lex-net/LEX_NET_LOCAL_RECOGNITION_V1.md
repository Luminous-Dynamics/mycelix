# LEX-NET Local Recognition Profile v1

Status: executable research contract; no legal or external-effect authority claimed.

Qualified parent: `497182225c76fe4db6531422f7476dcef6b25365` (LEX-NET-000 R3 exact-head qualification PASS).

## Purpose

LEX-NET-001 defines the local admission decision that sits between authenticated foreign institutional evidence and any local use of that evidence.

The governing theorem is:

```text
foreign authenticated evidence
+ explicit local recognition policy
+ explicit purpose/resource context
+ explicit currentness evidence
    -> bounded locally recognized evidence projection
    != local authority
    != external-effect authority
```

Recognition is a technical local policy result. It is not diplomatic recognition, legal recognition, treaty recognition, regulatory approval, enforceability, or proof that the foreign claims are substantively true.

## Scope

This tranche freezes a pure evaluator contract and golden corpus. It intentionally does not add a runtime authority type, network resolver, trust registry, transport provider, capability token, effect adapter, or legal-rule engine.

The evaluator operates only over frozen inputs supplied to it. It does not fetch registries, consult an ambient clock, infer legal applicability, or select a local policy on behalf of the caller.

## Inputs

### Foreign evidence descriptor

The caller supplies a descriptor whose fields identify the already-obtained evidence without granting it trust:

- exact evidence commitment;
- origin domain/federation/institution profile;
- issuer identity or issuer-class evidence reference;
- external evidence profile identity and version;
- currentness/freshness evidence state;
- present claim names or selectors;
- references to any required supplementary evidence.

Authentication/provenance of the foreign artifact belongs to an upstream verifier. LEX-NET-001 consumes that result as evidence; it does not convert authentication into local authority.

### Local recognition policy

The local policy input binds:

- exact policy identity/version/commitment;
- exact supported foreign evidence profiles and versions;
- exact recognized issuer or issuer classes for this policy;
- exact permitted purpose/resource pair;
- exact claim projection that may be recognized;
- required supplementary evidence;
- reservations/exclusions;
- explicit currentness/freshness requirements.

The active policy must be selected by the legitimate local policy/authority layer. A foreign object or request caller may not substitute a more permissive recognition policy.

### Evaluation context

The caller supplies the exact local purpose/resource context and currentness evidence. Time-sensitive evaluation must use explicit evidence supplied to the deterministic evaluation boundary; no ambient wall clock is part of the pure contract.

## Dispositions

LEX-NET-001 freezes exactly five high-level dispositions:

- `RecognizedEvidence` — a non-empty bounded claim projection is locally recognized for the exact purpose/resource under the exact policy;
- `NeedsAdditionalEvidence` — the profile is otherwise potentially admissible but a frozen required evidence item is absent;
- `Unsupported` — the foreign evidence profile/version is outside the exact supported profile set;
- `Rejected` — the evidence is definitively outside the selected local policy (for example unrecognized issuer, wrong purpose, expired evidence, or prohibited recognition path);
- `Indeterminate` — a required state such as policy availability or currentness cannot be established.

`Indeterminate` and `NeedsAdditionalEvidence` never promote to positive recognition by default.

## LR-001 — foreign authentication is input only

A valid foreign signature, transparency receipt, registry entry, credential proof, or foreign authorization may establish facts about origin context. It does not directly create local recognition or authority.

## LR-002 — recognition policy is local

Recognition is evaluated against one exact local policy identity/version. The foreign principal, transport provider, caller, AI model, or remote registry may not choose a substitute policy.

## LR-003 — recognition is purpose- and resource-bound

A positive result is valid only for the exact purpose/resource pair that was evaluated. Recognition for one customs submission does not imply recognition for banking, health, immigration, procurement, identity administration, or another shipment.

## LR-004 — recognition is projection-bound

A positive result recognizes only the explicit intersection of:

- claims actually present in the foreign evidence;
- claims requested for the exact local use;
- claims permitted by the local policy.

One safe recognized field never promotes the whole foreign object into trusted local state. Unrecognized extra fields remain foreign evidence.

## LR-005 — recognition is currentness-bound

Expired evidence is rejected under the frozen reference policy. Unknown currentness is `Indeterminate`, never positive recognition. Later tranches may carry richer currentness/freshness profiles, but may not weaken this fail-closed distinction by implication.

## LR-006 — missing evidence never becomes positive

If the selected profile requires supplementary evidence and it is absent, the result is `NeedsAdditionalEvidence`. The evaluator does not infer the missing fact from reputation, similarity, neighboring records, AI output, or prior transactions.

## LR-007 — recognition is directed and non-transitive

A direct local recognition policy may recognize evidence from a foreign origin. Recognition may not be inferred transitively through another domain.

```text
A recognizes B
+ B recognizes C
!= A recognizes C
```

Reciprocal/bilateral/multilateral agreements remain a later explicit profile and still do not create transitive recognition unless the exact local policy says so.

## LR-008 — policy substitution is rejected

Caller-selected or foreign-selected policy substitution is a rejection in this contract. Policy negotiation may select only among locally pre-authorized profiles in a later tranche.

## LR-009 — recognition receipt is evidence, not authority

A recognition result may bind the exact evidence commitment, policy identity/version, purpose/resource, recognized claim projection, currentness inputs, disposition, and reason codes.

It must also state that it grants neither local authority nor external-effect authority. A recognition receipt is not a reusable capability token.

## LR-010 — evaluator is pure over frozen inputs

The reference evaluator performs no network access, registry lookup, ambient-clock read, AI inference, or external side effect. Different current external data requires a new frozen input set and a new evaluation receipt rather than silently changing a historical result.

## Reference evaluation order

For the frozen v1 corpus, evaluate in this order:

1. required local policy available;
2. local policy selection provenance acceptable;
3. direct recognition path (no transitive inference);
4. foreign profile/version supported;
5. issuer recognized by the selected policy;
6. currentness/freshness state acceptable;
7. exact purpose/resource permitted;
8. required supplementary evidence present;
9. claim projection non-empty;
10. emit `RecognizedEvidence` with only the permitted projection.

The order is part of the frozen reference oracle because it determines the first explicit failure reason. Future profiles may introduce richer reason sets only through a versioned change.

## Recognition receipt minimum

A positive receipt should bind at minimum:

- evidence commitment;
- origin/issuer/profile/version evidence references;
- local recognition-policy identity/version/commitment;
- exact purpose/resource;
- explicit currentness evaluation input;
- recognized claim projection;
- disposition and deterministic reason;
- supplementary evidence references used;
- `grants_local_authority = false`;
- `grants_external_effect_authority = false`.

Negative and indeterminate receipts should preserve enough of the same context to reconstruct why positive recognition was not established without unnecessarily copying protected payload data.

## Explicit nonclaims

A LEX-NET-001 PASS does not establish:

- truth of the foreign claims;
- diplomatic or legal recognition;
- treaty or convention applicability;
- regulatory approval or compliance;
- identity truth beyond the frozen evidence inputs;
- enforceability of a foreign act;
- local `AuthorityGrant`;
- a local capability lease;
- business acceptance;
- external-effect authority;
- production readiness.

Recognized evidence is still evidence. Any consequential local action must traverse the ordinary local authority and effect boundaries.