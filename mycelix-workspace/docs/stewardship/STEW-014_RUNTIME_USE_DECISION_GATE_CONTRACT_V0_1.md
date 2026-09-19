# STEW-014 — Runtime Use-Decision Gate Contract v0.1

## Status

Draft composition contract. No runtime authorization is implemented here.

## Purpose

STEW-014 freezes the evidence gates that must be satisfied before a STEW-012D cross-policy assessment may contribute to a consequential runtime use decision for knowledge, cultural material, protected content, or AI operations.

It is intentionally downstream of policy syntax and structural policy-authority candidates.

## Core theorem

```text
policy-set assessment
!= applicable authorities established
!= authority evidence verified
!= constraints satisfied
!= duties satisfied
!= cultural protocol satisfied
!= disclosure authorized
!= runtime authorization
```

Likewise:

```text
candidate prohibition
!= verified authority to deny
```

Negative authority requires evidence too. This prevents unverified policy objects from becoming censorship primitives.

## Exact request identity

A later executable profile should bind one request containing at least:

- exact STEW-001 representation identity;
- requesting principal;
- requested `KnowledgeUseActionV1`;
- purpose/context profile reference;
- request nonce or unique request identity;
- decision-evaluation profile/version;
- time/currentness context where relevant.

A decision for one exact request must not be silently reused for another action, representation, principal, purpose, or later currentness state.

## Required evidence planes

A positive runtime use-decision candidate must not be emitted until all applicable planes have explicit evaluated results.

### 1. Principal / delegation

The requester and relevant policy authorities must have independently evaluated identity/currentness evidence. If authority is delegated, STEW-013 scope, revocation, succession, and subdelegation rules apply.

```text
key possession != identity authority
membership != delegation
represented_collective != delegation
```

### 2. Policy applicability and authority

Every policy considered consequential must have explicit applicability and evaluated authority. Structural STEW-012B candidates are insufficient by themselves.

### 3. Cross-policy composition / precedence

If multiple applicable authorities disagree, the decision requires an admitted composition/precedence profile or remains `Conflict` / `Indeterminate`.

There is no universal newest-wins, majority-wins, most-restrictive-wins, state-primacy, community-primacy, archive-primacy, platform-primacy, or copyright-holder-primacy reducer.

### 4. Constraints

Every constraint referenced by a permission candidate must have an evaluated result under a named constraint profile.

```text
constraint reference exists != constraint satisfied
```

### 5. Duties / reciprocity

Duties must be typed by execution semantics rather than treated as one generic checkbox. A future profile should distinguish at least:

- precondition duties that must be satisfied before use;
- concurrent duties that bind the use itself;
- post-use obligations that can remain outstanding after use;
- reporting/audit duties;
- reciprocity obligations linked to STEW-010/011.

A `ReportedFulfilled` reciprocity receipt remains evidence, not verified satisfaction.

### 6. Cultural protocol

Cultural protocol evaluation remains independent of copyright, legal title, generic policy authority, and technical decryption capability.

```text
legal permission != cultural authorization
cultural authorization != factual truth
```

### 7. Protected payload

For protected representations, successful decryption capability does not itself authorize retrieval, view, disclosure, training, derivation, or redistribution.

### 8. Metadata / existence disclosure

STEW-020A/B remains an independent gate. Permission to retrieve payload bytes does not imply permission to reveal title, summary, relationships, location, community association, or even existence.

### 9. AI-specific use

AI actions remain action-specific:

```text
Retrieve != Reason
Reason != Disclose
Reason != TrainAi
TrainAi != GenerateDerivative
GenerateDerivative != Redistribute
```

A future Symthaea KNOW-AUTH layer must request the exact action rather than inferring broad authority from model access.

## Proposed decision vocabulary

A later executable child should use outcome names that preserve evidence uncertainty. Suggested v1 states:

```text
PermitCandidate
DeniedByEvaluatedPolicy
DeniedMissingPrerequisite
Conflict
Indeterminate
```

`PermitCandidate` remains distinct from the runtime enforcement event that actually releases a key, plaintext, export, derivative, model-training capability, or disclosure channel.

## Enforcement separation

```text
decision candidate
!= capability issuance
!= successful enforcement
!= irreversible downstream control
```

The authorization engine should produce a narrowly scoped decision/capability artifact; protected-content storage, Holochain coordinators, Symthaea tools, export systems, and other consumers enforce it independently.

## Audit / receipt direction

Consequential use should be able to emit an evidence-bearing use receipt containing the exact request, decision profile, evaluated policy/authority references, resulting action, triggered duties, and downstream artifact/derivative references where appropriate.

Privacy policy may require protected or selectively disclosed receipts. Auditability must not itself force sensitive community membership, sacred governance processes, or protected artifact existence into public metadata.

## Fail-closed rule

For consequential protected-content, disclosure, training, derivation, redistribution, or commercialization actions:

```text
missing evaluated prerequisite
OR unresolved applicable authority
OR unresolved cross-policy conflict
OR unsatisfied blocking constraint/duty
-> no positive runtime use-decision candidate
```

This rule does not declare which unresolved claimant or policy is substantively correct.

## Deliberate non-claims

No identity proofing, mandate verification, policy applicability, legal interpretation, cultural authority, community consent, precedence, constraint satisfaction, duty satisfaction, reciprocity discharge, decryption authority, metadata disclosure authority, AI-use permission, runtime authorization, enforcement correctness, or legal compliance is established.
