# Integral D6S — Canonical Derivation Receipts over DKG Projections

Status: **ReferenceModelOnly**

## Purpose

D6S defines the reproducibility boundary between the persistent Mycelix Distributed Knowledge Graph (DKG) and a bounded Integral derivation.

The DKG is the semantic knowledge substrate. It may contain multiple relation types, historical states, contradictions, provenance, custody, dependencies, and legitimate cycles.

A derivation is a qualified projection of that substrate. It is not the DKG itself.

The canonical pipeline is:

```
DKG
  -> qualified projection
  -> derivation DAG
  -> canonical derivation receipt
```

## Architectural law

```
same qualified inputs
+ same semantic environment
+ same derivation profile
+ same projection/canonicalization version
= same derivation commitment
```

Changing any committed input or semantic rule changes the derivation identity.

## DKG boundary

The existing `mycelix-knowledge` implementation contains DKG claims, attestations, consensus snapshots, disputes, graph/inference surfaces, and confidence calculations.

Integral must not silently reinterpret those outputs as semantic truth.

In particular:

- attestation count != independent evidence;
- reputation != semantic authority;
- confidence score != truth;
- `get_truth` output != Integral conclusion;
- consensus snapshot != D6N/D6P qualification receipt;
- DKG reachability != semantic validity;
- graph traversal order != evidentiary precedence.

Any such output may participate only through an explicit qualified derivation profile that defines exactly what it means and what authority it carries.

## Canonical receipt inputs

A D6S receipt commits to:

1. exact input node/statement/evidence commitments;
2. exact edge commitments selected by the derivation;
3. exact D6P current-eligibility receipts when currentness is asserted;
4. exact D6N/D6O observer qualification/lifecycle context when relevant;
5. exact semantic environment;
6. exact derivation rule/profile and version;
7. exact projection/canonicalization version;
8. explicit claim ceiling;
9. resulting assessment/conclusion status;
10. contradiction and unresolved state.

The receipt MUST NOT contain an implicit input such as "all reachable nodes" unless that selection rule itself is explicitly committed.

## Semantic environment

The semantic environment binds the contextual facts that can affect derivation meaning, including as applicable:

- semantic profile/version;
- derivation profile/version;
- current frontier;
- D6P eligibility context;
- D6N/D6O observer context;
- membership/authority scope;
- dependency snapshot;
- historical cutoff;
- policy version;
- claim ceiling.

A derivation is not canonical merely because its node hashes are canonical if the semantic environment is omitted.

## Projection rules

The projection is a selected, ordered-independent set of typed nodes and edges.

Canonicalization MUST:

- sort or otherwise normalize all unordered collections deterministically;
- reject duplicate identities with conflicting content;
- bind exact endpoint identities;
- bind exact edge kinds;
- preserve contradiction and unresolved states;
- preserve historical/current distinctions;
- preserve foreign/local provenance distinctions;
- exclude unused DKG material from the derivation commitment;
- include every materialized dependency required by the derivation profile.

The projection MUST be sufficient to reconstruct the derivation without querying mutable ambient state.

## Cycle semantics

The DKG may contain legitimate cycles.

D6S therefore distinguishes:

- semantic derivation/support cycles — rejected by default;
- explicit recursive/fixpoint derivations — allowed only under a dedicated qualified profile;
- provenance/custody/history cycles — preserved as graph facts but not treated as derivation support;
- unused DKG cycles — irrelevant to the selected derivation.

A cycle in the DKG is not itself evidence that the knowledge is invalid.

## Non-amplification

```
derived semantic authority <= qualified authority of exact inputs
```

A derivation may preserve, refine, contextualize, aggregate, or narrow its inputs.

It may not silently increase:

- scope;
- currentness;
- authority;
- independence;
- endorsement;
- truth status;
- authorization;
- claim ceiling.

Missing evidence produces an unresolved/blocked result, not a rejection.

Contradiction remains contradiction unless an explicit qualified profile supplies a legitimate resolution rule.

## Reference implementation status

D6T (#3464) freezes **D6S-CANON-1** as a D6S-specific canonical JSON subset.

The profile explicitly defines:

- recursive UTF-16 code-unit ordering for object properties;
- preserved array order;
- deterministic JSON string escaping with no Unicode normalization;
- integer-only numeric representation;
- exact lowercase spellings for null and booleans;
- exact UTF-8 output;
- typed-value duplicate-property exclusion;
- explicit SHA-256 domain separation and per-object commitment labels;
- receipt self-commitment with the commitment field excluded from its own preimage.

D6S-CANON-1 is deliberately **not** described as RFC 8785/JCS-compatible. A future JCS compatibility claim requires independent primitive, Unicode, number, parser, and golden-vector verification.

## Receipt identity

Conceptually:

```
DerivationCommitment =
  H(
    schema_version,
    projection_version,
    semantic_environment_commitment,
    derivation_profile_commitment,
    canonical_input_node_commitments,
    canonical_input_edge_commitments,
    result_commitment,
    claim_ceiling
  )
```

D6T freezes SHA-256 as the reference commitment algorithm and freezes the D6S domain prefix plus per-object commitment labels. The receipt commitment excludes `receipt_commitment` itself from its preimage.

The important invariant is that every semantically material input is committed.

## Adversarial requirements

A conforming implementation must demonstrate that:

- replacing one input commitment changes the receipt;
- changing the derivation profile changes the receipt;
- changing the semantic environment changes the receipt;
- DKG reachability alone cannot create support;
- attestation count alone cannot create independence;
- heuristic confidence cannot become authority without qualification;
- provenance cannot become causal support;
- historical evidence cannot become current through projection;
- contradiction cannot disappear through traversal order;
- missing evidence cannot become rejection;
- correlated observers cannot become independent by count;
- Symthaea proposals remain non-authoritative;
- conclusions cannot authorize actuation.

## Symthaea boundary

Symthaea may propose candidate projections, identify missing dependencies, detect contradictions, search for impacted derivations, and suggest requalification targets.

Serialization, replay, ranking, or persistence of a Symthaea proposal MUST NOT itself grant semantic authority.

## Xenia boundary

Cryptographic commitment and verification establish integrity/authenticity properties defined by the cryptographic protocol. They do not independently establish semantic truth, causal validity, current authority, or authorization.

## Holochain boundary

Holochain is a realization substrate for storing, validating, linking, and distributing these objects. D6S semantics remain independent of any single storage substrate.

## Claim ceiling

**ReferenceModelOnly.**

This specification does not establish physical truth, causal validity, legal authority, production finality, economic settlement, or actuation safety.

## D6S hardening: source, frontier, and conflict conservation

The reference implementation now binds the selected projection to an exact `source_dkg_snapshot_commitment` and an explicit `D6S-CANON-1` canonicalization version. Both are receipt inputs.

For current `Supported` results:

- every projected node must be non-historical;
- every projected node must bind the exact environment current frontier;
- the environment must declare a current frontier;
- every required D6P receipt must be structurally valid, `EligibleCurrent`, exactly committed, and at the exact environment frontier;
- the projection's D6P receipt-commitment set must match the environment's D6P eligibility context root;
- D6N and D6O projection context commitments must match the semantic environment exactly.

Result flags are also constrained:

- `Supported` cannot preserve contradiction or unresolved state;
- `Disputed` must preserve contradiction;
- `Unresolved` and blocked qualification/currentness/missing-evidence states must preserve unresolved state.

This prevents a canonical receipt from turning stale or historical material into current support merely by projection, or from recording a resolved status while silently dropping the unresolved/contradictory state.

The D6S implementation remains a bounded canonicalization layer. It does not itself make DKG confidence, attestation count, reputation, consensus snapshots, or graph reachability authoritative.

