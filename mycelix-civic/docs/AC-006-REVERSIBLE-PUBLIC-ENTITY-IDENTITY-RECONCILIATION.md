# AC-006 — Reversible Public-Entity Identity Reconciliation

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-006 reconciles public/legal-entity records across standards and publishers without destructively merging their AC-003 identities.

The core model is:

`source identity -> AC-005 policy-qualified identifier observation -> link proposal -> authoritative evidence -> review -> structurally aggregation-eligible link`

Source nodes remain intact throughout.

## Public authority boundary

The low-level matcher is deliberately crate-private.

External callers use the public `IdentityResolutionContract` facade in `qualified_identity_resolution`, which requires each input to contain both:

- the `StandardsIngestionResult`; and
- the exact `StandardsIngestionPolicy` that authorized the result's public/legal-entity identifier schemes.

Before reconciliation, the facade re-checks:

1. the policy reference is present;
2. `result.policy_ref == policy.policy_ref`;
3. every configured allowlist scheme is non-empty and canonical with respect to surrounding whitespace;
4. the source evidence has a stable source reference, digest-shaped content hash and validation receipt reference;
5. every emitted `EntityIdentifierBinding.scheme` is actually present in the supplied policy allowlist.

This prevents a caller from fabricating a public `StandardsIngestionResult` containing a private or non-approved identifier scheme and bypassing AC-005's deny-by-default export policy.

The internal matcher remains available to sibling modules inside `civic-types`, but it is not re-exported as part of the public crate API.

## Eligible subjects

The initial reconciliation path accepts only AC-003:

- `Organization`; and
- `LegalEntity`.

`PrivatePersonCredential` is explicitly ineligible. AC-006 has no person-matching or person-profile path.

## Exact identifier proposals

Automatic matching uses exact `(scheme, identifier)` equality only after the scheme has passed the AC-005 policy boundary.

There is no authoritative fuzzy-name, address, embedding, social-graph or reputation matching API.

Automatic matching can create only `Proposed` links.

If one source assigns the same exact identifier to multiple different nodes, AC-006 treats that as ambiguity and fails closed.

At least two distinct source references must contribute to a cross-node proposal.

## Link states

- `Proposed`
- `Corroborated`
- `Challenged`
- `Rejected`
- `Superseded`

A structurally valid `Corroborated` link requires evidence for both endpoints, distinct source observations, authoritative verification that supports those endpoint identifiers, explicit review provenance/rationale, reversibility and no unresolved challenge.

## Authoritative verification records

The current evidence vocabulary supports:

- authoritative registry lookup for one exact scheme + identifier; and
- authoritative crosswalk between public identifier systems.

Verification records carry verifier reference, source provenance, content hashes and verification time.

These fields are **evidence claims**, not cryptographic proof by themselves.

That distinction is intentional: AC-006 establishes the structural contract for qualification evidence, but the core crate does not claim to cryptographically verify registry signatures, reviewer authority or Xenia attestations.

A later public deployment boundary must inject a verifier capable of validating those external trust roots before any identity equivalence is treated as deployment-authoritative.

## Structural aggregation eligibility

The internal AC-006 engine can determine whether a link is structurally eligible for aggregation under this contract.

That means:

- all required fields and evidence relationships are coherent;
- the status is `Corroborated`;
- no unresolved challenge exists;
- the evidence supports the endpoints;
- the link remains reversible.

It does **not** mean the external evidence has been cryptographically authenticated.

The public facade therefore exposes validation/proposal functionality without pretending that structural validity alone is a final trust decision.

## Challenge and reversal

An unresolved challenge makes corroborated use invalid immediately.

Rejected and superseded states require review provenance. Supersession must point to a different replacement link.

Because links are separate records rather than merged nodes, correction changes the reconciliation view rather than rewriting source history.

## Failure semantics

AC-006 fails closed for, among other cases:

- missing/mismatched AC-005 policy references;
- identifier bindings whose schemes were not actually allowlisted;
- malformed source evidence envelopes;
- private-person identifier bindings;
- bindings whose node is absent from the AC-005 result;
- conflicting node kinds for one node ID;
- duplicate exact identifier assignment within one source;
- links between the same endpoint;
- non-reversible links;
- endpoint/evidence mismatches;
- corroboration without distinct source observations;
- corroboration without supporting authoritative verification;
- corroboration without review provenance;
- corroboration with an unresolved challenge;
- malformed verification provenance;
- self-supersession.

## Non-goals

AC-006 does not:

- reconcile natural persons;
- trust fuzzy matching;
- automatically promote proposals;
- choose a canonical source node;
- erase or mutate AC-003 provenance;
- cryptographically verify external authority by itself;
- infer ownership, guilt, corruption or sanctions from identity equivalence;
- model mergers, succession or parent/subsidiary relationships as simple identity.

## Qualification gate

Before AC-006 is qualified:

1. exact-subject Civic workspace tests pass;
2. rustfmt and warnings-denied Clippy pass;
3. mutation tests prove proposals cannot self-promote and challenges revoke structural eligibility;
4. policy-bypass tests prove fabricated identifier bindings cannot skip AC-005's allowlist;
5. collision tests cover duplicate identifiers and conflicting node kinds;
6. privacy tests prove `PrivatePersonCredential` cannot enter reconciliation;
7. independent fixtures cover same-scheme registry verification and authoritative crosswalk verification;
8. reviewer-fault tests prove malformed review/verification evidence fails closed;
9. public API review confirms the low-level matcher is not externally accessible;
10. downstream deployment integration supplies a cryptographic/authority verifier before treating links as externally trusted equivalence.

## Next tranche

AC-007 builds an internal reversible equivalence-view engine over structurally eligible AC-006 links. AC-008 should then expose the deployment-authoritative public view through an injected verifier interface capable of validating signed qualification receipts or equivalent Xenia/Mycelix authority evidence.
