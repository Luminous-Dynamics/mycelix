# AC-006 — Reversible Public-Entity Identity Reconciliation

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-006 solves a narrow but important problem created deliberately by AC-005: records from different standards and publishers remain separate graph nodes even when they may refer to the same real-world organization.

The solution is **not** destructive entity merging.

AC-006 adds provenance-bearing, reversible identity-link claims. Source nodes remain intact forever. Consumers may aggregate across nodes only when a link has reached AC-006's qualified `Corroborated` state.

## Why a separate layer

AC-005 correctly refuses to infer identity merely because two imported records look similar. Its allowlisted `EntityIdentifierBinding` values are evidence that can support reconciliation, but they are not permission to overwrite source provenance.

AC-006 therefore separates:

source identity -> identifier observation -> link proposal -> independent verification -> review -> aggregation eligibility

from the unsafe shortcut:

similar records -> destructive merge

## Eligible subjects

The initial AC-006 path accepts only AC-003 `Organization` and `LegalEntity` nodes.

`PrivatePersonCredential` is constitutionally outside this reconciliation path. AC-006 has no person-matching or person-profile mechanism.

## Exact identifier proposals

`propose_exact_identifier_links` consumes AC-005 ingestion results and groups only exact `(scheme, identifier)` pairs that survived AC-005's deny-by-default public-entity identifier policy.

Automatic matching can create only `IdentityLinkStatus::Proposed`.

There is deliberately no authoritative fuzzy-name, address, embedding, reputation, or probabilistic matching API in AC-006.

A repeated exact identifier assigned to multiple different nodes inside the same source result is treated as ambiguity and fails closed. It is not used as evidence that those nodes are identical.

At least two distinct source references are required before AC-006 creates an exact-identifier proposal between different graph nodes.

## Link states

- `Proposed` — machine-generated or manually submitted evidence exists, but the link is not aggregation-authoritative.
- `Corroborated` — the link has distinct source evidence, authoritative verification, and an explicit review reference/rationale. Only this state can become aggregation-eligible.
- `Challenged` — a challenge is preserved and the link is ineligible for aggregation.
- `Rejected` — reviewed decision that the link must not be used.
- `Superseded` — reviewed replacement by another link record; the original evidence remains intact.

There is no operation that deletes or rewrites either source node.

## Corroboration gate

A `Corroborated` link must contain:

1. evidence for both endpoints;
2. bindings observed in at least two distinct source references;
3. at least one structurally valid authoritative verification;
4. a verification that actually supports the endpoint identifiers;
5. a non-empty review reference and rationale;
6. no unresolved challenge references;
7. `reversible = true`.

Authoritative verification can currently be:

- a lookup against the issuing registry for one exact scheme + identifier shared by both endpoints;
- an authoritative crosswalk between the public identifier carried by one endpoint and the public identifier carried by the other.

A verification record itself carries verifier identity, provenance, digest-bearing evidence and verification time.

## Challenge and reversal

An unresolved challenge immediately makes a link ineligible for aggregation. A link cannot remain validly `Corroborated` while challenge references are present.

Rejected and superseded states require review provenance and rationale. A superseded link must point to a different replacement link; self-supersession fails closed.

Because links are separate records rather than merged nodes, reversal changes only the reconciliation view. Original OCDS/BODS/Mycelix provenance remains available for audit.

## Aggregation semantics

`aggregation_eligible(link)` returns true only when:

- status is `Corroborated`; and
- the entire AC-006 structural validator passes.

AC-006 does not itself rewrite AC-004 metrics to collapse equivalent nodes. A later tranche should consume only aggregation-eligible links and preserve the exact identity-link set used by each derived metric.

## External alignment

The architecture follows the principle used by organization-identifier infrastructure such as org-id.guide: an identifier is meaningful only together with its scheme/list, and high-quality identifiers can support joining information across datasets.

BODS similarly recommends real-world entity identifiers, including company registration numbers and LEIs, with identifier schemes drawn from org-id.guide. AC-006 uses those public entity identifiers as evidence while retaining the source records that supplied them.

## Failure semantics

AC-006 fails closed for, among other cases:

- private-person identifier bindings;
- bindings whose node is absent from the AC-005 result;
- the same graph node ID appearing with incompatible node kinds;
- one source assigning the same exact identifier to multiple different nodes;
- links between the same endpoint;
- non-reversible links;
- evidence referring to nodes outside the link;
- corroboration without distinct source observations;
- corroboration without supporting authoritative verification;
- corroboration without review provenance;
- corroboration with an unresolved challenge;
- invalid verification provenance;
- challenged state without a challenge reference;
- rejected/superseded terminal state without review;
- supersession without a valid replacement reference.

## Non-goals

AC-006 does not:

- match natural persons;
- use names, addresses, embeddings, social graphs or reputation to establish authoritative identity;
- automatically promote proposals to corroborated links;
- choose a canonical node and erase the others;
- infer ownership, guilt, corruption, sanctions, or civic standing from identity equivalence;
- establish legal succession after mergers, dissolutions or reorganizations.

Those require distinct typed relationships rather than pretending organizational succession is the same thing as identity.

## Qualification gate

Before AC-006 is qualified:

1. exact-subject Civic workspace tests pass;
2. rustfmt and warnings-denied Clippy pass;
3. mutation tests prove proposals cannot self-promote and challenges revoke aggregation eligibility;
4. collision tests cover duplicated identifiers within one source and conflicting node kinds;
5. privacy tests prove `PrivatePersonCredential` cannot enter the reconciliation path;
6. independent fixtures cover same-scheme registry verification and authoritative crosswalk verification;
7. reviewer-fault tests prove empty or malformed review/verification provenance fails closed;
8. downstream metric tests prove any entity aggregation records the exact AC-006 links used and can be recomputed without destructive node rewriting.

## Next tranche

AC-007 should add a **qualified equivalence-view / metric projection** over AC-006 links. It should compute reversible entity groups for analysis without mutating AC-003, detect contradictory link components before transitive closure, and make every AC-004 concentration result retain the exact identity links that changed its denominator or grouping.
