# AeroCommons Integration Audit V1

Status: research/design audit
Parent: AEROCOMMONS-004 (#3669)
Related: MYC-EVID-001 (#882), MYC-EVID-001A (#886), MYC-EVID-001B (#888), EVIDENCE-CI-004E (#1660), MYC-EVID-001B1..B7 (#1661-#1669), PRAX-EVIDENCE-001/002 (#2467/#2471), ROS-002 (#3654)

## 1. Executive finding

AeroCommons should not create a second general-purpose evidence/provenance authority.

The repository already contains an evidence architecture with a canonical evidence-manifest waist and source-specific evidence adapters. The strongest reusable pattern is:

```
source acquisition
-> exact source-evidence identity
-> source-specific normalization
-> execution/observation identity
-> typed composition
-> bounded claim projection
```

AeroCommons should add the engineering-domain layer around configuration, artifacts, requirements, physical tests, manufacturing/inspection records, and engineering relationships.

## 2. Existing substrate discovered

### MYC-EVID-001 / #882

The evidence program establishes a versioned canonical evidence-manifest architecture rather than allowing each consumer to invent its own authority model.

Important properties:
- evidence/status semantics are versioned;
- current and historical evidence are distinct;
- claims have explicit authority classes;
- evidence references are not themselves truth claims;
- source adapters feed the canonical waist;
- claim projections must remain within their evidence ceiling.

### MYC-EVID-001A / #886

The canonical manifest validator is intentionally a small, deterministic waist.

Its documented contract includes:
- separate evidence domains;
- exact subject identity;
- receipt/evidence digest requirements for strongest technical PASS;
- stale/historical evidence semantics;
- fail-closed handling of unknown authority-bearing keys;
- deterministic projection;
- no scalar confidence replacement for typed evidence.

This is directly reusable as an architectural pattern for AeroCommons.

### MYC-EVID-001B / #888

The GitHub source adapter demonstrates the correct source-adapter boundary:

```
provider/discovery data
-> exact source identity
-> canonical evidence projection
```

It explicitly prevents branch names, PR numbers, mutable navigation metadata, or unrelated successful runs from becoming authority.

### EVIDENCE-CI-004E / #1660

This work provides an especially important reusable identity pattern:

```
provider response acquisition
-> exact source-evidence identity
-> source normalization
-> downstream composition
```

The source-evidence identity is opaque to downstream normalizers. Later observations create new evidence identities instead of rewriting historical evidence.

### MYC-EVID-001B1..B7 / #1661-#1669

These follow-on contracts reinforce several AeroCommons requirements:
- exact tested/designated subject identity must be separated from workflow/run identity;
- source identity must not collapse merely because projected technical status is equal;
- receipts cannot retroactively prove missing subject provenance;
- run-level terminal outcomes must not automatically become semantic failure;
- historical evidence remains immutable;
- provider source identity does not grant truth, runner trust, publication authority, or deployment authority.

These are directly analogous to aircraft configuration and engineering-evidence lineage.

### PRAX-EVIDENCE-001/002 / #2467/#2471

The Praxis evidence work adds a useful storage boundary:
- an observed event is distinct from admission;
- persistence is distinct from capability/credential decisions;
- provenance completeness must not be fabricated for legacy records;
- append-only observations should not be silently rewritten;
- arbitrary links must not become evidence authority.

This supports the AeroCommons distinction between physical observations, engineering interpretations, and governance/authorization.

### ROS-002 / #3654

ROS-002 independently reaches the same architecture:
- source != assertion != evidence reference != inference != authorization;
- exact source/currentness/revision context must be retained;
- corrections and supersession create lineage rather than rewriting history;
- protected evidence can be referenced without publishing its payload.

AeroCommons should align with this vocabulary rather than inventing competing semantics.

## 3. Consequence for AeroEvidenceV1

The current `aero-evidence-types` crate remains useful, but its role should be narrowed:

```
AeroEvidenceV1
= engineering-domain representation / adapter contract

not

AeroEvidenceV1
= second universal Mycelix evidence authority
```

Where an existing Mycelix evidence primitive already carries:
- source identity;
- provenance;
- currentness;
- correction/supersession;
- epistemic status;
- controlled disclosure;
- replication lineage;

AeroCommons should reference or adapt that primitive.

Engineering-specific fields belong in AeroCommons only where the existing substrate has a genuine semantic gap.

## 4. New graph model

The Holochain engineering graph should have two classes of relationships.

### Structural relationships

Ordinary typed links are appropriate for navigation/indexing:

- Configuration -> contains -> Artifact
- Configuration -> parent -> Configuration
- ChangeSet -> proposes -> Configuration
- Evidence -> references -> Artifact

These links do not independently establish engineering truth.

### Epistemic relationships

A relationship that asserts something consequential about reality should be a provenance-bearing record:

- Inspection demonstrates Requirement
- Test failed for Artifact under Procedure
- Analysis predicts Quantity
- Observation contradicts Prediction
- Evidence reproduces Evidence
- Evidence invalidates Evidence
- ChangeSet requires Revalidation

Conceptually:

```
EpistemicRelationV1 {
    relation_id,
    subject_ref,
    predicate,
    object_ref,
    assertion_kind,
    basis_evidence_refs,
    source/provenance,
    observation/effective time,
    epistemic status,
    validity scope,
    lifecycle,
    correction/supersession,
    visibility
}
```

The exact type should be reconciled with the existing evidence substrate before implementation.

## 5. Holochain mapping

Current Holochain guidance makes this boundary especially clear.

Entries are typed content-addressed records paired with authoring actions. Action identity already provides authorship/timestamp context, while entry hashes identify content. Links are graph metadata connecting addresses. citeturn0search0turn0search7

Therefore:

### Entries

Use entries for substantive authored engineering objects and assertions:
- Artifact
- Configuration
- ChangeSet
- engineering evidence adapter/reference
- EpistemicRelation
- Dispute
- Obligation

### Links

Use links for traversal/indexing:
- configuration membership
- parent/child navigation
- evidence reference indices
- relation discovery indices

Do not put a consequential engineering assertion solely into an anonymous link tag.

### Validation

Integrity validation should enforce structural and protocol invariants that peers can deterministically reproduce.

Holochain explicitly requires validation to be deterministic; dependencies must be addressable/retrievable, and unresolved dependencies remain unresolved rather than being silently treated as valid. citeturn0search2turn0search9

That maps directly to AeroCommons:

```
missing requirement/evidence/configuration dependency
!= valid by default

unresolved engineering dependency
-> Unknown / unresolved obligation
```

But Holochain validation must not attempt to decide physical truth, engineering adequacy, or regulatory airworthiness.

## 6. Identity stack

AeroCommons should preserve distinct identities:

1. Artifact identity — what engineering object is referenced.
2. Content identity — exact bytes/digest of an external artifact.
3. Configuration identity — exact configuration membership/lineage.
4. Source-evidence identity — exact observation/analysis/test source.
5. Execution/observation identity — exact run, inspection, build, or test instance.
6. Claim/relation identity — exact assertion made from those sources.

Do not collapse these into one SHA or one generic UUID.

The existing Mycelix evidence work strongly supports this separation.

## 7. External digital-thread boundary

AeroCommons should continue to sit above engineering standards rather than replace them.

NIST identifies STEP/AP242 as a foundational engineering/manufacturing interchange standard and QIF as part of standards-based manufacturing/quality integration. NIST's digital-thread work also explicitly addresses design, manufacturing, inspection, and traceability. citeturn0search8turn0search10

Therefore:

```
STEP/AP242 / CAD / CAE / QIF / manufacturing / inspection
        |
        v
exact artifact/source identity
        |
        v
AeroCommons engineering evidence envelope
        |
        v
Mycelix provenance / relationship substrate
```

AeroCommons describes the lineage and assertions surrounding engineering artifacts; it should not attempt to replace STEP, QIF, CAD, CAE, MES, metrology, or certification standards.

## 8. Proposed implementation sequence

### Phase 1 — substrate adapter

Do not add a new broad-purpose evidence zome.

Instead:
1. define an adapter from AeroEvidenceV1 to the existing Mycelix evidence waist;
2. preserve exact source-evidence identity;
3. preserve currentness/correction/supersession semantics;
4. preserve controlled disclosure;
5. add engineering-specific configuration/artifact references.

### Phase 2 — engineering objects

Add only domain-specific records:
- AeroArtifactRef
- AeroConfiguration
- AeroChangeSet
- AeroEngineeringRelation
- AeroDispute
- AeroObligation

### Phase 3 — graph

Add structural links for navigation.

Add provenance-bearing relation entries for epistemic assertions.

### Phase 4 — deterministic impact

Feed the existing `ChangeSetV1 -> EvidenceImpact` classifier with explicit dependency references.

Unknown dependencies remain Unknown.

### Phase 5 — physical reproducibility challenge

Only after the above is coherent, execute AEROCOMMONS-002 using a non-flight-critical physical artifact.

## 9. Critical invariants

```
structural link != engineering proof

valid Holochain record != physical truth

attestation != measurement

consensus != physical evidence

prediction != observation

evidence reference != evidence truth

Symthaea inference != observed fact

provenance != certification

Holochain validation != airworthiness

open artifact != regulatory approval
```

## 10. Audit conclusion

The strongest path is now clear:

```
Existing Mycelix evidence/provenance substrate
                    |
                    v
          AeroCommons adapter layer
                    |
          +---------+---------+
          |                   |
          v                   v
   Configuration        Physical evidence
      lineage               lineage
          |                   |
          +---------+---------+
                    v
          Epistemic engineering graph
                    |
                    v
               Symthaea
                    |
                    v
        human engineering decision
                    |
                    v
             physical result
                    |
                    v
              new evidence
```

The central architectural achievement is therefore not another evidence ledger.

It is the ability to make **engineering relationships themselves provenance-bearing, contestable, configuration-bound, and reproducible while reusing Mycelix's existing epistemic substrate**.

That gives AeroCommons a much smaller implementation surface and a much stronger claim boundary.
