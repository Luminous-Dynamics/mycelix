# Mobility Evidence Relationship Algebra V1

## Purpose

This contract defines the small, domain-neutral relationship vocabulary connecting engineering evidence across the mobility lifecycle.

It exists because a digital thread is not merely a collection of records. NIST describes useful digital threads as containing internal representations and associations across lifecycle stages, with standards such as STEP/AP242, QIF, and MTConnect providing interoperable representations and traceability. citeturn0search7turn0search10

The relationship algebra is deliberately narrower than an engineering ontology. It establishes provenance and semantic boundaries; it does not establish physical safety, certification, regulatory approval, or engineering correctness.

## Relationship vocabulary

| Relation | Source → target | Meaning |
|---|---|---|
| `derived_from` | DesignArtifact → Requirement; EvidenceRecord → DesignArtifact; ChangeSet → EvidenceRecord | explicit derivation/provenance |
| `manufactured_as` | ManufacturingEvent → PhysicalArtifact | manufacturing event identifies the resulting artifact |
| `inspected_as` | InspectionRecord → PhysicalArtifact | inspection record concerns an artifact |
| `tested_as` | TestRecord → PhysicalArtifact | test record concerns an artifact |
| `observed_as` | OperationalObservation → PhysicalArtifact | operational observation concerns an artifact |
| `interprets` | EvidenceRecord → InspectionRecord/TestRecord/OperationalObservation | interpretation is attached to evidence without becoming the underlying observation |
| `supersedes` | DesignArtifact → DesignArtifact; EvidenceRecord → EvidenceRecord; ChangeSet → ChangeSet | later item supersedes an earlier item while preserving history |
| `changes` | ChangeSet → DesignArtifact/PhysicalArtifact/EvidenceRecord | explicit change lineage |
| `requires_revalidation` | ImpactAssessment/ChangeSet → RevalidationObligation | change or impact creates an explicit open obligation |
| `disputes` | EvidenceRecord → EvidenceRecord | attributable disagreement between evidence records |
| `authorizes` | ExternalAuthorityReference → EvidenceRecord/PhysicalArtifact | externally attributable authority relationship |

## Non-implication rules

A valid relationship **never implies another relationship**.

In particular:

- `derived_from` does not imply `manufactured_as`.
- `manufactured_as` does not imply physical equivalence to a design.
- `inspected_as` does not imply `tested_as`.
- `tested_as` does not imply `observed_as`.
- `observed_as` does not imply a diagnosis or root cause.
- `interprets` does not imply `observed_as`.
- `supersedes` does not delete or invalidate historical evidence.
- `changes` does not imply that the change passed revalidation.
- `requires_revalidation` does not imply that revalidation is complete.
- `disputes` does not imply that either disputed record is physically false.
- `authorizes` can only originate from an explicit external-authority reference; commons consensus cannot manufacture external authority.

## Holochain boundary

Holochain validation is appropriate for deterministic protocol/data-integrity predicates and addressable dependencies. Its validation model explicitly distinguishes valid, invalid, and unresolved dependencies; an unavailable dependency is retried rather than treated as an invalid operation. citeturn0search0turn0search2turn0search3

Mobility relationship qualification remains above that protocol layer:

- a Holochain-valid edge is not an engineering truth claim;
- an unresolved dependency is not a contradiction;
- an invalid Holochain operation is not a finding that the physical artifact failed;
- a relationship edge is not a certification decision.

## Historical preservation

Relationships are append-only evidence of provenance. Supersession creates a new relationship; it does not rewrite the predecessor. This preserves the ability to reconstruct how a configuration, observation, interpretation, or obligation changed over time.

## Qualification boundary

The reference and Rust implementations qualify:

1. exact relation vocabulary;
2. exact source/target typing;
3. deterministic serialization;
4. explicit non-implication boundaries;
5. preservation of historical and disputed relationships.

They do **not** qualify:

- structural safety;
- flightworthiness;
- marine or roadworthiness;
- regulatory compliance;
- certification;
- physical equivalence;
- correctness of measurements;
- truth of an engineering interpretation.

Those claims require appropriate domain evidence and competent external processes.
