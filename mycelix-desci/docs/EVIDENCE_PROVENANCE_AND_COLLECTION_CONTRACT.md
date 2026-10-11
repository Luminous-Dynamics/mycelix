# Evidence Provenance and Collection Contract

**Status:** specification with a first schema-v4 declaration/projection slice drafted in [PR #4953](https://github.com/Luminous-Dynamics/mycelix/pull/4953); not merged or runtime-qualified. Independent evaluator events remain unimplemented.  
**Draft contract version:** `evidence-provenance-contract/0.1.0`  
**Scope:** evidence identity, data lineage, collection provenance, and qualification boundaries for reproduction/replication claims

## 1. Purpose

Prevent an `IndependentReplication` label, organization name, protocol label, uploader identity, artifact ID, or changed content hash from being mistaken for proof that a study collected new data. Preserve a replayable evidence lineage that an independent assessor can inspect before a replication is allowed to affect evidence maturity or resource-allocation recommendations.

This contract records provenance assertions and the evidence used to evaluate them. It does **not** make independent data collection cryptographically provable in every scientific domain, decide whether a result is true, or automatically authorize funding.

## 2. Existing implementation boundary

Static source inspection of the canonical event path found:

- `EvidenceAttached` signs and records an `EvidenceArtifact` with an ID, content hash, media type, locator, license, and availability.
- An attestation must reference evidence IDs that already exist in the stream.
- On the default branch, `ClaimProjection` retains a bare list of artifacts; attachment actor/organization and study/collection identity are not retained next to each artifact. Draft PR #4953 adds a separate `EvidenceProvenanceDeclared` schema-v4 event and stores its event/actor/organization context in a deterministic projection.
- `IndependentReplication` requires a protocol reference and evidence IDs, but those fields alone do not establish newly collected data. The new declaration makes lineage mechanically inspectable, not independently qualified.
- Raw supportive reproduction/replication counts no longer promote maturity in draft policy 1.2.0 without separately qualified counters. Those counters remain zero until an independent evaluator event/policy is implemented.
- The signed event envelope and authority receipt bind who made/accepted an assertion. Neither proves that the asserted real-world data collection occurred.

These are source observations, not runtime/test results. See [Canonical Scientific Event API](CANONICAL_EVENT_API.md), [Scientific Authority Receipts](AUTHORITY_RECEIPTS.md), and the [due-diligence RFC](SCIENTIFIC_DUE_DILIGENCE_AND_ALLOCATION_RFC.md).

## 3. Evidence model

Keep five things distinct:

1. **Artifact identity** — the specific content or resource, identified by an artifact ID and content digest. A digest detects byte changes; it does not prove how, where, or when data were collected.
2. **Provenance assertion** — a signed, versioned statement about an artifact's origin, parent artifacts, collection activity, and method.
3. **Collection activity** — the bounded real-world or computational activity that produced or transformed data; it needs a stable identifier and a typed role.
4. **Lineage relation** — a declared relation between this artifact and its inputs or the original study data.
5. **Qualification decision** — a separate, policy-versioned assessment of whether provenance is sufficiently supported for a specific claim and domain. A submitter's signed assertion is not its own independent validation.

### 3.1 Minimum provenance record (conceptual schema)

The following is a design contract, not a currently available Rust type or an existing signed-event payload:

- `provenance_schema_version`: explicit version.
- `provenance_record_id`: stable identifier for this immutable declaration.
- `artifact_id` and `content_hash`: bind the declaration to the exact attached artifact.
- `study_id` and `collection_activity_id`: identify the study and the producing activity; absence must remain explicit.
- `artifact_role`: `primary_observation`, `reused_input`, `derived_dataset`, `analysis_output`, `synthetic_data`, `protocol_or_method`, `reference_material`, or `unknown`.
- `origin_relation`: `new_collection`, `derived_from`, `copied_or_repackaged_from`, `same_data_reanalysis`, `partially_overlapping`, `external_reference`, or `unknown`.
- `parent_artifacts`: zero or more exact artifact IDs and content hashes, each with a relation/role. Do not permit an unexplained parent omission to imply independence.
- `protocol_reference` and relevant method/software/version identifiers, where applicable.
- `collection_manifest_commitment`: a canonical digest of a manifest or collection record when the domain can provide one. Keep private/regulated records out of public events; use authorized references or commitments.
- `collection_time` and other contextual metadata only where safe, necessary, and meaningful for the domain. Do not make sensitive location or participant data mandatory.
- `declared_by`: supplied by the signed event envelope/authority receipt rather than trusted as free-text identity.
- `supporting_provenance_artifacts`: evidence IDs for preregistration, data-collection logs, instrument records, custody records, repository manifests, or other supporting material.
- `disposition`: initially `asserted`, `unknown`, or `disputed`; only a separate evaluator under a declared domain policy may mark the assertion `qualified_for_profile`. No generic boolean `independent=true`.

The manifest commitment supports later integrity checks only when the manifest format, canonicalization, and covered entries are specified. A digest without a resolvable manifest or trusted production evidence is not a proof of study independence.

## 4. Qualification rules

### 4.1 Computational reproduction

A computational-reproduction candidate must bind the original input data (or its exact immutable digest set), code/software, computational steps, methods, and relevant analysis conditions. A new run ID or a new organization does not turn the same data into a replication. Any changed input or environment must be shown explicitly, not silently described as an identical replay.

### 4.2 Replication

A replication candidate must point to a distinct collection activity intended to investigate the same or a suitably similar scientific question and identify the newly collected observations or data artifacts. The active domain profile must state which differences are acceptable and how overlap, exclusions, population/context differences, and measurement changes are treated.

New collection can be performed by the same investigators or in the same laboratory; institutional novelty is not a universal requirement for replication. Conversely, a new lab or a different organization label is not sufficient.

The system can mechanically check identifiers, hashes, signed event linkage, declared lineage, manifest coverage, and policy-required fields. It generally cannot infer from those facts alone that a data-collection assertion is true. Qualification therefore needs an independent evidence check suited to the domain.

### 4.3 Partial overlap and derived datasets

If new and prior data overlap, report the overlap and its source. Do not count the full artifact as independent merely because it also contains new rows. A domain profile may qualify a bounded new-data subset when its manifest, deduplication method, and analysis population are independently inspectable; otherwise mark the independence relation unknown or disputed.

Transformed, filtered, sampled, normalized, or repackaged data inherit lineage from their inputs. Transformation is not new data collection. A copy with a new artifact ID remains a copy if its lineage shows the same source. A different digest can reflect compression, metadata, schema, or content changes and is not enough to establish independence.

### 4.4 Outcomes

Keep `supports`, `does_not_support`, and `inconclusive` as distinct outcomes. A non-supporting result is relevant evidence; it is not automatically misconduct, a proof of falsity, or a reason to discard the study. Qualification should assess study design and data lineage separately from whether the hypothesis succeeded.

### 4.5 Fail-closed disposition

For any evidence that may increase a claim's replication maturity:

- missing provenance, unresolved lineage, conflicting active declarations, inaccessible required evidence, or evaluator conflicts => **not qualified / indeterminate**;
- invalid event signature, wrong artifact digest, broken parent reference, or invalid event chain => **reject the record or fail replay**, according to the integrity rule;
- provenance supplied only by the attesting actor => **asserted, not independently qualified**;
- unsupported independence must never silently become PASS because other quality or impact scores are high.

## 5. Event and compatibility strategy

Do not add fields to existing schema-v2/v3 signed payloads in place. Their canonical bytes and historical signatures must remain verifiable.

Draft PR #4953 now contains the first step of this plan: a schema-v4 provenance declaration event using additive canonical payload tag 9, deterministic parent/hash checks, cycle rejection, and actor-attributed projection records. That draft has not been merged or runtime-qualified. The remaining implementation/review work includes:

1. Add/verify golden byte vectors for the new schema-v4 payload and prove old payload tags and schema-v2/v3 signatures remain verifiable.
2. Validate that the artifact ID and content hash exactly match the artifact in the same stream; validate every parent artifact reference and reject self/cyclic lineage.
3. Preserve event actor/organization and authority receipt when projecting the declaration. Conflicting active declarations must not use last-writer-wins semantics.
4. Add a correction/supersession mechanism that appends history and preserves prior declarations and earlier assessment decisions.
5. Introduce a new evidence-assessment policy version. Preserve raw historical attestation counts for audit, but do not treat old v2/v3 replication attestations as provenance-qualified under the new policy unless their evidence has been re-evaluated under the explicit migration rule.
6. Keep evidence attribution, study-level provenance qualification, outcome interpretation, and funding authority as separate state/decision boundaries.

Before implementation, inspect the codec implementation and golden-vector suite to assign event tags and schema-version behavior without changing any existing signed bytes. Update schema constants, validation, projections, API serialization, documentation, and migrations together. A new event variant must be understood by every supported replay and verification implementation before it can be accepted for new writes.

## 6. Adversarial fixture matrix

| Fixture | Expected disposition | What it guards against |
|---|---|---|
| Original study artifact referenced by a replication attestation | Not qualified as a new-data replication | Relabeling reproduction as replication |
| Same bytes attached under a new artifact ID | Not qualified | ID inflation |
| Same data compressed, reformatted, or given new metadata | Not qualified by digest change alone | Packaging changes masquerading as new data |
| Same input manifest, different analysis run/protocol version | Candidate for computational reproduction, not replication | Same-data reruns misclassified |
| Derived data with recorded parents tracing to original study | Not qualified as a new collection | Transformation hiding lineage |
| Distinct data collection with complete manifest and supporting records | Candidate for independent evaluation, not automatic PASS | Separating mechanical evidence from scientific judgment |
| New dataset with partial overlap | Indeterminate unless a domain profile qualifies the bounded novel subset | Partial data leakage/shared cohorts |
| Conflicting provenance declarations for same artifact/activity | Indeterminate; retain all history | Last-writer-wins manipulation |
| Parent artifact missing or hash mismatch | Reject/fail closed | Broken lineage or substitution |
| Historical v2/v3 attestation with no provenance declaration | Retain history; no provenance-qualified replication under new policy | Legacy maturity inflation |
| Provenance assertion written by the same actor who wants the replication counted | Asserted pending separate evaluation | Self-certification |
| Valid non-supporting replication result | Preserve as qualified counterevidence if provenance/method pass | Outcome-biased qualification |

The initial machine-readable synthetic cases are in [fixtures/evidence-provenance-v0.1.0.json](fixtures/evidence-provenance-v0.1.0.json), governed by the [fixture-corpus JSON Schema](fixtures/evidence-provenance-fixtures.schema.json). That JSON Schema defines only the fixture format, not the production provenance event schema. The fixtures are contract inputs with expected dispositions, not signed canonical events and not tests consumed by production code yet. For each fixture, freeze exact events, signatures, content digests, collection manifests, expected projection fields, assessment policy version, and a canonical projection commitment. Run both the authoritative replay and a separate checker that does not share the projection's qualification helper.

## 7. Qualification gates

**Gate A — contract fixtures.** Define canonical JSON/examples plus expected projection commitments for each fixture above. Keep private test records synthetic and non-sensitive.

**Gate B — codec compatibility.** Keep old schema golden vectors byte-identical; add schema-v4 golden vectors and prove old events still replay. An unknown required event type must not be silently dropped by any verifier.

**Gate C — lineage validation.** Verify exact content-hash binding, parent resolution, no self-reference/cycles, deterministic conflict handling, event ordering, correction history, and replay equivalence.

**Gate D — decision semantics.** Demonstrate that only independently qualified provenance raises replication maturity; missing/contested provenance remains indeterminate; non-supporting outcomes remain visible; and no assessment event authorizes funding or disbursement.

**Gate E — independent challenge.** Have a separate checker recompute the lineage relation from frozen inputs. Compare generated reports with adjudicated domain-specific cases before using the counts operationally.

Queued CI is not a gate result. For each gate record exact source head, inputs, checker version, outputs/commitments, and reviewer disposition. Until these artifacts exist, the implementation remains unqualified.

## 8. Interoperability and references

Prefer interoperable provenance packages over a proprietary metadata island. Research Object Crate (RO-Crate) provides a machine-readable approach to grouping research files and metadata about datasets, people, organizations, software, workflows, equipment, licensing, and provenance. Map suitable RO-Crate metadata into this contract rather than blindly copying an entire external schema into signed events:

- [RO-Crate 1.3 Introduction and Specification](https://www.researchobject.org/ro-crate/specification/1.3/introduction.html)
- [RO-Crate technical overview](https://www.researchobject.org/ro-crate/technical_overview)

The [National Academies report on reproducibility and replicability](https://www.nationalacademies.org/read/25303/chapter/3) supplies the terminology boundary: computational reproduction concerns the same data and computational process; replication concerns new data collection aimed at the same or a similar question. These definitions guide the initial profile but must be declared per scientific domain.

## 9. Claims ceiling

A schema-valid, signed provenance declaration proves only that a particular actor made an integrity-protected assertion. A valid authority receipt proves which authority context accepted the event. Neither proves that the data were independently collected or that the scientific result is true.

A passing mechanical gate means only that the specified lineage/qualification invariants held for the exact implementation, policy, and fixtures. Scientific truth, generalizability, responsible use, and improved funding decisions require separate domain evaluation and prospective evidence.
