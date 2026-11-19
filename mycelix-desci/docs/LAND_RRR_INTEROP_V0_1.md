# LAND-RRR Interoperability v0.1

**Status:** draft schema, mapping specification and typed structural preflight; no runtime event adapter, authenticated authority integration or legal-effect claim  
**Owner:** Mycelix / DeSci evidence integration  
**Tracked by:** [LAND-RRR-INTEROP-001 #6315](https://github.com/Luminous-Dynamics/mycelix/issues/6315)  
**Upstream contract:** [Sol Atlas PR #936](https://github.com/Luminous-Dynamics/sol-atlas-leptos/pull/936)  
**Benchmark:** [Sol Atlas TIB-001 #937](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/937)

## Purpose

Define a narrow Mycelix payload for referencing Sol Atlas territorial assertions that describe land, customary/community, mineral, water, access, conservation and stewardship relationships.

The payload is an application reference carried through the existing Mycelix-DeSci scientific event and evidence architecture. It is not a second event store, a geometry database, a cadastral register, or a legal rules engine.

## Governing boundary

- Authenticated publisher is not the same as competent legal authority.
- A signed Mycelix event is not proof of claim truth.
- DHT consensus is not statutory registration.
- A community decision is not automatically a valid mandate or legally required consent.
- Mycelix Property transfer completion is not national deeds registration.
- An external receipt reference is not verified legal effect.

The schema sets claim_ceiling to reference-only-no-legal-effect. A trusted host must resolve and verify source evidence, signer/authority credentials, jurisdiction, scope, currentness and revocation before making any decision. This schema does not implement that verification.

## Standards and source alignment

- ISO 19152-1:2024 provides the generic LADM concepts: parties, basic administrative units, rights/responsibilities/restrictions, spatial units, sources and versioned objects.
- ISO 19152-2:2025 provides the land-registration model, including space above and below the surface, surveying and spatial representation.
- OGC Features and Geometries JSON (JSON-FG), released in May 2026, is an additional feature-payload encoding to test when extended CRS, geometry and feature-class semantics are needed; it does not establish legal boundary status or survey accuracy.
- Sol Atlas owns the spatial assertion and its geometry/feature references. This Mycelix payload does not copy coordinates or polygons.
- Mycelix EPI-008 owns typed temporal assertions and evidence-frontier semantics. Use temporal references where source precision is a calendar date, interval, open-ended period or uncertain time; do not coerce source dates into timestamps.
- Mycelix LAND-STEWARD-001 #5277 / PR #5298 owns the consent-aware landscape project, site access, measurement and publication authorization profile.
- Mycelix CIVOS-EVIDENCE-001 #5709 owns rights-aware claims, provenance, corrections, disputes, supersession, revocation and restricted knowledge semantics.

## Data mapping

| Sol Atlas source | Mycelix representation | Rule |
|---|---|---|
| TerritorialRecord schema/version, SHA-256 | source_assertion.source_schema_version / record_sha256 | Pin the exact external record, not a latest mutable URL |
| TerritorialAssertion.id | source_assertion.assertion_id | Identify the individual assertion within that exact source record |
| holder_ref | LADM party_reference / group_party_reference | Opaque ref only; resolve identity/mandate in owning system |
| interest_kind + source_interest_type_code/profile | rrr.generic_interest_kind + source code/profile | Preserve source vocabulary; mapping is not a legal interpretation |
| relationship | rrr.relationship | Right, responsibility or restriction remain distinct |
| spatial.feature_ref | ladm_references.spatial_unit_reference | Feature reference only; no copied geometry or invented survey precision |
| external BAUnit | ladm_references.basic_administrative_unit_reference | Optional reference; never overload territory_ref as BAUnit |
| claim bases | claim_basis | Preserve asserted, community-attested, administrative, court, contractual and statutory bases |
| authority_ids | authority_references | Multiple scoped authorities are permitted; do not flatten to one issuer or one trust score |
| challenge assessment / frontier | dispute_assessment | No challenge observed is bounded to a complete declared frontier; partial or unavailable coverage cannot support it |
| external register receipt | external_register_receipt | Receipt reference only; host must verify source, authenticity, scope, validity and currentness |
| Mycelix consent/access decisions | consent_authorization_references + access_policy_reference | Reference existing records; no duplicate consent ledger |
| stewardship obligations | existing landscape stewardship profile | Reuse the current commitment/observation/acceptance model rather than cloning it here |

## Schema and structural preflight

The machine-readable schema is:

- [land-rrr-reference-v0.schema.json](../schemas/land-rrr-reference-v0.schema.json)
- [synthetic example](../examples/land-rrr-reference-synthetic-v0.json)

The typed Rust payload and structural gate are in `mycelix-desci/src/core/src/land_rrr_reference.rs`, exported by the existing DeSci core library. Thirty-three focused unit tests cover the synthetic example, source-interest code/profile pairing, positive composition of distinct claim bases, wrong authority-kind rejection, duplicate claim-basis and consent-reference rejection, duplicate versus distinct scoped authority references, challenge-frontier completeness, external receipt claim-ceiling preservation, schema identity, source digest shape and the four-outcome TIB qualification policy.

The isolated Cargo harness at `mycelix-desci/land-rrr-reference-gate-harness` compiles that exact source file without building unrelated DeSci core modules. The path-scoped workflow is [land-rrr-reference.yml](../../.github/workflows/land-rrr-reference.yml). Run it locally from the repository root:

```sh
cargo fmt --locked --manifest-path mycelix-desci/land-rrr-reference-gate-harness/Cargo.toml -- --check
cargo test --locked --manifest-path mycelix-desci/land-rrr-reference-gate-harness/Cargo.toml --lib
cargo clippy --locked --manifest-path mycelix-desci/land-rrr-reference-gate-harness/Cargo.toml --all-targets -- -D warnings
```

The gate checks record structure and authority *kind* pairing. It does not authenticate the issuer, validate legal competence, resolve URI contents, check revocation/currentness, establish registration, prove consent or establish legal effect. It deliberately adds no new event store or zome; a later runtime adapter should carry this payload through the existing signed scientific-event envelope after the host resolves the references and applies the required trust checks.

All objects are closed against undeclared fields. Unknown jurisdictional source codes must be retained as source code plus profile reference. A generic mapping must never silently upgrade an assertion to an official registration or court outcome.



## FELA and implementation constraints

The UN-GGIM Sixth Expert Meeting on Land Administration and Management met in Cape Town on 29–30 May 2026 and emphasized bridging frameworks and implementation. FELA spans Governance, Technology and People. A 2026 comparative dataset of 85 jurisdictions reports that its 15-observation Eastern and Southern Sub-Saharan Africa sample (including South Africa, not a South Africa-specific estimate) had 6.7% registry API access, 13.3% cadastre API access and 46.7% registry/cadastre integration. This reinforces the design requirement to handle non-API sources, stale snapshots, offline capture, explicit source frontiers and assisted review. It is not evidence of the current condition of a particular South African register.

Reference: [UN-GGIM FELA meeting material (2026)](https://www.un.org/globalgeospatial/sites/default/files/2026-06/fela.pdf); [2026 comparison of property institutions in 85 jurisdictions](https://doi.org/10.1016/j.worlddev.2026.107354); [OGC JSON-FG announcement (21 May 2026)](https://www.ogc.org/announcement/ogc-releases-the-features-and-geometries-json-json-fg-standard/).

## Auditable qualification decision packet

The machine-readable packet contract is:
- [land-qualification-decision-packet-v0.schema.json](../schemas/land-qualification-decision-packet-v0.schema.json)
- [synthetic review-required example](../examples/land-qualification-decision-packet-synthetic-v0.json)

A `QualificationDecisionPacketV0` preserves the exact source record reference and digest, the declared evidence frontier and coverage, source-by-source qualification states, material gaps, required reviewer competencies/questions, recheck triggers, dissent references and explicit limitations. It distinguishes an unavailable source from a retrieved snapshot; an inaccessible source cannot claim a snapshot digest. A `not_evaluable` packet cannot claim complete coverage of its declared frontier.

The typed structural validator is intentionally narrow. It does not resolve source references, authenticate issuers, determine reviewer competence, establish legal effect or prove that a source search is complete. It requires assessed sources to be accounted for by the frontier's included/excluded sets, requires coverage to agree with those partitions (complete = included only, partial = both, unavailable = excluded only), prohibits an inaccessible source from claiming a retrieved snapshot digest, uses a closed gap-severity vocabulary and requires unique recheck triggers. A frontier cannot name a source without a corresponding source assessment. A `not_evaluable` packet must also describe partial or unavailable frontier coverage; the positive unavailable-frontier case is covered by a dedicated test. It records *why the host must hold* and what evidence/review would allow reconciliation.

The only mutation boundary in the packet schema is `read_only_no_authoritative_mutation`; the Rust enum cannot deserialize an affirmative mutation grant. This is a packet contract and test seam, not a live Property transaction-coordinator gate. The current code does not automatically enforce these holds at a Property write boundary.

## Four-outcome qualification policy seam

The same module now defines `TibEvaluationOutcome` and a pure `qualification_disposition` mapping for the TIB result envelope:

| TIB outcome | Host disposition | Permitted next step |
|---|---|---|
| `structural_accept` | `ContinueReadOnlyQualification` | Continue independent, read-only checks only |
| `structural_reject` | `RejectPayload` | Reject the malformed or invariant-violating payload |
| `review_required` | `HoldForReconciliation` | Hold dependent operations pending competent review |
| `not_evaluable` | `HoldForReconciliation` | Hold dependent operations until required source/permission/ground truth is available |

Every disposition explicitly returns `allows_authoritative_mutation() == false`. This adapter cannot set national registration, ownership, transfer, consent or ecological completion state. A structural accept is not an authorization; it merely allows the next read-only qualification stage.

The policy is deliberately a **pure seam, not yet wired into Mycelix Property's transaction coordinator**. The five added tests exercise the outcome mapping, fail-closed holds, no-mutation ceiling and rejection of unknown serialized outcomes. A future integration must enforce this policy at the host command boundary and reconcile unknown/ambiguous transaction outcomes before retrying mutations. This change does not claim to have implemented that runtime integration.

## External land operation lifecycle

The additional reference module `land_operation_lifecycle.rs` specifies the operational boundary between a Mycelix internal workflow and an external authority. It is deliberately separate from `LandRrrReferenceV0`: a payload can be structurally valid while a real-world operation remains blocked.

Artifacts:
- [Lifecycle JSON Schema](../schemas/land-operation-lifecycle-v0.schema.json)
- [Synthetic timeout-unknown example](../examples/land-operation-lifecycle-timeout-unknown-v0.json)
- [Executable TIB sequence vectors](../examples/land-operation-lifecycle-sequences-v0.json)
- [Sol Atlas nine-scenario transition corpus](https://github.com/Luminous-Dynamics/sol-atlas-leptos/blob/research/territorial-rights-evidence-v0-1/benchmarks/territorial-integrity/tib-property-state-transitions-v0.1.json)

The pure transition sequence is:

| Phase | Meaning | Safe next action |
|---|---|---|
| `prepared` | Operation is locally prepared, not yet externally submitted | Submit once with a stable idempotency key |
| `submitted` | An identified attempt was sent | Record a response or mark outcome unknown |
| `outcome_unknown` | Timeout/transport result did not establish commit or non-commit | Reconcile the original request; no retry |
| `retry_eligible` | Reconciliation confirms non-commit and same-key retry semantics have an evidence reference | Retry with the original idempotency key and a new attempt ID |
| `receipt_observed` | Exact receipt reference, digest and capture time are recorded | Independently qualify issuer/scope/currentness |
| `external_status_qualified` | Authority profile, qualification record, verifier and qualification digest are referenced | Project only the scoped externally reported status |
| `external_status_projected` | A limited internal read-model projection was recorded with full receipt and verifier lineage | Remain alert to later correction, challenge or revocation |
| `external_status_invalidated` | New evidence invalidates the prior projection | Hold downstream reliance and reconcile the new evidence |
| `held` / `rejected` | Operation is frozen or payload is rejected | No automatic retry or promotion |

A `ConfirmedNotCommitted` reconciliation is insufficient by itself to unlock retry: it must also carry a reference to the external endpoint's idempotency contract or equivalent evidence that reusing the same key is safe. A boolean supplied by a caller is not enough. If the endpoint cannot establish this, the operation is held.

The model preserves every submission attempt and reconciliation reference. A timed-out operation cannot be resubmitted while unknown; a later challenge, correction or revocation can invalidate a previously recorded status projection. The projected state retains the exact authority profile, qualification, verifier and digest that justified the projection.

The source contains 27 lifecycle tests; with 33 land-RRR structural/qualification tests, the focused harness now has **60 source tests**. Nine frozen TIB sequence vectors are also exercised by one table-driven Rust test. These are implementation tests, not yet a claim of passing exact-head CI.

**Hard boundary:** no phase in this module authorizes authoritative title, ownership, payment, transfer, consent or external registry mutation. `external_status_projected` means a scoped read-model projection of what a qualified external source reported; it is not a legal conclusion. The model is not yet wired into Mycelix Property's live transfer coordinator. Issue [PROP-LEGAL-EFFECT-001 #6319](https://github.com/Luminous-Dynamics/mycelix/issues/6319) owns that host-level integration.

## Evidence correction and revocation propagation

The module `evidence_invalidation.rs` adds a deterministic dependency index over canonical event/source references. It answers a critical operational question: when a source is corrected, revoked or challenged, which derived assertions, authority assessments and external-status projections must no longer be presented as current?

Artifacts:
- [Evidence invalidation graph JSON Schema](../schemas/evidence-invalidation-graph-v0.schema.json)
- [Synthetic dependency graph example](../examples/evidence-invalidation-graph-synthetic-v0.json)
- [Eight-case TIB invalidation corpus](../examples/tib-evidence-invalidation-v0.1.json)
- [Rust reference implementation](../src/core/src/evidence_invalidation.rs)

The graph is a **rebuildable index**, not a second evidence ledger. It stores references and dependency edges; canonical source events remain in the owning Mycelix or external system. Invalidation appends causal records and does not delete or rewrite source evidence or historical derived records.

The validator requires unique node identities, exact SHA-256 payload digests, existing dependency references, an acyclic dependency graph, known invalidation roots/affected nodes, and a valid causal path from the root to every affected node. For each invalidation event it also requires complete propagation to the root and *every* transitive dependent. A partial event that invalidates the source but omits a dependent projection is rejected.

The operation is deterministic and idempotent for the same event/root/reason. Multiple independent invalidation causes are retained for the same downstream node. Unrelated branches remain current. A rebuild can recompute the index from canonical evidence history without pretending that old records never existed.

**Important boundary:** this index marks records as no longer current for downstream reliance. It does not itself revoke a legal right, modify a national register, delete a community record, retract a published copy, or provide access-control enforcement. The host still needs to notify dependent systems and apply each owner's correction, disclosure and appeal process.

The focused harness now includes 33 RRR/decision-packet tests, 27 lifecycle tests and 15 invalidation-graph/corpus tests: **75 Rust tests in three production source modules**, plus the table-driven nine-scenario TIB transition vector test within the lifecycle module. This is test inventory, not a passing CI claim.

## Challenge and absence semantics

The payload permits these independent dispute assessment states:
- unassessed;
- no challenge observed within a declared frontier;
- challenged;
- under review;
- resolved.

The no-challenge state requires:
- an exact source challenge-assessment reference;
- a frontier reference and scope reference;
- complete-within-declared-frontier coverage;
- the source systems searched;
- a challenge-search report reference.

This means only that no challenge was observed in the declared search. It does not claim universal absence. The host should reuse Mycelix EPI-008 EvidenceFrontier and CurrentnessAssessment; do not create a competing global completeness mechanism.

## External registration and Mycelix Property firewall

The current Mycelix Property transfer coordinator's complete_transfer flow calls registry.transfer_ownership, updates the internal Property entry and creates an internal TitleDeed record. That is an internal Mycelix property-application transition.

It is not proof that a transfer has been accepted or registered by the competent national deeds registry or other legally relevant authority. For customary or informal interests it is not proof of transferability, consent or lawful extinguishment either.

The future adapter must therefore keep separate:
1. private contract / agreement state;
2. internal escrow and payment state;
3. Mycelix Property's internal ownership/transfer projection;
4. evidence submitted to an external registry, court or regulator;
5. receipt/decision returned by that external authority;
6. independent verification of receipt authenticity, scope, validity and currentness;
7. any downstream legal interpretation by appropriately qualified actors.

No internal event, DHT consensus, DID, escrow payment, agreement signature, community vote or AI recommendation may silently set a legal-title or registered-right status. The optional external_register_receipt field only references an external receipt; it does not say the receipt has been verified or is legally effective.

This is the principal safety requirement for integrating Mycelix Property into the land-rights programme.

## Digest and temporal rules

- Sol Atlas record_sha256 remains SHA-256 and refers to the exact source record bytes.
- Mycelix's existing ContentHash uses BLAKE3 for Mycelix-side event/artifact identity. Preserve algorithm identities; never reinterpret one digest as the other.
- Use EPI-008 temporal references for authority validity, claim validity, capture, assertion, verification, source publication and supersession. Do not convert a date-only source to an exact timestamp.
- A signed timestamp is not an independently trusted timestamp unless a specific trusted-time profile establishes that property.
- A receipt captured at time T is not automatically current at a later decision time T2.

## Privacy, consent and community control

- Do not duplicate coordinates, sacred-site locations, personal data, oral histories or restricted community knowledge into this payload.
- Measurement consent, site access, publication consent, representation mandate and legal authority are separate references and must be checked for the actual purpose.
- Use the existing landscape stewardship profile's private/controlled/embargoed/public policy and consent boundaries.
- A community decision reference does not itself prove the decision-maker's mandate; resolve that through the owning community/governance profile.
- Withdrawal, correction and revocation append new events; do not rewrite prior history or claim that external copies have disappeared.
- Never disclose restricted evidence through public indexes, logs, search, audit exports or model prompts.

### Consent and publication authorization crosswalk

The Land RRR payload must **reference** the existing LAND-STEWARD measurement-authorization record rather than creating a second consent/grant model.

| RRR purpose reference | LAND-STEWARD field/purpose | Required interpretation |
|---|---|---|
| `measurement` | `consent_status` + `consent_evidence_sha256` | Permission to participate in the named measurement activity only |
| `site_access` | `site_access_status` + `site_access_evidence_sha256` | Site-entry/access authorization; not implied by measurement consent |
| `publication` | `publication_consent_status`, `publication_policy`, `coordinate_disclosure` and publication evidence digest | A separate publication authorization; a reference does not mean it was granted |
| `participation` / `data_sharing` | Owning governance/access profile | Must be resolved for that exact audience, purpose and scope |

The current LAND-STEWARD schema is `mycelix.landscape-measurement-authorization.v0` ([schema on PR #5298](https://github.com/Luminous-Dynamics/mycelix/blob/feat/landscape-stewardship-contract-v0/mycelix-desci/schemas/landscape-measurement-authorization-v0.schema.json)). Its safety gate validates shape/status/digest consistency and has a declared validity-window helper, but it explicitly does not authenticate signatures, resolve authority, or check live revocation. The trusted host must fetch the exact record, pin its digest, validate it, check the requested purpose and time window, then resolve signer/authority and current revocation before performing the action.

The RRR synthetic example now references the same opaque authorization record under three separate purposes. It says measurement consent is granted and site access authorized, while publication consent remains `not-requested` and policy remains `private`. This is intentional: the publication reference exists to identify the relevant policy record, **not** to imply publication is allowed.

## Runtime integration stages

1. Validate the JSON Schema with an independently selected Draft 2020-12 validator.
2. Validate the Mycelix structural safety gate using exact upstream source references and existing DeSci event primitives.
3. Resolve the Sol Atlas record by exact source reference and SHA-256; verify the assertion ID belongs to that record.
4. Map LADM references losslessly; preserve unknown source codes and unmapped concepts.
5. Resolve issuer identity, authority profile, jurisdiction, scope, delegation, expiry and revocation through the appropriate Mycelix/external authorities.
6. Resolve challenge assessment and temporal frontiers at decision time; unknown or partial currentness remains unknown.
7. Resolve consent and access policy for this exact operation and data disclosure.
8. Store the payload in the existing signed immutable event envelope; do not add a parallel ledger or zome until an ownership review proves it necessary.
9. Add multi-agent tests for authority mismatch, appeal/revocation, challenge-frontier incompleteness, external receipt absence, property transfer without external receipt, selective disclosure, offline replay and supersession.
10. Publish a conformance receipt with exact source commit, schema version, fixtures, independent scorer, limitations and claim ceiling.

## Explicit non-claims

This v0.1 schema and document do not:
- authenticate source documents, signatures or authorities;
- decide whether an institution has legal competence or a community representative has a valid mandate;
- adjudicate title, customary tenure, compensation, mineral rights, consent or priority;
- establish a national registry entry or land transfer;
- verify spatial geometry, surveying or boundary correctness;
- prove environmental outcomes;
- implement encryption, access control or selective disclosure;
- provide a deployed runtime adapter or multi-agent conformance result.

No real landholder, community, right, registry receipt, pilot site or partner is represented by the synthetic example.

## Acceptance for the next tranche

- A JSON Schema and mapping document exist with exact version and explicit claim ceiling.
- All source references are content-addressed and preserve SHA-256 vs BLAKE3 algorithm identity.
- No exact geometry or private community data is duplicated in Mycelix.
- No status can be interpreted as legal registration without a separate, verified external authority receipt.
- No challenge-absence statement can be made without a complete declared frontier.
- Multi-agent tests prove that internal transfer completion alone cannot create an external registration projection.
- Consent, site access and publication remain separate and are enforced before disclosure.
- Independent CI and TIB tests are green on exact head; queued/skipped CI is not PASS.
