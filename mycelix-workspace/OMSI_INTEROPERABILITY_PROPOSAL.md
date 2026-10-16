# OMSI Manufacturing Evidence Interoperability Proposal

**Status:** design proposal; no runtime bridge is implemented here  
**Scope:** read-only, versioned interoperability between OMSI semantic manufacturing validation and Mycelix identity, knowledge, fabrication, supply-chain and governance capabilities  
**Counterpart:** [OMSI integration design in luminous-platform](https://github.com/Luminous-Dynamics/luminous-platform/blob/feat/omsi-manufacturing-rust-validator/docs/standards/omsi/manufacturing-mycelix-interoperability.md)  
**OMSI pilot PR:** [luminous-platform #53](https://github.com/Luminous-Dynamics/luminous-platform/pull/53)

## Decision

Integrate by contract, not by merging responsibilities.

OMSI should remain an offline-capable, vendor-neutral profile, validator and replay contract. Mycelix should add optional decentralized identity, issuer credentials, revocation evidence, epistemic claims, review/endorsement workflows, and evidence-receipt references. Mycelix must not re-implement manufacturing numeric semantics ad hoc, and OMSI must not require a Holochain conductor or a MATL score to perform its deterministic evaluation.

No Mycelix runtime bridge, receipt entry type, or production integration is asserted to exist by this proposal. The counterpart OMSI draft PR #53 now contains local `audit-adapter-map`, `apply-adapter-map`, `verify-transform-vectors`, `audit-adapter-schema-coverage`, `create-provenance-manifest`, and `verify-provenance-manifest` CLI commands. Its 10-case decimal/unit transform corpus remains narrow. Schema-coverage examples now include a four-leaf MES-like fixture and an eight-leaf QMS-like fixture; both are synthetic, not vendor schemas. The schema-coverage command catches absent optional mappings and presence mismatches within a restricted JSON Schema subset; it does not establish vendor conformance or validate XML/XSD/QIF. Provenance manifest contract v0.2 requires a fresh schema-coverage report and links eight artifacts through five typed activities; its downstream verifier reconstructs the expected manifest from all supplied artifact bytes. These are same-implementation local consistency checks—not source authentication, an independent verifier, a Mycelix adapter, vendor certification, or production integration.

## Why these projects complement rather than duplicate one another

Current Mycelix source provides several relevant building blocks:

- Identity has DID/credential structures, including a [W3C VC 2.0-shaped integrity entry](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-identity/zomes/verifiable_credential/integrity/src/lib.rs) with issuer, valid-from/until, credential-status and proof fields.
- The [epistemic claim SDK](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/sdk/src/epistemic/claims.rs) represents a claim plus evidence references, issuer, timestamp and metadata. Its E/N/M axes classify empirical verifiability, normative agreement and materiality/retention.
- The [Fabrication verification integrity zome](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/happs/fabrication/zomes/verification/integrity/src/lib.rs) defines public design-verification, safety-claim and verification-request records.
- The [Fabrication print integrity zome](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/happs/fabrication/zomes/prints/integrity/src/lib.rs) defines public print-job, print-record, monitoring-session and anomaly records. The [integration guide](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/happs/fabrication/docs/INTEGRATION_GUIDE.md) describes links with Knowledge, Supply Chain, Marketplace, Property and other Mycelix services.
- The [shared bridge routing registry](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/crates/mycelix-bridge-common/src/routing_registry.rs) has manufacturing and supply-chain allowlist constants, but the [architecture documentation](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/ARCHITECTURE.md) notes that not all bridge zomes have migrated to the central registry.

These components are *potential integration surfaces*, not proof of a completed OMSI workflow. Confirm the exact runtime APIs, route wiring, credential verification behavior and multi-agent test coverage before code depends on them.

## Packaging blocker to fix first

GitHub's Contents API returns the content of `mycelix-workspace/happs/manufacturing` as:

```text
/srv/luminous-dynamics/mycelix-workspace/mycelix-manufacturing
```

The original target is an absolute machine-local path, not a portable repository-relative path. The canonical Manufacturing source tree is tracked at `mycelix-workspace/mycelix-manufacturing/`; the wider alias audit now shows that most source-backed aliases can be repaired without moving or duplicating sources.

[Mycelix PR #5045](https://github.com/Luminous-Dynamics/mycelix/pull/5045) now changes 18 source-backed aliases from machine-local absolute targets to repository-relative links and adds a dedicated clean-checkout guard. Two paths remain deliberately unresolved: `happs/health` lacks the expected tracked `mycelix-health/` source, and `happs/supplychain` lacks the expected tracked `mycelix-supplychain/holochain/` source. The exact-head portability and D6S jobs are queued; clean-checkout execution, unified hApp packaging, WASM build and multi-agent qualification remain unverified.

The current Fabrication hApp has committed source files and an explicit verification zome, so it is a sensible **first design target** for a synthetic print-quality workflow while the Manufacturing path is resolved. That does not mean Fabrication is production-ready; check its current maturity and multi-agent test status separately.

## Architecture boundaries

### OMSI owns

- Manufacturing profile and schema versions.
- Decimal-safe deterministic interpretation and named comparison rules.
- Semantic report and rule-level decision trace.
- Conformance vectors and cross-implementation expectations.
- Replay envelope contract, exact input-artifact digest semantics and verifier behavior.
- Disclosed limitations and source-vs-derived claim distinctions.

### Mycelix owns

- DID and credential presentation/verification, including issuer authorization and revocation/status checks under a named policy.
- The identity of agents or organizations making claims; credentials for inspectors, labs or operators where the policy requires them.
- Epistemic classification and links to knowledge claims, kept separate from OMSI result states.
- Minimal evidence-receipt records and cross-cluster links between Fabrication, Supply Chain, Knowledge, Identity and any future Manufacturing cluster.
- Collaborative review, dispute and governance policy workflows.
- Access, publication, retention and selective-disclosure policy for shared evidence references.

### Explicitly not delegated

- MATL or reputation does not determine calibration validity or alter numeric comparisons.
- DKG/quorum agreement does not prove a physical measurement is true.
- A public Holochain record does not establish the truth of every referenced external fact.
- A well-formed or signed claim does not, by itself, authorize product release or machine control.
- Holochain is not the high-frequency telemetry bus and must not receive every sensor sample.

## Proposed record lifecycle

1. **Collect locally.** A MES/QMS/metrology or Fabrication export supplies a bounded observation with explicit source revision, observed time, snapshot semantics, unit and specification revision. Keep private and high-rate process data in its authorized source or private store.
2. **Evaluate with OMSI.** Run the pinned OMSI profile/rule and emit a semantic report plus raw-byte replay envelope. Preserve reject/conflict/loss states.
3. **Replay locally.** Verify the envelope and re-run the evaluator over the exact input bytes before publication. Replay consistency is not source authentication.
4. **Verify attribution.** Through a supported Mycelix Identity verification path, resolve the DID and relevant issuer credentials, proof type, intended use, expiry and revocation status required by policy. Record when and how status was checked.
5. **Publish a minimal receipt.** Create or link a Mycelix evidence-receipt object with the envelope digest, pinned profile/rule IDs, outcome, agent/issuer references, credential status evidence, opaque subject reference and visibility/retention classes. Avoid the full report and raw process data in a public DHT entry unless an explicit disclosure review approves those fields.
6. **Review independently.** A second agent verifies the receipt and envelope. Reviewers may add independent attestations or conflicts, not overwrite the original source claim or OMSI's deterministic result.
7. **Keep release separate.** Any quality-release or control action requires a separate domain authority, approver policy, hazard analysis, audit trail and safety case.

## Source-schema coverage: mappings must outlive the sample

The OMSI draft also adds `audit-adapter-schema-coverage`. A record-only field audit can miss a schema-defined optional field when that field simply does not appear in the current sample. The schema-coverage report inventories scalar/array leaf paths from a supported, closed-object JSON Schema subset and requires each to have exactly one adapter mapping with matching required/optional presence. It reports missing schema mappings, adapter paths outside the schema, duplicate paths, presence mismatches and structural errors such as a missing child required when an optional parent object is present.

This is a **bounded field-path and structural coverage profile**, not a full JSON Schema runtime, a vendor-conformance certification, or proof that a transformation is semantically correct. It deliberately rejects open objects, `$ref`, composition/conditional and pattern-based constructs it cannot inventory safely, and it treats arrays as atomic paths. It does not yet handle XML/XSD or QIF's complete model. The MES-like fixture has four leaf paths, with only the required temperature present; the QMS-like fixture has eight required leaf paths and is also used in the schema-bound provenance integration. CI is authored to reject absent optional mappings, mismatched presence, unsupported open schemas, and a stale coverage report. The latest OMSI workflow result observed for its implementation head is queued with no conclusion; the subsequent docs/scorecard head has no reported exact-head run, so no pass is claimed.

**Mycelix rule:** require schema-coverage evidence before qualifying any future source adapter; bind the schema digest and coverage-report digest into the eventual evidence receipt/attestation. Do not mark an adapter complete based solely on observed fields from one sample record. Keep the schema and any proprietary source models under appropriate data-owner licensing and disclosure controls.

## Provenance manifest: useful chain evidence, not an attestation

The OMSI draft now models manufacturing evidence as a derivation graph rather than a stand-alone JSON verdict. Provenance manifest contract `omsi-manufacturing-provenance-manifest/0.2` names eight artifacts—source record, source schema, adapter map, schema-coverage report, projection report, semantic input, semantic report and replay envelope—and five activities: schema-coverage audit, adapter projection, semantic-input extraction, semantic evaluation and replay-envelope creation. Fourteen typed edges express which artifacts each activity used and generated. The graph declares a constrained `w3c-prov-dm-aligned-subset/0.2` vocabulary; it does not claim complete PROV-DM serialization conformance.

The `create-provenance-manifest` command re-runs `audit-adapter-schema-coverage` from the exact source record/schema/map bytes and refuses to construct a bundle unless the coverage report is complete and equals the freshly generated report. It then re-runs adapter projection, compares the projection report, requires semantic input to equal the projection payload, recomputes the semantic report, and verifies the replay envelope against exact semantic-input bytes. The manifest includes schema-coverage contract/status/counts plus the source-schema and coverage-report digests; both artifacts are separately included in its eight-artifact digest list. `verify-provenance-manifest` reconstructs the manifest from all eight supplied artifact bytes and rejects field/digest/edge drift. Both commands are local, same-implementation checks. The manifest is unsigned and cannot establish who originated the source, that a calibration or physical reading is valid, or that a release is authorized.

**Mycelix integration rule:** do not treat `local_chain_consistent` or `manifest_verified` as an identity credential, truth claim, consensus result, or release permission. The coverage auditor deliberately supports only a restricted closed-object JSON Schema field-path/structural profile. It fails closed on open objects, references, composition/conditional constructs and unsupported field semantics; it is not full JSON Schema validation, vendor conformance or XML/XSD/QIF support. Require this coverage artifact before qualifying an adapter, but do not mistake it for proof of semantic equivalence. If policy needs authenticated production history, the next layer should use per-step attestations with signer identity, key lifecycle and policy verification. The in-toto Link predicate is a candidate envelope for a later integration, not yet implemented. <https://github.com/in-toto/attestation/blob/main/spec/predicates/link.md>

The public receipt should store only the needed digest references, version identifiers, issuer/policy/status evidence and disclosure metadata. Keep raw/high-rate telemetry and confidential manufacturing parameters in authorized storage; apply a separate privacy review before exposing artifact/schema digests or stable tenant identifiers.

**Proposed hard gate:** every valid schema/record/map/coverage/projection/input/report/envelope chain fixture must build and verify; mutations across each of the eight artifacts, an altered manifest digest, a stale/incomplete coverage report, or a broken derivation edge must reject. Platform PR #53's code/tests/workflow are authored, but its exact-head CI is queued and no pass is claimed. Mycelix receipt/bridge code remains unimplemented.

## Proposed receipt contract (not yet implemented)

The eventual entry type should contain enough information to locate and replay an artifact, not enough to centralize a factory's telemetry. Use a stable contract ID, and review all field names and hashes before turning this table into a Rust/HDI struct.

| Field | Required meaning |
|---|---|
| `receipt_contract` | Explicit versioned schema identifier |
| `subject_ref` | Opaque or audience-scoped part/lot/print/observation reference |
| `omsi_profile` | Profile ID and version |
| `rule_ref` | Exact rule ID/version; never current-default inference |
| `input_artifact_digest` | Algorithm, digest, byte length, digest scope and canonicalization declaration |
| `envelope_artifact_digest` | Algorithm and exact digest semantics for the preserved envelope artifact |
| `report_outcome` | OMSI outcome copied as-is |
| `report_ref` | Authorized pointer to the report/evidence bundle if safe to share |
| `validator_ref` | Validator version/identity and relevant build or release reference |
| `issuer_ref` | DID and credential ID/reference for each attestation role that policy requires |
| `credential_status_receipt` | Status outcome plus check time, method and policy version—not a timeless “valid” boolean |
| `evidence_dimensions` | Typed independent dimensions and blockers; no single trust number |
| `epistemic_ref` | Optional E/N/M classification and Knowledge claim link, explicitly marked as a separate layer |
| `visibility_retention` | Publication scope, access category and retention policy |
| `supersedes_or_disputes` | Links to newer or conflicting attestations without destructive replacement |

Do not assume SHA-256 used by an OMSI envelope is equivalent to Mycelix's other hash/commitment functions. Every digest field must name its algorithm and scope.

## Preserve epistemic meaning: E/N/M is not OMSI assurance

The Mycelix epistemic cube defines three different axes:

- **E (empirical):** the mode/strength of verifying a claim, from unverifiable belief/testimony through private verification, cryptographic proof and public reproducibility.
- **N (normative):** who agrees with or adopts the claim, from personal to community/network/axiomatic agreement.
- **M (materiality):** how long the claim matters, from ephemeral to foundational.

OMSI needs different fields for tenant scope, source identity, revision history, snapshot consistency, calibration coverage, measurement uncertainty and decision-rule applicability. Those dimensions do not map one-to-one to E/N/M. For example, a reproducible calculation could have weak source provenance, and a widely agreed policy could be normatively strong without proving that a particular machine reading is true.

A future adapter may attach both models to one evidence receipt, but it must preserve each model's own semantics. Unknown or incomplete evidence remains explicit; no conversion from “E4” or “high MATL” to “measurement proven” is permitted.

## MATL, trust and governance: advisory only for this use

MATL and reputation can help prioritize review, identify suspicious contributor behavior under a documented model, or inform a governance decision about who is eligible to attest. They cannot replace a manufacturing measurement rule. Keep separate outcomes for:

1. semantic validity of the input;
2. source/issuer authentication and credential status;
3. traceability and metrology sufficiency;
4. peer/reputation indicators;
5. human or organizational release authority.

Do not aggregate these into a scalar that lets a high reputation score hide an expired calibration, mismatched tenant, missing unit or rejected input. DKG can be used for governance or multi-party approval of the policy/role registry when appropriate, not as an oracle for measurement correctness.

## Critical Holochain validation design rule

Holochain integrity validation must be deterministic and must not depend on mutable outside services or current wall-clock time. Keep credential-status lookups and online policy resolution in a coordinator/preflight workflow that creates an explicit verification receipt. Integrity validation should validate the self-contained receipt and deterministic DHT dependencies available to all validators. A later verifier can re-check whether a credential was revoked since the historical check, but it must not silently rewrite the historical decision.

See the [Holochain validation guidance](https://developer.holochain.org/build/validation/) for the determinism and dependency requirements.

## Privacy, scale and industrial safety

- Current Fabrication verification/print entry structs label multiple entries public. Treat this as a real privacy threat model, not as a harmless default.
- Never publish raw customer designs, confidential recipe parameters, raw high-rate telemetry, operator personal data, tenant IDs or stable IDs that permit unwanted cross-site correlation without explicit authorization and review.
- Store high-rate streams locally; publish selected evidence windows, summaries and artifact hashes/controlled pointers only where the data owner approves.
- A digest is not authentication. The current OMSI envelope is unsigned; if authenticated exchange is necessary, evaluate W3C VC and a standard subject/predicate/envelope system such as in-toto/DSSE, including trust roots, key rotation and revocation semantics.
- Holochain integrity constraints show that a record is structurally admissible under that DNA's rules, not that the upstream fact is physically true.
- No OMSI/Mycelix evidence receipt may automatically trigger PLC/SCADA writes, process parameter changes, machine shutdown or product release.

Research anchors: [W3C Verifiable Credentials 2.0 Recommendation](https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/), [in-toto Attestation Framework](https://github.com/in-toto/attestation/blob/main/spec/README.md), and [NIST Digital Thread for Manufacturing](https://www.nist.gov/programs-projects/digital-thread-manufacturing).

## Acceptance gates

### Gate 0 — Make the source portable
- Complete the relative-link repair for the 18 source-backed aliases.
- Resolve the health and supply-chain canonical source mappings rather than pointing them at a similar-but-not-equivalent cluster.
- Pass clean-checkout path validation, then test hApp manifest/DNA packaging and WASM builds at the exact commit.
- Verify correct Holochain version and actual multi-agent test status rather than relying on table claims.

### Gate 1 — Freeze a small interop contract
- Define receipt and digest schema versions.
- Specify input vs report vs envelope digest scopes.
- Specify policy/credential status evidence and privacy/retention categories.
- Add fixtures with conflicts, loss, missing revision and denied scopes.

### Gate 2 — Qualify the local, read-only adapter
- The OMSI-side draft now supplies `audit-adapter-map`, `apply-adapter-map`, and `verify-transform-vectors`, including field-loss accounting, fail-closed projection, and exact vectors for two narrowly scoped transforms. Adapter mappings can distinguish `required` and `optional` paths: missing required fields block projection, while absent optional fields remain listed and counted as skipped rather than being mislabeled as drops.
- Add source-schema coverage for optional fields, representative vendor fixtures, and approved transformation/crosswalk vectors before qualifying any real source system.
- Verify the output schema and OMSI replay before creating a receipt; no conductor/network dependency may be required for local deterministic evaluation.
- Independent implementation tests must reproduce semantic output and digest scope; queued CI is not qualification.

### Gate 3 — Add Mycelix identity and receipt integration
- One allowlisted coordinator path with issuer-purpose/scope checks.
- Credentials include proof, expiration and status/revocation handling.
- Integrity validation stays deterministic with addressable dependencies.
- Preserve conflicts and supersession rather than overwrite.

### Gate 4 — Multi-agent qualification
- Tryorama/sweettest with at least two agents.
- Negative tests for modified input/report/envelope, expired/revoked credential, unauthorized issuer, replay, duplicate receipt, cross-tenant ID, missing DHT dependency, route bypass, disclosure leakage and conflicting reviewers.
- Recheck exact-head CI; queued jobs are not passes.

### Gate 5 — Domain review
- Manufacturing-quality/metrology reviewer approves the selected rule and its evidence preconditions.
- Security/privacy reviewer approves the publication model and authority separation.
- No product release or equipment-control claim until its separate safety and authorization program exists.

## Research references

- [Mycelix architecture](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/ARCHITECTURE.md)
- [Mycelix Fabrication verification](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/happs/fabrication/zomes/verification/integrity/src/lib.rs)
- [Mycelix Fabrication prints](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/happs/fabrication/zomes/prints/integrity/src/lib.rs)
- [Mycelix Identity VC zome](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-identity/zomes/verifiable_credential/integrity/src/lib.rs)
- [Mycelix epistemic claims](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/sdk/src/epistemic/claims.rs)
- [NIST Digital Thread for Manufacturing](https://www.nist.gov/programs-projects/digital-thread-manufacturing)
- [ISO 23952:2020 QIF](https://www.iso.org/standard/77461.html)
- [W3C VC Data Model 2.0](https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/)
- [Holochain validation guidance](https://developer.holochain.org/build/validation/)
- [in-toto Attestation Framework](https://github.com/in-toto/attestation/blob/main/spec/README.md)

This is a proposed integration architecture. The receipt type, adapter and bridge calls are not implemented. No security audit, independent interoperability test or production readiness is claimed.


## Shared benchmark contract

The counterpart OMSI draft PR #53 now proposes a versioned [benchmark scorecard](https://github.com/Luminous-Dynamics/luminous-platform/blob/feat/omsi-manufacturing-rust-validator/docs/standards/omsi/benchmark-scorecard-0.1.md), a non-gating phase-separated microbenchmark, a hard adapter field-accounting gate, and a separate [10-case transform-vector corpus](https://github.com/Luminous-Dynamics/luminous-platform/blob/feat/omsi-manufacturing-rust-validator/docs/standards/omsi/adapters/0.1/TRANSFORM-VECTORS.json). Reuse its semantic-vector and replay mutation catalogue rather than inventing a second set of result semantics.

Mycelix-specific acceptance targets:
- **100%** exact OMSI vector agreement before receipt creation; **zero** unexpected accepts in cross-tenant, malformed or contradictory evidence controls.
- **Zero silent field loss**: each adapter field is preserved, explicitly transformed under a named rule, or recorded as loss/unmapped.
- **Zero prohibited sensitive fields** in public receipts.
- At least **two agents** independently verify receipt/envelope references, with explicit negative scenarios for unauthorized issuers/routes, expired or revoked credentials, duplicates, missing DHT dependencies and conflicting claims.
- Keep performance separate by phase: local OMSI evaluation, credential verification/status checks, receipt creation/resolution, and end-to-end multi-agent flow. Any latency goals remain provisional until corpus, host, Holochain version, compiler and test topology are recorded.

These are acceptance targets, not measured results. Local unit tests and queued CI do not establish multi-agent qualification, security audit or production readiness.
