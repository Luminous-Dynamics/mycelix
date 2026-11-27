# Health cluster integration and qualification contract

**Status:** integration contract; not a clinical deployment qualification
**Canonical source:** `Luminous-Dynamics/mycelix-health`
**Umbrella location:** root Git submodule `mycelix-health/`

## 1. Source-of-truth model

- `mycelix-health/` is the canonical Health hApp source repository.
- The main `mycelix` repository pins one exact Health commit as a Git submodule.
- `mycelix-workspace/happs/health` is only a relative symlink used by DNA build and unified-hApp packaging. It must not become a copied or independently edited mirror.
- Shared contracts and reusable infrastructure remain in their canonical Mycelix crates/SDKs. Health may consume sibling shared crates through the umbrella checkout; the supported integration build therefore initializes the parent repository's submodules and resolves those dependencies from the pinned umbrella tree.

Initialize and check the source before building:

```bash
git submodule update --init --recursive
cd mycelix-workspace
just health-integration-check
just build-health
```

The check verifies the root submodule declaration, canonical URL, relative symlink, initialized checkout, exact parent gitlink pin, required Health source files, and the unified hApp's `health` role/path.

## 2. Pin-update policy

A submodule pin is a dependency lock, not a floating branch pointer. An update must be a deliberate parent-repository change and should include:

1. exact upstream Health commit SHA and source diff;
2. compatibility review of shared-crate and SDK contracts;
3. Health native tests and warnings-denied Clippy where supported;
4. WASM build and DNA packaging;
5. unified-hApp manifest and role wiring validation;
6. multi-agent Sweettest for consent, caller identity, private data, revocation and restart/recovery behavior;
7. an explicit record of checks that are pending, skipped, structurally checked or passed.

A queued/skipped workflow is not a pass. A submodule pointer update does not itself prove compatibility.

## 3. Clinical and privacy boundary

- The Health DNA remains a distinct domain role. Cross-role calls must pass through explicit, typed contracts and authorization.
- Patient/clinical payloads must not be copied into public indexes or replicated merely to simplify cross-cluster queries.
- Consent must be purpose-, category-, actor- and time-bound; authorization failure or unavailable authority must fail closed.
- Clinical corrections and source conflicts must preserve the competing assertions and provenance. No generic timestamp or similarity heuristic may silently decide which clinical fact is true.
- AI-generated interpretation is a derived assertion, never a source record. It must carry model/version, inputs, evidence references, uncertainty and human-review state.
- Build success, schema validation, and unit tests do not establish clinical safety, regulatory compliance or permission to process real patient data.

## 4. Required qualification layers

| Layer | Minimum evidence |
|---|---|
| Source wiring | Exact submodule pin, canonical relative symlink, manifest role |
| Build | Health WASM build, DNA package, unified hApp package |
| Interoperability | FHIR R4 resource validation against selected implementation guides; loss-aware round-trip corpus |
| Identity and access | Caller-bound consent tests, wrong-agent negative controls, expiry/revocation and network-failure tests |
| Privacy | DHT visibility review, metadata-leakage census, access-response minimization |
| Clinical data quality | Patient matching, terminology/version retention, provenance, correction and reconciliation corpus |
| Multi-agent behavior | Sweettest with owner, authorized provider, unauthorized provider, and attacker |
| AI assurance | Clinician-adjudicated benchmark, abstention, calibration, subgroup and distribution-shift analysis |
| Operations | Backup/recovery, key rotation, incident response, audit review and exportability |
| Independent review | External cryptographic, security, privacy, clinical and legal assessment |

## 5. Standards and conformance strategy

Keep FHIR R4 as the initial compatibility baseline because the existing gateway and Health SDK target R4. Add profiles/adapters as explicit versioned packages rather than claiming that generic FHIR JSON acceptance equals national or international conformance.

Candidate profiles to evaluate include:
- HL7 FHIR R4 base validation and SMART App Launch;
- International Patient Summary (IPS) for portable essential clinical summaries;
- applicable South African National Health Normative Standards Framework (HNSF) and national implementation guidance;
- US Core only where the target integration or evaluation explicitly requires US-realm conformance.

Use the HL7 FHIR validator plus server/client conformance suites such as Touchstone and Inferno where the specific profile and test coverage apply. Preserve each test's profile/version and validator version in the result receipt.

## 6. Evidence and promotion language

Use these distinct states in all dashboards and release notes:

- `Designed`
- `Implemented`
- `StructurallyChecked`
- `UnitTested`
- `MultiAgentTested`
- `ConformanceTested`
- `IndependentlyAudited`
- `ClinicallyValidated`
- `ProductionQualified`

Do not promote a capability to a stronger state by inference. This contract is an integration starting point, not a certification.
