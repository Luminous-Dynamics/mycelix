# NeuroTwin × Mycelix research governance and integration contract v0.1

**Status:** architecture proposal; not deployed, implemented as an end-to-end authorization service, ethics approval, legal compliance certification or security audit.  
**Canonical repository:** [Luminous-Dynamics/mycelix](https://github.com/Luminous-Dynamics/mycelix).  
**Tracking:** [NEUROTWIN-001 / #6276](https://github.com/Luminous-Dynamics/mycelix/issues/6276).

## 1. Architectural decision

NeuroTwin is a research capability composed over existing Mycelix owners. It is not a second Health Vault, a new identity provider, or a clinical-authority layer.

| Existing owner | NeuroTwin relationship | Does not imply |
|---|---|---|
| **Personal / Health Vault** | Participant-controlled health-record source and selective-disclosure boundary | That current consent records already authorize research access or that raw payloads are encrypted/private in every deployed DNA |
| **Identity** | Establishes the cryptographic identity/credential presentation of a researcher, institution or service where the issuer and credential scope are trusted | That a DID, signed claim or trust score proves professional licence, ethics approval or lawful purpose |
| **Health** | Potential domain integration for an institutionally governed clinical workflow | A live clinical service, approved diagnostic model or authority to write research inferences into care records |
| **DeSci / research provenance** | Records protocol versions, review decisions, source lineage and aggregate research artifacts | That a hash proves source truth, dataset validity, consent or clinical correctness |
| **Core FL / 0TML** | Optional governed computation across approved sites | That gradients, embeddings or model updates are anonymous or that federated learning removes the need for data-use approval |
| **Symthaea / NeuroTwin models** | Candidate research inference and representation learning | Clinical decision authority or self-certification |

The repository root currently includes a separate `mycelix-health` Git submodule. This NeuroTwin programme is owned by the main Mycelix repository and must not depend on writing its canonical protocol or issue tracking into that submodule.

## 2. Current Health Vault reality and hard dependencies

The current Personal Health Vault implementation must be treated conservatively:

- `HealthRecord.data` is a string payload; the coordinator's current create/view path writes and returns it as supplied. The existing PQC roadmap also identifies client-side at-rest encryption integration as unfinished.
- The original `ConsentGrant` shape contains a grantee, record-type list, optional expiry, active flag and creation time. That is not a complete research authorization: study, purpose, field scope, modality, secondary use, retention, commercial use and enforcement context are not represented there.
- Owner-local consent listing is not a grantee-side protected read path. UI display is not authorization enforcement.
- Unannotated Health Vault entry types and owner/record-type links are being addressed by privacy work. Do not treat comments or source-chain-local UI as proof that DHT content/metadata are private.
- An append-only owner-side revocation event does not prove that every consumer, cache, copy or disconnected peer immediately enforces revocation.

**Hard dependencies:**
- [#5315 HEALTH-PRIVACY-001](https://github.com/Luminous-Dynamics/mycelix/issues/5315) and draft [PR #5317](https://github.com/Luminous-Dynamics/mycelix/pull/5317) for private entry definitions and metadata minimization.
- [#5240 CONSENT-LIFECYCLE-001](https://github.com/Luminous-Dynamics/mycelix/issues/5240), [PR #5271](https://github.com/Luminous-Dynamics/mycelix/pull/5271) and [PR #5279](https://github.com/Luminous-Dynamics/mycelix/pull/5279) for expiry and append-only owner-side revocation lifecycle.
- [#5281 CONSENT-ENFORCEMENT-001](https://github.com/Luminous-Dynamics/mycelix/issues/5281) for caller-bound grantee access and enforcement at the actual protected read/presentation boundary.

These PRs are open/draft and not fully qualified. The NeuroTwin protocol does not treat them as merged, deployed or independently validated.

### Legacy-data warning

A new private-entry definition cannot retroactively make data private if earlier DNA versions published the entry content or correlating links to a shared DHT. Any migration requires a separate user-controlled export/re-encryption plan, a clean versioned DNA/network decision, explicit legacy-exposure classification and a tested migration path. Never label a legacy public entry “private” solely because a newer source definition has private visibility.

## 3. Research-use authority is a composed decision

A NeuroTwin run is admitted only when every relevant authority is independently present:

1. **Participant authority:** the participant's valid, informed, study-specific consent as collected and maintained by the responsible study institution. A Mycelix wallet record alone does not replace the institution's consent process.
2. **Institutional authority:** documented REB/IRB approval or exemption and institutional authorization for the exact project and analysis.
3. **Data-provider authority:** an executed DUA, named approved users, controlled workspace, permitted variables/modalities and publication constraints.
4. **Recipient authority:** authenticated researcher/service identity and a verified role credential whose issuer, validity, revocation and scope have been checked.
5. **Purpose authority:** the exact approved study, protocol version, analysis purpose, modalities, derived-data operations, retention and dissemination scope.
6. **Technical enforcement:** the consuming boundary checks the requester, original grant, exact scope, expiry and current revocation state; unavailable or stale authority fails closed.
7. **Independent evidence:** a reproducible receipt records which policy and source versions governed the run, without exposing participant-level data.

A failure at any one of these gates means **not admitted**. A signature, reputation score, immutable record, privacy budget or model result cannot compensate for missing legal or institutional authority.

## 4. Proposed research-use manifest (not participant consent)

Before a controlled analysis, the study operator should produce a versioned **study-use manifest** in the approved controlled environment. This manifest describes the approved study-level scope; it is not a participant consent form, a substitute for a Holochain grant, or a global authorization token.

Minimum fields:
- stable study identifier and protocol version/digest;
- data-provider and exact dataset release identity;
- ethics approval/exemption reference and validity period;
- DUA reference, approved user group and approved compute location;
- allowed analysis purposes (e.g. retrospective prognosis, external validation, randomized treatment-effect estimation);
- allowed modalities and approved derived features;
- prohibited use (diagnosis, prescribing, autonomous triage, unrelated secondary use, re-identification, onward transfer);
- model training/update and federated-computation permissions;
- retention and deletion/return deadline;
- publication/disclosure policy, minimum-cell threshold and suppression rule;
- named study data custodian and incident/withdrawal contact.

The manifest should be signed by the relevant institution/data custodian, bound to exact protocol and dataset versions, and stored in the source-controlled research environment. A public Mycelix artifact may record only a reviewed, non-identifying commitment/reference and lifecycle metadata. Do not place participant consents, PHI, raw data, per-person record hashes, exact enrolment dates or study linkage keys in public DHT/IPFS.

## 5. Data flow

```
Participant + study institution
    │  institution's consent / REB / DUA authority
    ▼
Personal Health Vault (owner-controlled source)
    │  purpose- and field-bounded, caller-authenticated disclosure
    ▼
Approved institutional research workspace
    │  source-controlled preprocessing + locked split manifest
    ▼
NeuroTwin / Symthaea candidate models
    │  aggregate evaluation only; uncertainty + abstention
    ▼
Independent analysis review
    │  disclosure-controlled aggregate receipt
    ▼
Mycelix DeSci / provenance layer
       protocol lineage, approvals, code identity, reviewed aggregate artifacts
```

Raw MRI, dMRI, EEG, clinical notes, direct identifiers and re-identification keys stay within the authorized controlled environment. Public DHT and IPFS are not research data lakes.

For external controlled datasets such as CAN-BIND-1, analysis remains within the provider's approved environment (Brain-CODE workspace where required). EMBARC/NIMH NDA data use remains subject to the current NDA request and DUA pathway. This repository work does not provide access or authorize data export.

## 6. Federated learning and privacy engineering

Federated learning is an execution topology, not a privacy proof. Before enabling it:

- obtain approval for local processing and model-update transfer at every participating site;
- specify secure aggregation, participant/site dropout behaviour, authentication and replay protection;
- threat-model gradient inversion, membership inference, model memorization and malicious/poisoned updates;
- apply differential privacy only with a declared unit of privacy, adjacency definition, clipping rule, noise mechanism and accountant;
- compose privacy budgets across rounds, modalities, retries and model releases; report ((epsilon,delta)) and assumptions;
- test utility loss and calibration under the same external splits;
- prevent an untrusted model update or trust score from becoming clinical evidence or authorization.

A DP label with no unit, accountant, composition report or threat model is not a privacy guarantee. A federated gradient or learned representation remains potentially sensitive.

## 7. Provenance and receipt policy

Every analysis receipt should bind:
- study-use manifest and protocol digests;
- source provider, release identifier and approved scope reference;
- exact code commit, dependency/environment identity and preprocessing digest;
- split manifest and evaluation direction;
- model identity, training configuration digest and inference-time feature contract;
- aggregate cohort denominator (subject to disclosure review);
- prespecified metrics, uncertainty, calibration and decision-utility results;
- failed gates, exclusions, deviations and known limitations.

The aggregate receipt schema is [evaluation-receipt-v0.1.schema.json](schemas/evaluation-receipt-v0.1.schema.json). It deliberately has no field for participant identifiers, row-level predictions, raw modalities, embeddings or gradients. The schema is an artifact contract only; no Mycelix service currently enforces it.

Only reviewed aggregate artifacts may be published. Hashes can still enable linkage or confirmation attacks if an attacker can guess the input, so hashes of per-person records, small cohorts or low-entropy source files are prohibited. Use access-controlled manifests and selective disclosure for detailed audit.

## 8. Interoperability without authority confusion

- Use HL7 FHIR Consent as an interoperability mapping target for actors, scope, purpose, period and permit/deny semantics; do not claim the current Mycelix ConsentGrant is a full FHIR Consent resource.
- Use BIDS conventions and the official BIDS Validator for imaging dataset structure; a structural pass does not prove participant consent, phenotype quality, endpoint validity or absence of leakage.
- Map source identity, version and provenance without turning a valid digital signature into proof that the clinical content is true.
- Treat model-derived features and embeddings as potentially identifying health data until a specific threat model and disclosure review establish otherwise.

## 9. Failure semantics

| Condition | Required state |
|---|---|
| Missing REB/IRB or DUA | Not admitted |
| Wrong or expired study scope | Denied |
| Unknown recipient role / invalid credential | Denied |
| Consent revoked, expired, superseded or unknown | No new protected read |
| Revocation source unavailable/stale | Indeterminate; fail closed |
| Dataset release or protocol mismatch | Quarantined |
| Missing modality or unsupported population | Model abstains or declared clinical-only baseline; never synthetic backfill |
| Out-of-distribution site/device/population | Abstain or separately qualify |
| Failed or incomplete run | No PASS receipt |
| Small-cell or linkage risk | Suppress / withhold publication |

Revocation can govern future access and processing where technically and legally possible; it cannot guarantee recall of data already copied, models already trained, or published aggregate outputs. The consent UI and study documents must describe those limits clearly.

## 10. Qualification targets

These are proposed acceptance targets, not measured results:

- zero unauthorised responses in a frozen multi-agent test suite at every protected read/presentation boundary;
- 100% of the frozen lifecycle cases distinguish granted, expired, revoked, superseded, unknown and source-unavailable states;
- zero raw participant-level data or sensitive derivatives in public artifacts, DHT, IPFS, repository logs or fixtures;
- 100% of evaluation receipts bind the exact study manifest, protocol, source release, code commit and split/preprocessing digests;
- 100% of unsupported, stale or out-of-scope model requests abstain or fail closed;
- zero clinical-action authority inferred from a model result, DID, trust score, signature or provenance receipt;
- report p50/p95/p99 authorized-read and receipt-generation latency only after security correctness is qualified.

No throughput target can compensate for an authorization or privacy failure.

## 11. Maturity boundary

Current status is design/research only. The existing Personal Health Vault has important open privacy and consent-enforcement work, and the separate Health cluster is not a substitute for those protections. This contract does not establish POPIA/HIPAA/GDPR compliance, ethics approval, clinical validity, device status, production readiness, independent security assurance or patient benefit.
