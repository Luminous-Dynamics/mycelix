//! Pre-dispatch operation-contract registry validation for MCRB.
//!
//! The registry freezes what an offered operation is supposed to establish
//! before dispatch. This module validates schema-level structure, exact
//! scenario linkage and operation semantics, and produces content-bound
//! operation-contract digests.
//!
//! It does NOT authenticate the registry owner, signers, evidence sources,
//! builder, runner or external effects. The caller must supply the registry
//! artifact digest and source manifest Git blob SHA from protected provenance.
//! Outcome-source independence and signatures remain owned by #6140/#6357.

use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

const EMBEDDED_SCENARIO_MANIFEST: &str =
    include_str!("../../../../docs/benchmarks/MCRB_V0_1_SCENARIO_MANIFEST.json");

pub const OPERATION_CONTRACT_REGISTRY_SCHEMA: &str =
    "mycelix-mcrb-operation-contract-registry-v0.1";
const OPERATION_CONTRACT_DIGEST_DOMAIN: &[u8] =
    b"MYCELIX-MCRB-OPERATION-CONTRACT\0v0.1\0";

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum RegistryStatus {
    ProposedNotExecuted,
    Reviewed,
    Qualified,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum EffectBoundary {
    LocalSourceChain,
    RemoteDht,
    PolicyDecision,
    ExternalProviderEffect,
    Recovery,
    ReadObservation,
    DeploymentAudit,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ExpectedDisposition {
    MustApply,
    MustDeny,
    MustObserve,
    MustReconcile,
    DiagnosticOnly,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum OperationOutcomeClass {
    QualifiedSuccess,
    CorrectDenial,
    UnexpectedDenial,
    Failed,
    Unknown,
    DuplicateEffect,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum AttemptState {
    NotDispatched,
    Dispatched,
    AcceptedAtLocalBoundary,
    TimedOut,
    Reconciled,
    Unsupported,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum EffectState {
    Applied,
    ConfirmedNotApplied,
    Partial,
    Unknown,
    NotAttempted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum EvidenceKind {
    ApplicationRecord,
    ProviderAcknowledgement,
    ProviderStatusQuery,
    IndependentObservation,
    PolicyDecision,
    FailureTrace,
    Reconciliation,
    CrossPeerRead,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum EvidenceSourceRole {
    EffectExecutor,
    Provider,
    PolicyEngine,
    IndependentObserver,
    ReconciliationAuthority,
    Telemetry,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OperationContractRegistryDocument {
    pub schema_version: String,
    pub registry_id: String,
    pub registry_version: String,
    pub status: RegistryStatus,
    pub source_scenario_manifest_git_blob_sha: String,
    pub contracts: Vec<OperationContractDocument>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OperationContractDocument {
    pub contract_id: String,
    pub contract_version: String,
    pub scenario_id: String,
    pub operation_class: String,
    pub effect_boundary: EffectBoundary,
    pub expected_disposition: ExpectedDisposition,
    pub qualification_proposition: QualificationProposition,
    pub expected_qops_outcome: OperationOutcomeClass,
    pub attempt_state_policy: AttemptStatePolicy,
    pub effect_state_policy: EffectStatePolicy,
    pub evidence_policy: EvidencePolicy,
    pub idempotency_required: bool,
    pub correct_denial_policy: DenialPolicy,
    pub unexpected_denial_policy: DenialPolicy,
    pub unknown_policy: UnknownPolicy,
    pub partial_policy: PartialPolicy,
    #[serde(default)]
    pub notes: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QualificationProposition {
    pub proposition_id: String,
    pub verifier_profile: String,
    pub description: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AttemptStatePolicy {
    pub allowed_states: Vec<AttemptState>,
    pub terminal_states: Vec<AttemptState>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EffectStatePolicy {
    pub allowed_states: Vec<EffectState>,
    pub expected_effect_states: Vec<EffectState>,
    pub requires_reconciliation: Vec<EffectState>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidencePolicy {
    pub allowed_evidence_kinds: Vec<EvidenceKind>,
    pub required_evidence_kinds: Vec<EvidenceKind>,
    pub allowed_source_roles: Vec<EvidenceSourceRole>,
    pub requires_independent_observer: bool,
    pub minimum_distinct_observer_sources: usize,
    pub requires_cross_host_visibility: bool,
    pub requires_complete_coverage: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DenialPolicy {
    pub enabled: bool,
    pub required_evidence_kinds: Vec<EvidenceKind>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct UnknownPolicy {
    pub blocks_qops: bool,
    pub requires_reconciliation: bool,
    pub preserve_prior_attestation: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PartialPolicy {
    pub blocks_qops: bool,
    pub requires_explicit_expected_action_set: bool,
    pub requires_reconciliation: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OperationContractIssue {
    InvalidJson(String),
    UnsupportedSchema(String),
    EmptyRegistryIdentity,
    SourceManifestGitBlobMismatch { expected: String, actual: String },
    InvalidRegistryDigest,
    RegistryDigestMismatch { expected: String, actual: String },
    EmptyContractRegistry,
    EmptyContractField { contract_id: String, field: &'static str },
    DuplicateContractId(String),
    UnknownScenario(String),
    ScenarioVisibilityWeakened { scenario_id: String },
    DuplicatePolicyState { contract_id: String, field: &'static str },
    EmptyPolicyStateSet { contract_id: String, field: &'static str },
    TerminalAttemptNotAllowed { contract_id: String, state: AttemptState },
    ExpectedEffectNotAllowed { contract_id: String, state: EffectState },
    RequiredEvidenceNotAllowed { contract_id: String, kind: EvidenceKind },
    IndependentObserverPolicyIncomplete { contract_id: String },
    IndependentObserverSourceRoleMissing { contract_id: String },
    DenialEvidenceNotAllowed { contract_id: String, kind: EvidenceKind },
    UnknownPolicyMustFailClosed { contract_id: String },
    PartialPolicyMustFailClosed { contract_id: String },
    DispositionOutcomeMismatch { contract_id: String },
    QualifiedSuccessRequiresAppliedEffect { contract_id: String },
    CorrectDenialRequiresConfirmedNoEffect { contract_id: String },
    MustDenyRequiresCorrectDenialEvidence { contract_id: String },
    ContractSerialization(String),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ValidatedOperationContract {
    document: OperationContractDocument,
    digest_sha256: String,
}

impl ValidatedOperationContract {
    pub fn contract_id(&self) -> &str {
        &self.document.contract_id
    }

    pub fn contract_version(&self) -> &str {
        &self.document.contract_version
    }

    pub fn scenario_id(&self) -> &str {
        &self.document.scenario_id
    }

    pub fn operation_class(&self) -> &str {
        &self.document.operation_class
    }

    pub fn digest_sha256(&self) -> &str {
        &self.digest_sha256
    }

    pub fn expected_qops_outcome(&self) -> OperationOutcomeClass {
        self.document.expected_qops_outcome
    }

    pub fn expected_disposition(&self) -> ExpectedDisposition {
        self.document.expected_disposition
    }

    pub fn requires_cross_host_visibility(&self) -> bool {
        self.document.evidence_policy.requires_cross_host_visibility
    }

    pub fn requires_independent_observer(&self) -> bool {
        self.document.evidence_policy.requires_independent_observer
    }

    pub fn idempotency_required(&self) -> bool {
        self.document.idempotency_required
    }

    pub fn allows_effect_state(&self, state: EffectState) -> bool {
        self.document.effect_state_policy.allowed_states.contains(&state)
    }

    pub fn expects_effect_state(&self, state: EffectState) -> bool {
        self.document.effect_state_policy.expected_effect_states.contains(&state)
    }

    pub fn allows_attempt_state(&self, state: AttemptState) -> bool {
        self.document.attempt_state_policy.allowed_states.contains(&state)
    }

    pub fn terminal_attempt_states(&self) -> &[AttemptState] {
        &self.document.attempt_state_policy.terminal_states
    }

    pub fn expected_effect_states(&self) -> &[EffectState] {
        &self.document.effect_state_policy.expected_effect_states
    }

    pub fn requires_reconciliation(&self, state: EffectState) -> bool {
        self.document.effect_state_policy.requires_reconciliation.contains(&state)
    }

    pub fn allowed_evidence_kinds(&self) -> &[EvidenceKind] {
        &self.document.evidence_policy.allowed_evidence_kinds
    }

    pub fn required_evidence_kinds(&self) -> &[EvidenceKind] {
        &self.document.evidence_policy.required_evidence_kinds
    }

    pub fn allows_evidence_kind(&self, kind: EvidenceKind) -> bool {
        self.document.evidence_policy.allowed_evidence_kinds.contains(&kind)
    }

    pub fn allowed_source_roles(&self) -> &[EvidenceSourceRole] {
        &self.document.evidence_policy.allowed_source_roles
    }

    pub fn allows_source_role(&self, role: EvidenceSourceRole) -> bool {
        self.document.evidence_policy.allowed_source_roles.contains(&role)
    }

    pub fn minimum_distinct_observer_sources(&self) -> usize {
        self.document.evidence_policy.minimum_distinct_observer_sources
    }

    pub fn requires_complete_coverage(&self) -> bool {
        self.document.evidence_policy.requires_complete_coverage
    }

    pub fn correct_denial_required_evidence_kinds(&self) -> &[EvidenceKind] {
        &self.document.correct_denial_policy.required_evidence_kinds
    }

    pub fn unexpected_denial_required_evidence_kinds(&self) -> &[EvidenceKind] {
        &self.document.unexpected_denial_policy.required_evidence_kinds
    }
}

/// Opaque, structurally validated registry. This is not a signature/trust
/// capability and does not establish that any operation actually occurred.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ValidatedOperationContractRegistry {
    registry_id: String,
    registry_version: String,
    registry_digest_sha256: String,
    source_scenario_manifest_git_blob_sha: String,
    contracts: BTreeMap<(String, String), ValidatedOperationContract>,
}

impl ValidatedOperationContractRegistry {
    pub fn registry_id(&self) -> &str {
        &self.registry_id
    }

    pub fn registry_version(&self) -> &str {
        &self.registry_version
    }

    pub fn registry_digest_sha256(&self) -> &str {
        &self.registry_digest_sha256
    }

    pub fn source_scenario_manifest_git_blob_sha(&self) -> &str {
        &self.source_scenario_manifest_git_blob_sha
    }

    pub fn contract_count(&self) -> usize {
        self.contracts.len()
    }

    pub fn find(
        &self,
        contract_id: &str,
        contract_version: &str,
    ) -> Option<&ValidatedOperationContract> {
        self.contracts
            .get(&(contract_id.to_string(), contract_version.to_string()))
    }
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut out = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut out, "{byte:02x}");
    }
    out
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

fn valid_git_blob_sha(value: &str) -> bool {
    value.len() == 40 && value.bytes().all(|b| b.is_ascii_hexdigit())
}

fn has_duplicates<T: Ord + Copy>(items: &[T]) -> bool {
    let mut seen = BTreeSet::new();
    items.iter().any(|item| !seen.insert(*item))
}

fn scenario_visibility(scenario_id: &str) -> Result<bool, OperationContractIssue> {
    let manifest: Value = serde_json::from_str(EMBEDDED_SCENARIO_MANIFEST)
        .map_err(|e| OperationContractIssue::InvalidJson(e.to_string()))?;
    let scenarios = manifest["scenarios"]
        .as_array()
        .ok_or_else(|| OperationContractIssue::InvalidJson(
            "embedded scenario manifest has no scenarios array".to_string()
        ))?;
    let scenario = scenarios
        .iter()
        .find(|scenario| scenario["id"].as_str() == Some(scenario_id))
        .ok_or_else(|| OperationContractIssue::UnknownScenario(scenario_id.to_string()))?;
    scenario["qops_requires_cross_host_visibility"]
        .as_bool()
        .ok_or_else(|| OperationContractIssue::InvalidJson(format!(
            "scenario '{scenario_id}' lacks qops_requires_cross_host_visibility"
        )))
}

fn operation_contract_digest(
    document: &OperationContractDocument,
) -> Result<String, OperationContractIssue> {
    let json = serde_json::to_vec(document)
        .map_err(|e| OperationContractIssue::ContractSerialization(e.to_string()))?;
    let mut material = Vec::with_capacity(OPERATION_CONTRACT_DIGEST_DOMAIN.len() + json.len());
    material.extend_from_slice(OPERATION_CONTRACT_DIGEST_DOMAIN);
    material.extend_from_slice(&json);
    Ok(sha256_hex(&material))
}

fn validate_contract(
    document: &OperationContractDocument,
) -> Result<(), Vec<OperationContractIssue>> {
    let id = document.contract_id.clone();
    let mut issues = Vec::new();

    let fields = [
        ("contract_id", document.contract_id.as_str()),
        ("contract_version", document.contract_version.as_str()),
        ("scenario_id", document.scenario_id.as_str()),
        ("operation_class", document.operation_class.as_str()),
        ("proposition_id", document.qualification_proposition.proposition_id.as_str()),
        ("verifier_profile", document.qualification_proposition.verifier_profile.as_str()),
        ("proposition_description", document.qualification_proposition.description.as_str()),
    ];
    for (field, value) in fields {
        if value.trim().is_empty() {
            issues.push(OperationContractIssue::EmptyContractField {
                contract_id: id.clone(),
                field,
            });
        }
    }

    if has_duplicates(&document.attempt_state_policy.allowed_states) {
        issues.push(OperationContractIssue::DuplicatePolicyState {
            contract_id: id.clone(),
            field: "attempt_state_policy.allowed_states",
        });
    }
    if has_duplicates(&document.attempt_state_policy.terminal_states) {
        issues.push(OperationContractIssue::DuplicatePolicyState {
            contract_id: id.clone(),
            field: "attempt_state_policy.terminal_states",
        });
    }
    if document.attempt_state_policy.allowed_states.is_empty()
        || document.attempt_state_policy.terminal_states.is_empty()
    {
        issues.push(OperationContractIssue::EmptyPolicyStateSet {
            contract_id: id.clone(),
            field: "attempt_state_policy",
        });
    }
    for state in &document.attempt_state_policy.terminal_states {
        if !document.attempt_state_policy.allowed_states.contains(state) {
            issues.push(OperationContractIssue::TerminalAttemptNotAllowed {
                contract_id: id.clone(),
                state: *state,
            });
        }
    }

    if has_duplicates(&document.effect_state_policy.allowed_states)
        || has_duplicates(&document.effect_state_policy.expected_effect_states)
        || has_duplicates(&document.effect_state_policy.requires_reconciliation)
    {
        issues.push(OperationContractIssue::DuplicatePolicyState {
            contract_id: id.clone(),
            field: "effect_state_policy",
        });
    }
    if document.effect_state_policy.allowed_states.is_empty()
        || document.effect_state_policy.expected_effect_states.is_empty()
    {
        issues.push(OperationContractIssue::EmptyPolicyStateSet {
            contract_id: id.clone(),
            field: "effect_state_policy",
        });
    }
    for state in &document.effect_state_policy.expected_effect_states {
        if !document.effect_state_policy.allowed_states.contains(state) {
            issues.push(OperationContractIssue::ExpectedEffectNotAllowed {
                contract_id: id.clone(),
                state: *state,
            });
        }
    }
    for state in &document.effect_state_policy.requires_reconciliation {
        if !document.effect_state_policy.allowed_states.contains(state) {
            issues.push(OperationContractIssue::ExpectedEffectNotAllowed {
                contract_id: id.clone(),
                state: *state,
            });
        }
    }

    if has_duplicates(&document.evidence_policy.allowed_evidence_kinds)
        || has_duplicates(&document.evidence_policy.required_evidence_kinds)
        || has_duplicates(&document.evidence_policy.allowed_source_roles)
    {
        issues.push(OperationContractIssue::DuplicatePolicyState {
            contract_id: id.clone(),
            field: "evidence_policy",
        });
    }
    for kind in &document.evidence_policy.required_evidence_kinds {
        if !document.evidence_policy.allowed_evidence_kinds.contains(kind) {
            issues.push(OperationContractIssue::RequiredEvidenceNotAllowed {
                contract_id: id.clone(),
                kind: *kind,
            });
        }
    }
    if document.evidence_policy.requires_independent_observer
        && document.evidence_policy.minimum_distinct_observer_sources == 0
    {
        issues.push(OperationContractIssue::IndependentObserverPolicyIncomplete {
            contract_id: id.clone(),
        });
    }
    if document.evidence_policy.requires_independent_observer
        && !document.evidence_policy.allowed_source_roles.contains(&EvidenceSourceRole::IndependentObserver)
    {
        issues.push(OperationContractIssue::IndependentObserverSourceRoleMissing {
            contract_id: id.clone(),
        });
    }
    for kind in document
        .correct_denial_policy
        .required_evidence_kinds
        .iter()
        .chain(document.unexpected_denial_policy.required_evidence_kinds.iter())
    {
        if !document.evidence_policy.allowed_evidence_kinds.contains(kind) {
            issues.push(OperationContractIssue::DenialEvidenceNotAllowed {
                contract_id: id.clone(),
                kind: *kind,
            });
        }
    }

    if !document.unknown_policy.blocks_qops
        || !document.unknown_policy.requires_reconciliation
        || !document.unknown_policy.preserve_prior_attestation
    {
        issues.push(OperationContractIssue::UnknownPolicyMustFailClosed {
            contract_id: id.clone(),
        });
    }
    if !document.partial_policy.blocks_qops
        || !document.partial_policy.requires_explicit_expected_action_set
        || !document.partial_policy.requires_reconciliation
    {
        issues.push(OperationContractIssue::PartialPolicyMustFailClosed {
            contract_id: id.clone(),
        });
    }

    let disposition_matches = match document.expected_disposition {
        ExpectedDisposition::MustApply | ExpectedDisposition::MustObserve => {
            document.expected_qops_outcome == OperationOutcomeClass::QualifiedSuccess
        }
        ExpectedDisposition::MustDeny => {
            document.expected_qops_outcome == OperationOutcomeClass::CorrectDenial
        }
        ExpectedDisposition::MustReconcile | ExpectedDisposition::DiagnosticOnly => {
            document.expected_qops_outcome == OperationOutcomeClass::Unknown
        }
    };
    if !disposition_matches {
        issues.push(OperationContractIssue::DispositionOutcomeMismatch {
            contract_id: id.clone(),
        });
    }

    match document.expected_qops_outcome {
        OperationOutcomeClass::QualifiedSuccess => {
            if !document.effect_state_policy.expected_effect_states.contains(&EffectState::Applied) {
                issues.push(OperationContractIssue::QualifiedSuccessRequiresAppliedEffect {
                    contract_id: id.clone(),
                });
            }
        }
        OperationOutcomeClass::CorrectDenial => {
            if document.effect_state_policy.expected_effect_states.contains(&EffectState::Applied) {
                issues.push(OperationContractIssue::DispositionOutcomeMismatch {
                    contract_id: id.clone(),
                });
            }
            if !document.effect_state_policy.expected_effect_states.contains(&EffectState::ConfirmedNotApplied) {
                issues.push(OperationContractIssue::CorrectDenialRequiresConfirmedNoEffect {
                    contract_id: id.clone(),
                });
            }
            if !document.correct_denial_policy.enabled
                || document.correct_denial_policy.required_evidence_kinds.is_empty()
            {
                issues.push(OperationContractIssue::MustDenyRequiresCorrectDenialEvidence {
                    contract_id: id.clone(),
                });
            }
        }
        OperationOutcomeClass::UnexpectedDenial
        | OperationOutcomeClass::Failed
        | OperationOutcomeClass::Unknown
        | OperationOutcomeClass::DuplicateEffect => {}
    }

    if issues.is_empty() {
        Ok(())
    } else {
        Err(issues)
    }
}

/// Validate an operation-contract registry against externally pinned bytes
/// and the frozen scenario manifest's Git blob identity.
///
/// The returned capability proves only structural consistency and exact
/// contract selection. It does not authenticate who approved the registry or
/// establish operation outcomes; those trust/evidence layers remain separate.
pub fn validate_operation_contract_registry(
    raw_registry: &[u8],
    expected_registry_digest_sha256: &str,
    expected_scenario_manifest_git_blob_sha: &str,
) -> Result<ValidatedOperationContractRegistry, Vec<OperationContractIssue>> {
    let actual_registry_digest = sha256_hex(raw_registry);
    if !valid_sha256(expected_registry_digest_sha256) {
        return Err(vec![OperationContractIssue::InvalidRegistryDigest]);
    }
    if actual_registry_digest != expected_registry_digest_sha256 {
        return Err(vec![OperationContractIssue::RegistryDigestMismatch {
            expected: expected_registry_digest_sha256.to_string(),
            actual: actual_registry_digest,
        }]);
    }
    if !valid_git_blob_sha(expected_scenario_manifest_git_blob_sha) {
        return Err(vec![OperationContractIssue::SourceManifestGitBlobMismatch {
            expected: expected_scenario_manifest_git_blob_sha.to_string(),
            actual: "invalid-expected-git-blob-sha".to_string(),
        }]);
    }

    let document: OperationContractRegistryDocument = serde_json::from_slice(raw_registry)
        .map_err(|e| vec![OperationContractIssue::InvalidJson(e.to_string())])?;
    let mut issues = Vec::new();

    if document.schema_version != OPERATION_CONTRACT_REGISTRY_SCHEMA {
        issues.push(OperationContractIssue::UnsupportedSchema(document.schema_version.clone()));
    }
    if document.registry_id.trim().is_empty() || document.registry_version.trim().is_empty() {
        issues.push(OperationContractIssue::EmptyRegistryIdentity);
    }
    if document.source_scenario_manifest_git_blob_sha != expected_scenario_manifest_git_blob_sha {
        issues.push(OperationContractIssue::SourceManifestGitBlobMismatch {
            expected: expected_scenario_manifest_git_blob_sha.to_string(),
            actual: document.source_scenario_manifest_git_blob_sha.clone(),
        });
    }
    if document.contracts.is_empty() {
        issues.push(OperationContractIssue::EmptyContractRegistry);
    }

    let mut keys = BTreeSet::new();
    let mut validated = BTreeMap::new();
    for contract in document.contracts {
        let key = (contract.contract_id.clone(), contract.contract_version.clone());
        if !keys.insert(key.clone()) {
            issues.push(OperationContractIssue::DuplicateContractId(contract.contract_id.clone()));
            continue;
        }

        match scenario_visibility(&contract.scenario_id) {
            Ok(required_cross_host)
                if required_cross_host && !contract.evidence_policy.requires_cross_host_visibility =>
            {
                issues.push(OperationContractIssue::ScenarioVisibilityWeakened {
                    scenario_id: contract.scenario_id.clone(),
                });
                continue;
            }
            Ok(_) => {}
            Err(issue) => {
                issues.push(issue);
                continue;
            }
        }

        if let Err(mut contract_issues) = validate_contract(&contract) {
            issues.append(&mut contract_issues);
            continue;
        }

        match operation_contract_digest(&contract) {
            Ok(digest) => {
                validated.insert(
                    key,
                    ValidatedOperationContract {
                        document: contract,
                        digest_sha256: digest,
                    },
                );
            }
            Err(issue) => issues.push(issue),
        }
    }

    if !issues.is_empty() {
        return Err(issues);
    }

    Ok(ValidatedOperationContractRegistry {
        registry_id: document.registry_id,
        registry_version: document.registry_version,
        registry_digest_sha256: expected_registry_digest_sha256.to_string(),
        source_scenario_manifest_git_blob_sha: document.source_scenario_manifest_git_blob_sha,
        contracts: validated,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const EMBEDDED_OPERATION_CONTRACTS: &str =
        include_str!("../../../../docs/benchmarks/MCRB_OPERATION_CONTRACTS_V0_1.json");
    const EXPECTED_SCENARIO_MANIFEST_GIT_BLOB_SHA: &str =
        "e3fb66b94622fd7e4a1f9367ca45ba413a30722c";

    fn embedded_registry() -> ValidatedOperationContractRegistry {
        let raw = EMBEDDED_OPERATION_CONTRACTS.as_bytes();
        let digest = sha256_hex(raw);
        validate_operation_contract_registry(
            raw,
            &digest,
            EXPECTED_SCENARIO_MANIFEST_GIT_BLOB_SHA,
        )
        .expect("embedded operation-contract registry should be structurally valid")
    }

    #[test]
    fn validates_embedded_registry_and_calculates_content_bound_contract_digests() {
        let registry = embedded_registry();
        assert_eq!(registry.registry_id(), "mcrb-operation-contracts");
        assert_eq!(registry.contract_count(), 4);

        let remote = registry
            .find("MCRB-OP-REMOTE-DHT-PUBLICATION", "0.1.0")
            .unwrap();
        assert_eq!(remote.scenario_id(), "MCRB-03");
        assert!(remote.requires_cross_host_visibility());
        assert!(remote.requires_independent_observer());
        assert_eq!(remote.expected_qops_outcome(), OperationOutcomeClass::QualifiedSuccess);
        assert_eq!(remote.digest_sha256().len(), 64);
    }

    #[test]
    fn revocation_denial_is_not_mislabeled_as_an_applied_effect_success() {
        let registry = embedded_registry();
        let revocation = registry
            .find("MCRB-OP-CAPABILITY-REVOCATION", "0.1.0")
            .unwrap();
        assert_eq!(revocation.expected_qops_outcome(), OperationOutcomeClass::CorrectDenial);
        assert!(revocation.expects_effect_state(EffectState::ConfirmedNotApplied));
        assert!(!revocation.expects_effect_state(EffectState::Applied));
    }

    #[test]
    fn rejects_registry_digest_mismatch() {
        let raw = EMBEDDED_OPERATION_CONTRACTS.as_bytes();
        let error = validate_operation_contract_registry(
            raw,
            &"0".repeat(64),
            EXPECTED_SCENARIO_MANIFEST_GIT_BLOB_SHA,
        )
        .unwrap_err();
        assert!(matches!(error[0], OperationContractIssue::RegistryDigestMismatch { .. }));
    }

    #[test]
    fn rejects_wrong_scenario_manifest_git_blob() {
        let raw = EMBEDDED_OPERATION_CONTRACTS.as_bytes();
        let digest = sha256_hex(raw);
        let error = validate_operation_contract_registry(raw, &digest, &"a".repeat(40))
            .unwrap_err();
        assert!(matches!(error[0], OperationContractIssue::SourceManifestGitBlobMismatch { .. }));
    }

    fn mutate_registry(mutator: impl FnOnce(&mut Value)) -> Vec<u8> {
        let mut value: Value = serde_json::from_str(EMBEDDED_OPERATION_CONTRACTS).unwrap();
        mutator(&mut value);
        serde_json::to_vec(&value).unwrap()
    }

    fn validate_mutated(raw: &[u8]) -> Result<ValidatedOperationContractRegistry, Vec<OperationContractIssue>> {
        let digest = sha256_hex(raw);
        validate_operation_contract_registry(
            raw,
            &digest,
            EXPECTED_SCENARIO_MANIFEST_GIT_BLOB_SHA,
        )
    }

    #[test]
    fn rejects_a_registry_that_weakens_scenario_cross_host_visibility() {
        let raw = mutate_registry(|value| {
            let contracts = value["contracts"].as_array_mut().unwrap();
            let contract = contracts.iter_mut().find(|item| {
                item["contract_id"] == "MCRB-OP-REMOTE-DHT-PUBLICATION"
            }).unwrap();
            contract["evidence_policy"]["requires_cross_host_visibility"] = Value::Bool(false);
        });
        let issues = validate_mutated(&raw).unwrap_err();
        assert!(issues.iter().any(|issue| matches!(
            issue,
            OperationContractIssue::ScenarioVisibilityWeakened { scenario_id }
                if scenario_id == "MCRB-03"
        )));
    }

    #[test]
    fn rejects_relabeling_expected_revocation_denial_as_qualified_success() {
        let raw = mutate_registry(|value| {
            let contracts = value["contracts"].as_array_mut().unwrap();
            let contract = contracts.iter_mut().find(|item| {
                item["contract_id"] == "MCRB-OP-CAPABILITY-REVOCATION"
            }).unwrap();
            contract["expected_qops_outcome"] = Value::String("QualifiedSuccess".to_string());
        });
        let issues = validate_mutated(&raw).unwrap_err();
        assert!(issues.iter().any(|issue| matches!(
            issue,
            OperationContractIssue::DispositionOutcomeMismatch { contract_id }
                if contract_id == "MCRB-OP-CAPABILITY-REVOCATION"
        )));
        assert!(issues.iter().any(|issue| matches!(
            issue,
            OperationContractIssue::QualifiedSuccessRequiresAppliedEffect { contract_id }
                if contract_id == "MCRB-OP-CAPABILITY-REVOCATION"
        )));
    }

    #[test]
    fn rejects_fail_open_unknown_policy() {
        let raw = mutate_registry(|value| {
            let contracts = value["contracts"].as_array_mut().unwrap();
            let contract = contracts.iter_mut().find(|item| {
                item["contract_id"] == "MCRB-OP-REMOTE-DHT-PUBLICATION"
            }).unwrap();
            contract["unknown_policy"]["blocks_qops"] = Value::Bool(false);
        });
        let issues = validate_mutated(&raw).unwrap_err();
        assert!(issues.iter().any(|issue| matches!(
            issue,
            OperationContractIssue::UnknownPolicyMustFailClosed { contract_id }
                if contract_id == "MCRB-OP-REMOTE-DHT-PUBLICATION"
        )));
    }

    #[test]
    fn rejects_duplicate_contract_id_and_version() {
        let raw = mutate_registry(|value| {
            let contracts = value["contracts"].as_array_mut().unwrap();
            let duplicate = contracts[0].clone();
            contracts.push(duplicate);
        });
        let issues = validate_mutated(&raw).unwrap_err();
        assert!(issues.iter().any(|issue| matches!(
            issue,
            OperationContractIssue::DuplicateContractId(contract_id)
                if contract_id == "MCRB-OP-LOCAL-APPEND-READ"
        )));
    }
}
