//! Mandatory evidence requalification before optional decision-value experiments.
//!
//! This crate consumes canonical impact classifications. It does not implement
//! a second change-impact classifier or an experiment-design optimizer.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

pub const CASE_SCHEMA: &str = "AeroChangeToTestCaseV1";
pub const REQUIRED_INVARIANTS: &[&str] = &[
    "MANDATORY_SET_IMMUTABLE",
    "UNKNOWN_NEVER_UNAFFECTED",
    "REASON_PATH_COMPLETE",
    "DETERMINISTIC_REPLAY",
    "OPTIONAL_TESTS_ADMISSIBILITY_GATED",
];

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArtifactRef {
    pub id: String,
    pub kind: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ImpactClass {
    Unaffected,
    ConditionalReview,
    Invalidated,
    RequiresReview,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum DependencyCoverage {
    Complete,
    Incomplete,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ObligationClass {
    Mandatory,
    ConditionalReview,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChangeSetManifest {
    pub change_id: String,
    pub sha256: String,
    pub changed_artifacts: Vec<ArtifactRef>,
    pub changed_inputs: Vec<ArtifactRef>,
    pub changed_requirements: Vec<ArtifactRef>,
    pub changed_methods: Vec<ArtifactRef>,
    pub changed_toolchains: Vec<ArtifactRef>,
}

#[derive(Serialize)]
struct CanonicalChangeSetPayload {
    schema: &'static str,
    change_id: String,
    changed_artifacts: Vec<ArtifactRef>,
    changed_inputs: Vec<ArtifactRef>,
    changed_requirements: Vec<ArtifactRef>,
    changed_methods: Vec<ArtifactRef>,
    changed_toolchains: Vec<ArtifactRef>,
}

/// Canonical SHA-256 over the typed ChangeSet body, excluding its own digest.
/// Reference arrays are sorted so equivalent permutations have one identity.
pub fn compute_change_set_digest(change: &ChangeSetManifest) -> String {
    let mut changed_artifacts = change.changed_artifacts.clone();
    let mut changed_inputs = change.changed_inputs.clone();
    let mut changed_requirements = change.changed_requirements.clone();
    let mut changed_methods = change.changed_methods.clone();
    let mut changed_toolchains = change.changed_toolchains.clone();

    for references in [
        &mut changed_artifacts,
        &mut changed_inputs,
        &mut changed_requirements,
        &mut changed_methods,
        &mut changed_toolchains,
    ] {
        references.sort();
    }

    let payload = CanonicalChangeSetPayload {
        schema: "AeroChangeSetDigestV1",
        change_id: change.change_id.clone(),
        changed_artifacts,
        changed_inputs,
        changed_requirements,
        changed_methods,
        changed_toolchains,
    };
    let bytes = serde_json::to_vec(&payload).expect("typed ChangeSet digest payload serializes");
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceImpactInput {
    pub evidence_id: String,
    pub configuration: ArtifactRef,
    pub subject: ArtifactRef,
    pub dependencies: Vec<ArtifactRef>,
    pub expected_impact: ImpactClass,
    pub dependency_coverage: DependencyCoverage,
    pub reason_path: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ObligationState {
    Open,
    Closed,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObligationInput {
    pub obligation_id: String,
    pub evidence_id: String,
    pub class: ObligationClass,
    pub mandatory: bool,
    pub reason_path: Vec<String>,
    pub demotable: bool,
    pub state: ObligationState,
    pub closure_receipt: Option<ArtifactRef>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum GateStatus {
    Admitted,
    Rejected,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AdmissibilityGates {
    pub authority: GateStatus,
    pub method: GateStatus,
    pub envelope: GateStatus,
    pub independent_reference: GateStatus,
    pub rights: GateStatus,
}

impl AdmissibilityGates {
    fn statuses(&self) -> [GateStatus; 5] {
        [
            self.authority,
            self.method,
            self.envelope,
            self.independent_reference,
            self.rights,
        ]
    }

    fn evaluate(&self) -> CandidateDisposition {
        let statuses = self.statuses();
        if statuses.contains(&GateStatus::Rejected) {
            CandidateDisposition::Rejected
        } else if statuses.contains(&GateStatus::Unknown) {
            CandidateDisposition::Indeterminate
        } else {
            CandidateDisposition::AdmissibleUncalibrated
        }
    }

    fn all_admitted(&self) -> bool {
        self.statuses().iter().all(|status| *status == GateStatus::Admitted)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DecisionValueEstimate {
    pub lower: f64,
    pub upper: f64,
    pub estimator_id: String,
    pub calibrated: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OptionalCandidate {
    pub candidate_id: String,
    pub target_decision: String,
    pub admissibility: AdmissibilityGates,
    pub cost: f64,
    pub decision_value: DecisionValueEstimate,
    pub mandatory_substitute: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ExpectedInvariant {
    MandatorySetImmutable,
    UnknownNeverUnaffected,
    ReasonPathComplete,
    DeterministicReplay,
    OptionalTestsAdmissibilityGated,
    OptionalTestsWaitForMandatoryClosure,
}

impl ExpectedInvariant {
    fn as_str(&self) -> &'static str {
        match self {
            Self::MandatorySetImmutable => "MANDATORY_SET_IMMUTABLE",
            Self::UnknownNeverUnaffected => "UNKNOWN_NEVER_UNAFFECTED",
            Self::ReasonPathComplete => "REASON_PATH_COMPLETE",
            Self::DeterministicReplay => "DETERMINISTIC_REPLAY",
            Self::OptionalTestsAdmissibilityGated => "OPTIONAL_TESTS_ADMISSIBILITY_GATED",
            Self::OptionalTestsWaitForMandatoryClosure => "OPTIONAL_TESTS_WAIT_FOR_MANDATORY_CLOSURE",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChangeToTestCaseV1 {
    pub schema: String,
    pub benchmark_id: String,
    pub case_id: String,
    pub predecessor_configuration: ArtifactRef,
    pub proposed_configuration: ArtifactRef,
    pub change_set: ChangeSetManifest,
    pub evidence: Vec<EvidenceImpactInput>,
    pub obligations: Vec<ObligationInput>,
    pub optional_candidates: Vec<OptionalCandidate>,
    pub expected_invariants: Vec<ExpectedInvariant>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CandidateDisposition {
    Ranked,
    AdmissibleUncalibrated,
    BlockedByMandatory,
    Rejected,
    Indeterminate,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CandidateEvaluation {
    pub candidate_id: String,
    pub disposition: CandidateDisposition,
    pub score_lower_bound_per_cost: Option<f64>,
    pub reason: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RequalificationPlan {
    pub case_id: String,
    pub mandatory_obligations: Vec<ObligationInput>,
    pub conditional_obligations: Vec<ObligationInput>,
    pub unknown_obligations: Vec<ObligationInput>,
    pub mandatory_closure_complete: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ChangeToTestReport {
    pub benchmark_id: String,
    pub plan: RequalificationPlan,
    pub optional_candidates: Vec<CandidateEvaluation>,
    /// Ordered candidate IDs only; mandatory obligations are in a separate,
    /// immutable part of this report.
    pub ranked_optional_candidate_ids: Vec<String>,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum QualificationError {
    #[error("wrong case schema")]
    WrongSchema,
    #[error("invalid identity: {0}")]
    InvalidIdentity(String),
    #[error("invalid or malformed SHA-256 for ChangeSet {0}")]
    InvalidChangeDigest(String),
    #[error("ChangeSet SHA-256 does not match its canonical payload: {0}")]
    ChangeDigestMismatch(String),
    #[error("duplicate evidence identity: {0}")]
    DuplicateEvidence(String),
    #[error("duplicate obligation identity: {0}")]
    DuplicateObligation(String),
    #[error("duplicate obligation for evidence: {0}")]
    DuplicateEvidenceObligation(String),
    #[error("duplicate candidate identity: {0}")]
    DuplicateCandidate(String),
    #[error("evidence impact has no reason path: {0}")]
    MissingReasonPath(String),
    #[error("an unaffected disposition lacks complete dependency coverage: {0}")]
    UnknownCannotBeUnaffected(String),
    #[error("declared obligations do not match the canonical impact set: {0}")]
    ObligationSetMismatch(String),
    #[error("mandatory obligation is demotable or not marked mandatory: {0}")]
    MandatoryObligationDemoted(String),
    #[error("closure receipt is missing, malformed, or not bound to a closed obligation: {0}")]
    InvalidClosureReceipt(String),
    #[error("unknown impact cannot be closed without a new canonical impact assessment: {0}")]
    UnknownImpactCannotBeClosed(String),
    #[error("non-mandatory obligation has mandatory flag set: {0}")]
    InvalidMandatoryFlag(String),
    #[error("optional candidate cannot substitute for a mandatory obligation: {0}")]
    MandatorySubstitution(String),
    #[error("invalid cost for optional candidate: {0}")]
    InvalidCandidateCost(String),
    #[error("invalid decision-value interval for optional candidate: {0}")]
    InvalidDecisionValue(String),
    #[error("missing expected invariant: {0}")]
    MissingInvariant(String),
    #[error("invalid case JSON: {0}")]
    InvalidCaseJson(String),
    #[error("unknown field in {scope}: {field}")]
    UnknownField { scope: String, field: String },
    #[error("missing field in {scope}: {field}")]
    MissingField { scope: String, field: String },
    #[error("malformed JSON object/array at {0}")]
    MalformedJson(String),
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

fn valid_ref(value: &ArtifactRef) -> bool {
    !value.id.trim().is_empty() && !value.kind.trim().is_empty()
}

fn check_object(
    value: &serde_json::Value,
    allowed: &[&str],
    required: &[&str],
    scope: &str,
) -> Result<(), QualificationError> {
    let object = value
        .as_object()
        .ok_or_else(|| QualificationError::MalformedJson(scope.to_owned()))?;
    for key in object.keys() {
        if !allowed.contains(&key.as_str()) {
            return Err(QualificationError::UnknownField {
                scope: scope.to_owned(),
                field: key.clone(),
            });
        }
    }
    for key in required {
        if !object.contains_key(*key) {
            return Err(QualificationError::MissingField {
                scope: scope.to_owned(),
                field: (*key).to_owned(),
            });
        }
    }
    Ok(())
}

fn check_array_of_objects<'a>(
    value: &'a serde_json::Value,
    allowed: &[&str],
    required: &[&str],
    scope: &str,
) -> Result<&'a Vec<serde_json::Value>, QualificationError> {
    let array = value
        .as_array()
        .ok_or_else(|| QualificationError::MalformedJson(scope.to_owned()))?;
    for (index, item) in array.iter().enumerate() {
        check_object(item, allowed, required, &format!("{scope}[{index}]"))?;
    }
    Ok(array)
}

/// Strictly parse the external JSON contract before typed deserialization.
pub fn parse_case_json(bytes: &[u8]) -> Result<ChangeToTestCaseV1, QualificationError> {
    let value: serde_json::Value = serde_json::from_slice(bytes)
        .map_err(|error| QualificationError::InvalidCaseJson(error.to_string()))?;

    const ROOT: &[&str] = &[
        "schema",
        "benchmark_id",
        "case_id",
        "predecessor_configuration",
        "proposed_configuration",
        "change_set",
        "evidence",
        "obligations",
        "optional_candidates",
        "expected_invariants",
    ];
    check_object(&value, ROOT, ROOT, "case")?;

    for field in ["predecessor_configuration", "proposed_configuration"] {
        let reference = value
            .get(field)
            .ok_or_else(|| QualificationError::MissingField {
                scope: "case".into(),
                field: field.into(),
            })?;
        check_object(reference, &["id", "kind"], &["id", "kind"], field)?;
    }

    const CHANGE_FIELDS: &[&str] = &[
        "change_id",
        "sha256",
        "changed_artifacts",
        "changed_inputs",
        "changed_requirements",
        "changed_methods",
        "changed_toolchains",
    ];
    let change = value
        .get("change_set")
        .ok_or_else(|| QualificationError::MissingField {
            scope: "case".into(),
            field: "change_set".into(),
        })?;
    check_object(change, CHANGE_FIELDS, CHANGE_FIELDS, "change_set")?;
    for field in [
        "changed_artifacts",
        "changed_inputs",
        "changed_requirements",
        "changed_methods",
        "changed_toolchains",
    ] {
        let refs = change
            .get(field)
            .and_then(serde_json::Value::as_array)
            .ok_or_else(|| QualificationError::MalformedJson(format!("change_set.{field}")))?;
        for (index, reference) in refs.iter().enumerate() {
            check_object(
                reference,
                &["id", "kind"],
                &["id", "kind"],
                &format!("change_set.{field}[{index}]"),
            )?;
        }
    }

    const EVIDENCE_FIELDS: &[&str] = &[
        "evidence_id",
        "configuration",
        "subject",
        "dependencies",
        "expected_impact",
        "reason_path",
        "dependency_coverage",
    ];
    let evidence = check_array_of_objects(
        value.get("evidence").ok_or_else(|| QualificationError::MissingField {
            scope: "case".into(),
            field: "evidence".into(),
        })?,
        EVIDENCE_FIELDS,
        EVIDENCE_FIELDS,
        "evidence",
    )?;
    for (index, item) in evidence.iter().enumerate() {
        for field in ["configuration", "subject"] {
            check_object(
                item.get(field).ok_or_else(|| QualificationError::MissingField {
                    scope: format!("evidence[{index}]"),
                    field: field.into(),
                })?,
                &["id", "kind"],
                &["id", "kind"],
                &format!("evidence[{index}].{field}"),
            )?;
        }
        let dependencies = item
            .get("dependencies")
            .and_then(serde_json::Value::as_array)
            .ok_or_else(|| QualificationError::MalformedJson(format!("evidence[{index}].dependencies")))?;
        for (dep_index, reference) in dependencies.iter().enumerate() {
            check_object(
                reference,
                &["id", "kind"],
                &["id", "kind"],
                &format!("evidence[{index}].dependencies[{dep_index}]"),
            )?;
        }
    }

    const OBLIGATION_FIELDS: &[&str] = &[
        "obligation_id",
        "evidence_id",
        "class",
        "mandatory",
        "reason_path",
        "demotable",
        "state",
        "closure_receipt",
    ];
    check_array_of_objects(
        value.get("obligations").ok_or_else(|| QualificationError::MissingField {
            scope: "case".into(),
            field: "obligations".into(),
        })?,
        OBLIGATION_FIELDS,
        OBLIGATION_FIELDS,
        "obligations",
    )?;

    const CANDIDATE_FIELDS: &[&str] = &[
        "candidate_id",
        "target_decision",
        "admissibility",
        "cost",
        "decision_value",
        "mandatory_substitute",
    ];
    let candidates = check_array_of_objects(
        value.get("optional_candidates").ok_or_else(|| QualificationError::MissingField {
            scope: "case".into(),
            field: "optional_candidates".into(),
        })?,
        CANDIDATE_FIELDS,
        CANDIDATE_FIELDS,
        "optional_candidates",
    )?;
    for (index, candidate) in candidates.iter().enumerate() {
        let gates = candidate.get("admissibility").ok_or_else(|| QualificationError::MissingField {
            scope: format!("optional_candidates[{index}]"),
            field: "admissibility".into(),
        })?;
        const GATE_FIELDS: &[&str] = &["authority", "method", "envelope", "independent_reference", "rights"];
        check_object(gates, GATE_FIELDS, GATE_FIELDS, &format!("optional_candidates[{index}].admissibility"))?;
        let estimate = candidate.get("decision_value").ok_or_else(|| QualificationError::MissingField {
            scope: format!("optional_candidates[{index}]"),
            field: "decision_value".into(),
        })?;
        const ESTIMATE_FIELDS: &[&str] = &["lower", "upper", "estimator_id", "calibrated"];
        check_object(estimate, ESTIMATE_FIELDS, ESTIMATE_FIELDS, &format!("optional_candidates[{index}].decision_value"))?;
    }

    serde_json::from_value(value)
        .map_err(|error| QualificationError::InvalidCaseJson(error.to_string()))
}

fn expected_obligation(
    evidence: &EvidenceImpactInput,
) -> Option<(ObligationClass, bool)> {
    if evidence.dependency_coverage != DependencyCoverage::Complete {
        return Some((ObligationClass::Unknown, false));
    }
    match evidence.expected_impact {
        ImpactClass::Unaffected => None,
        ImpactClass::Invalidated => Some((ObligationClass::Mandatory, true)),
        ImpactClass::ConditionalReview | ImpactClass::RequiresReview => {
            Some((ObligationClass::ConditionalReview, false))
        }
        ImpactClass::Unknown => Some((ObligationClass::Unknown, false)),
    }
}

fn validate_case(case: &ChangeToTestCaseV1) -> Result<(), QualificationError> {
    if case.schema != CASE_SCHEMA {
        return Err(QualificationError::WrongSchema);
    }
    for (value, label) in [
        (case.benchmark_id.as_str(), "benchmark_id"),
        (case.case_id.as_str(), "case_id"),
        (case.predecessor_configuration.id.as_str(), "predecessor_configuration.id"),
        (case.proposed_configuration.id.as_str(), "proposed_configuration.id"),
        (case.change_set.change_id.as_str(), "change_set.change_id"),
    ] {
        if value.trim().is_empty() {
            return Err(QualificationError::InvalidIdentity(label.to_owned()));
        }
    }
    if !valid_ref(&case.predecessor_configuration)
        || !valid_ref(&case.proposed_configuration)
        || case.predecessor_configuration.kind != "configuration"
        || case.proposed_configuration.kind != "configuration"
    {
        return Err(QualificationError::InvalidIdentity("configuration".into()));
    }
    if !is_sha256(&case.change_set.sha256) {
        return Err(QualificationError::InvalidChangeDigest(case.change_set.change_id.clone()));
    }
    if compute_change_set_digest(&case.change_set) != case.change_set.sha256 {
        return Err(QualificationError::ChangeDigestMismatch(case.change_set.change_id.clone()));
    }
    for reference in case
        .change_set
        .changed_artifacts
        .iter()
        .chain(&case.change_set.changed_inputs)
        .chain(&case.change_set.changed_requirements)
        .chain(&case.change_set.changed_methods)
        .chain(&case.change_set.changed_toolchains)
    {
        if !valid_ref(reference) {
            return Err(QualificationError::InvalidIdentity(reference.id.clone()));
        }
    }

    let mut evidence_ids = BTreeSet::new();
    for evidence in &case.evidence {
        if evidence.evidence_id.trim().is_empty()
            || !valid_ref(&evidence.configuration)
            || !valid_ref(&evidence.subject)
            || evidence.dependencies.iter().any(|reference| !valid_ref(reference))
        {
            return Err(QualificationError::InvalidIdentity(evidence.evidence_id.clone()));
        }
        if !evidence_ids.insert(evidence.evidence_id.clone()) {
            return Err(QualificationError::DuplicateEvidence(evidence.evidence_id.clone()));
        }
        if evidence.expected_impact != ImpactClass::Unaffected && evidence.reason_path.is_empty() {
            return Err(QualificationError::MissingReasonPath(evidence.evidence_id.clone()));
        }
        if evidence.dependency_coverage != DependencyCoverage::Complete
            && evidence.expected_impact == ImpactClass::Unaffected
        {
            return Err(QualificationError::UnknownCannotBeUnaffected(evidence.evidence_id.clone()));
        }
    }

    for invariant in REQUIRED_INVARIANTS {
        if !case.expected_invariants.iter().any(|item| item.as_str() == *invariant) {
            return Err(QualificationError::MissingInvariant((*invariant).to_owned()));
        }
    }

    let mut candidate_ids = BTreeSet::new();
    for candidate in &case.optional_candidates {
        if candidate.candidate_id.trim().is_empty()
            || candidate.target_decision.trim().is_empty()
            || candidate.decision_value.estimator_id.trim().is_empty()
        {
            return Err(QualificationError::InvalidIdentity(candidate.candidate_id.clone()));
        }
        if !candidate_ids.insert(candidate.candidate_id.clone()) {
            return Err(QualificationError::DuplicateCandidate(candidate.candidate_id.clone()));
        }
        if candidate.mandatory_substitute {
            return Err(QualificationError::MandatorySubstitution(candidate.candidate_id.clone()));
        }
        if !candidate.cost.is_finite() || candidate.cost <= 0.0 {
            return Err(QualificationError::InvalidCandidateCost(candidate.candidate_id.clone()));
        }
        let estimate = &candidate.decision_value;
        if !estimate.lower.is_finite()
            || !estimate.upper.is_finite()
            || estimate.lower > estimate.upper
        {
            return Err(QualificationError::InvalidDecisionValue(candidate.candidate_id.clone()));
        }
    }
    Ok(())
}

fn derive_plan(case: &ChangeToTestCaseV1) -> Result<RequalificationPlan, QualificationError> {
    let mut declared_by_evidence = BTreeMap::new();
    let mut obligation_ids = BTreeSet::new();
    for obligation in &case.obligations {
        if obligation.obligation_id.trim().is_empty() {
            return Err(QualificationError::InvalidIdentity("obligation_id".into()));
        }
        if !obligation_ids.insert(obligation.obligation_id.clone()) {
            return Err(QualificationError::DuplicateObligation(obligation.obligation_id.clone()));
        }
        if declared_by_evidence
            .insert(obligation.evidence_id.clone(), obligation)
            .is_some()
        {
            return Err(QualificationError::DuplicateEvidenceObligation(obligation.evidence_id.clone()));
        }
        if obligation.demotable {
            return Err(QualificationError::MandatoryObligationDemoted(obligation.obligation_id.clone()));
        }
        if obligation.class == ObligationClass::Mandatory && !obligation.mandatory {
            return Err(QualificationError::MandatoryObligationDemoted(obligation.obligation_id.clone()));
        }
        if obligation.class != ObligationClass::Mandatory && obligation.mandatory {
            return Err(QualificationError::InvalidMandatoryFlag(obligation.obligation_id.clone()));
        }
        if obligation.reason_path.is_empty() {
            return Err(QualificationError::MissingReasonPath(obligation.evidence_id.clone()));
        }
        match obligation.state {
            ObligationState::Open if obligation.closure_receipt.is_some() => {
                return Err(QualificationError::InvalidClosureReceipt(obligation.obligation_id.clone()));
            }
            ObligationState::Open => {}
            ObligationState::Closed => {
                let Some(receipt) = &obligation.closure_receipt else {
                    return Err(QualificationError::InvalidClosureReceipt(obligation.obligation_id.clone()));
                };
                if !valid_ref(receipt) || receipt.kind != "requalification_closure_receipt" {
                    return Err(QualificationError::InvalidClosureReceipt(obligation.obligation_id.clone()));
                }
                if obligation.class == ObligationClass::Unknown {
                    return Err(QualificationError::UnknownImpactCannotBeClosed(obligation.obligation_id.clone()));
                }
            }
        }
    }

    let mut mandatory = Vec::new();
    let mut conditional = Vec::new();
    let mut unknown = Vec::new();
    let mut expected_count = 0usize;

    for evidence in &case.evidence {
        let Some((class, mandatory_flag)) = expected_obligation(evidence) else {
            if declared_by_evidence.contains_key(&evidence.evidence_id) {
                return Err(QualificationError::ObligationSetMismatch(evidence.evidence_id.clone()));
            }
            continue;
        };
        expected_count += 1;
        let Some(declared) = declared_by_evidence.get(&evidence.evidence_id) else {
            return Err(QualificationError::ObligationSetMismatch(evidence.evidence_id.clone()));
        };
        if declared.class != class
            || declared.mandatory != mandatory_flag
            || declared.demotable
            || declared.reason_path != evidence.reason_path
        {
            return Err(QualificationError::ObligationSetMismatch(evidence.evidence_id.clone()));
        }
        match class {
            ObligationClass::Mandatory => mandatory.push((**declared).clone()),
            ObligationClass::ConditionalReview => conditional.push((**declared).clone()),
            ObligationClass::Unknown => unknown.push((**declared).clone()),
        }
    }
    if declared_by_evidence.len() != expected_count {
        return Err(QualificationError::ObligationSetMismatch("unexpected obligation".into()));
    }

    mandatory.sort_by(|a, b| a.obligation_id.cmp(&b.obligation_id));
    conditional.sort_by(|a, b| a.obligation_id.cmp(&b.obligation_id));
    unknown.sort_by(|a, b| a.obligation_id.cmp(&b.obligation_id));

    let mandatory_closure_complete = unknown.is_empty()
        && mandatory.iter().all(|item| item.state == ObligationState::Closed)
        && conditional.iter().all(|item| item.state == ObligationState::Closed);

    Ok(RequalificationPlan {
        case_id: case.case_id.clone(),
        mandatory_obligations: mandatory,
        conditional_obligations: conditional,
        unknown_obligations: unknown,
        mandatory_closure_complete,
    })
}

fn evaluate_candidate(
    candidate: &OptionalCandidate,
    mandatory_closure_complete: bool,
) -> CandidateEvaluation {
    if !mandatory_closure_complete {
        return CandidateEvaluation {
            candidate_id: candidate.candidate_id.clone(),
            disposition: CandidateDisposition::BlockedByMandatory,
            score_lower_bound_per_cost: None,
            reason: "mandatory/conditional/unknown requalification obligations remain open; optional experiments cannot be ranked".into(),
        };
    }
    let disposition = candidate.admissibility.evaluate();
    match disposition {
        CandidateDisposition::Rejected => CandidateEvaluation {
            candidate_id: candidate.candidate_id.clone(),
            disposition,
            score_lower_bound_per_cost: None,
            reason: "at least one mandatory admissibility gate rejected the candidate".into(),
        },
        CandidateDisposition::Indeterminate => CandidateEvaluation {
            candidate_id: candidate.candidate_id.clone(),
            disposition,
            score_lower_bound_per_cost: None,
            reason: "at least one admissibility gate is unknown; do not rank or select".into(),
        },
        _ if !candidate.admissibility.all_admitted() => CandidateEvaluation {
            candidate_id: candidate.candidate_id.clone(),
            disposition: CandidateDisposition::Indeterminate,
            score_lower_bound_per_cost: None,
            reason: "all admissibility gates must be explicitly admitted".into(),
        },
        _ if !candidate.decision_value.calibrated => CandidateEvaluation {
            candidate_id: candidate.candidate_id.clone(),
            disposition: CandidateDisposition::AdmissibleUncalibrated,
            score_lower_bound_per_cost: None,
            reason: "admissible, but decision-value estimate is uncalibrated and remains unranked".into(),
        },
        _ => CandidateEvaluation {
            candidate_id: candidate.candidate_id.clone(),
            disposition: CandidateDisposition::Ranked,
            score_lower_bound_per_cost: Some(candidate.decision_value.lower / candidate.cost),
            reason: "admissible and calibrated; ranked by conservative lower-bound value per total cost".into(),
        },
    }
}

/// Validate the canonical impact dispositions, preserve every mandatory
/// obligation, and only then rank eligible optional experiments.
pub fn evaluate_case(case: &ChangeToTestCaseV1) -> Result<ChangeToTestReport, QualificationError> {
    validate_case(case)?;
    let plan = derive_plan(case)?;
    let mandatory_closure_complete = plan.mandatory_closure_complete;

    let mut optional_candidates: Vec<_> = case.optional_candidates.iter()
        .map(|candidate| evaluate_candidate(candidate, mandatory_closure_complete))
        .collect();
    optional_candidates.sort_by(|a, b| a.candidate_id.cmp(&b.candidate_id));

    let mut ranked: Vec<_> = optional_candidates
        .iter()
        .filter(|candidate| candidate.disposition == CandidateDisposition::Ranked)
        .collect();
    ranked.sort_by(|a, b| {
        let left = a.score_lower_bound_per_cost.unwrap_or(f64::NEG_INFINITY);
        let right = b.score_lower_bound_per_cost.unwrap_or(f64::NEG_INFINITY);
        right.partial_cmp(&left).unwrap_or(Ordering::Equal)
            .then_with(|| a.candidate_id.cmp(&b.candidate_id))
    });

    let ranked_optional_candidate_ids = ranked
        .into_iter()
        .map(|candidate| candidate.candidate_id.clone())
        .collect();

    Ok(ChangeToTestReport {
        benchmark_id: case.benchmark_id.clone(),
        plan,
        optional_candidates,
        ranked_optional_candidate_ids,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> ChangeToTestCaseV1 {
        parse_case_json(include_bytes!("../../../fixtures/aero-change-to-test/change-to-test-v1.json"))
            .expect("fixture JSON")
    }

    fn close_resolvable_obligations(case: &mut ChangeToTestCaseV1) {
        for obligation in &mut case.obligations {
            if obligation.class != ObligationClass::Unknown {
                obligation.state = ObligationState::Closed;
                obligation.closure_receipt = Some(ArtifactRef {
                    id: format!("receipt:{}", obligation.obligation_id),
                    kind: "requalification_closure_receipt".into(),
                });
            }
        }
    }

    #[test]
    fn checked_in_case_derives_three_non_demotable_mandatory_obligations() {
        let report = evaluate_case(&fixture()).expect("valid case");
        assert_eq!(report.plan.mandatory_obligations.len(), 3);
        assert!(report.plan.conditional_obligations.is_empty());
        assert!(report.plan.unknown_obligations.is_empty());
        assert!(!report.plan.mandatory_closure_complete);
        assert!(report.plan.mandatory_obligations.iter().all(|item| item.mandatory && !item.demotable));
    }

    #[test]
    fn ineligible_optional_candidate_is_not_ranked() {
        let mut case = fixture();
        close_resolvable_obligations(&mut case);
        let report = evaluate_case(&case).expect("valid case");
        let candidate = report.optional_candidates.iter().find(|item| item.candidate_id == "candidate:repeat-nominal-measurement").unwrap();
        assert_eq!(candidate.disposition, CandidateDisposition::Indeterminate);
        assert!(candidate.score_lower_bound_per_cost.is_none());
        assert!(!report.ranked_optional_candidate_ids.contains(&candidate.candidate_id));
    }

    #[test]
    fn uncalibrated_candidate_is_visible_but_unranked() {
        let mut case = fixture();
        close_resolvable_obligations(&mut case);
        let report = evaluate_case(&case).expect("valid case");
        let candidate = report.optional_candidates.iter().find(|item| item.candidate_id == "candidate:nde-angle-study").unwrap();
        assert_eq!(candidate.disposition, CandidateDisposition::AdmissibleUncalibrated);
        assert!(candidate.score_lower_bound_per_cost.is_none());
        assert!(report.ranked_optional_candidate_ids.is_empty());
    }

    #[test]
    fn calibrated_optional_candidates_rank_by_lower_bound_per_cost() {
        let mut case = fixture();
        close_resolvable_obligations(&mut case);
        for candidate in &mut case.optional_candidates {
            candidate.admissibility.independent_reference = GateStatus::Admitted;
            candidate.decision_value.calibrated = true;
        }
        case.optional_candidates[0].decision_value.lower = 2.0;
        case.optional_candidates[0].decision_value.upper = 3.0;
        case.optional_candidates[0].cost = 5.0;
        case.optional_candidates[1].decision_value.lower = 1.0;
        case.optional_candidates[1].decision_value.upper = 2.0;
        case.optional_candidates[1].cost = 1.0;

        let report = evaluate_case(&case).expect("valid case");
        assert_eq!(
            report.ranked_optional_candidate_ids,
            vec![
                "candidate:repeat-nominal-measurement".to_owned(),
                "candidate:nde-angle-study".to_owned()
            ]
        );
    }

    #[test]
    fn optional_tests_are_blocked_until_all_mandatory_obligations_close() {
        let report = evaluate_case(&fixture()).expect("open requalification case");
        assert_eq!(report.ranked_optional_candidate_ids.len(), 0);
        assert!(report.optional_candidates.iter().all(|item| item.disposition == CandidateDisposition::BlockedByMandatory));
    }

    #[test]
    fn optional_candidate_cost_cannot_change_mandatory_set() {
        let mut case = fixture();
        let before = evaluate_case(&case).expect("baseline").plan.mandatory_obligations;
        for candidate in &mut case.optional_candidates {
            candidate.cost = 0.0001;
            candidate.decision_value.lower = 1_000_000.0;
            candidate.decision_value.upper = 2_000_000.0;
        }
        let after = evaluate_case(&case).expect("mutated optional values").plan.mandatory_obligations;
        assert_eq!(before, after);
    }

    #[test]
    fn mandatory_obligation_cannot_be_demoted() {
        let mut case = fixture();
        case.obligations[0].mandatory = false;
        assert!(matches!(evaluate_case(&case), Err(QualificationError::MandatoryObligationDemoted(_))));
    }

    #[test]
    fn mandatory_obligation_cannot_be_marked_demotable() {
        let mut case = fixture();
        case.obligations[0].demotable = true;
        assert!(matches!(evaluate_case(&case), Err(QualificationError::MandatoryObligationDemoted(_))));
    }

    #[test]
    fn incomplete_dependency_coverage_cannot_be_reported_unaffected() {
        let mut case = fixture();
        let evidence = case.evidence.iter_mut().find(|item| item.expected_impact == ImpactClass::Unaffected).unwrap();
        evidence.dependency_coverage = DependencyCoverage::Incomplete;
        assert!(matches!(evaluate_case(&case), Err(QualificationError::UnknownCannotBeUnaffected(_))));
    }

    #[test]
    fn unknown_impact_becomes_explicit_unknown_obligation() {
        let mut case = fixture();
        let evidence = case.evidence.iter_mut().find(|item| item.expected_impact == ImpactClass::Unaffected).unwrap();
        evidence.expected_impact = ImpactClass::Unknown;
        evidence.reason_path = vec!["change:process-plan-rev-002".into(), "unknown-dependency".into()];
        case.obligations.push(ObligationInput {
            obligation_id: "obligation:unknown-impact".into(),
            evidence_id: evidence.evidence_id.clone(),
            class: ObligationClass::Unknown,
            mandatory: false,
            reason_path: evidence.reason_path.clone(),
            demotable: false,
            state: ObligationState::Open,
            closure_receipt: None,
        });
        let report = evaluate_case(&case).expect("unknown must remain explicit");
        assert_eq!(report.plan.unknown_obligations.len(), 1);
        assert!(!report.plan.mandatory_closure_complete);
        assert!(report.optional_candidates.iter().all(|item| item.disposition == CandidateDisposition::BlockedByMandatory));
    }

    #[test]
    fn unknown_field_in_evidence_is_rejected() {
        let mut value: serde_json::Value = serde_json::from_slice(
            include_bytes!("../../../fixtures/aero-change-to-test/change-to-test-v1.json"),
        ).unwrap();
        value["evidence"][0]["hidden_release_authority"] = serde_json::json!(true);
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(matches!(parse_case_json(&bytes), Err(QualificationError::UnknownField { .. })));
    }

    #[test]
    fn missing_evidence_fields_are_rejected() {
        let mut value: serde_json::Value = serde_json::from_slice(
            include_bytes!("../../../fixtures/aero-change-to-test/change-to-test-v1.json"),
        ).unwrap();
        value["evidence"][0].as_object_mut().unwrap().remove("dependency_coverage");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(matches!(parse_case_json(&bytes), Err(QualificationError::MissingField { .. })));
    }

    #[test]
    fn input_order_does_not_change_plan_or_optional_ranking() {
        let mut case = fixture();
        let first = evaluate_case(&case).unwrap();
        case.evidence.reverse();
        case.obligations.reverse();
        case.optional_candidates.reverse();
        let second = evaluate_case(&case).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn unknown_impact_cannot_be_closed_by_a_generic_receipt() {
        let mut case = fixture();
        let evidence = case.evidence.iter_mut().find(|item| item.expected_impact == ImpactClass::Unaffected).unwrap();
        evidence.expected_impact = ImpactClass::Unknown;
        evidence.reason_path = vec!["change:process-plan-rev-002".into(), "unknown-dependency".into()];
        case.obligations.push(ObligationInput {
            obligation_id: "obligation:unknown-impact".into(),
            evidence_id: evidence.evidence_id.clone(),
            class: ObligationClass::Unknown,
            mandatory: false,
            reason_path: evidence.reason_path.clone(),
            demotable: false,
            state: ObligationState::Closed,
            closure_receipt: Some(ArtifactRef {
                id: "receipt:resolve-unknown".into(),
                kind: "requalification_closure_receipt".into(),
            }),
        });
        assert!(matches!(evaluate_case(&case), Err(QualificationError::UnknownImpactCannotBeClosed(_))));
    }

    #[test]
    fn change_set_digest_is_canonical_and_permutation_invariant() {
        let mut case = fixture();
        let expected = compute_change_set_digest(&case.change_set);
        case.change_set.sha256 = expected.clone();
        case.change_set.changed_artifacts.reverse();
        case.change_set.changed_inputs.reverse();
        case.change_set.changed_toolchains.reverse();
        assert_eq!(compute_change_set_digest(&case.change_set), expected);
    }

    #[test]
    fn change_set_digest_must_match_exact_canonical_payload() {
        let mut case = fixture();
        case.change_set.sha256 = compute_change_set_digest(&case.change_set);
        assert_eq!(evaluate_case(&case).unwrap().plan.mandatory_obligations.len(), 3);
        case.change_set.changed_inputs[0].id.push_str("-mutated");
        assert!(matches!(evaluate_case(&case), Err(QualificationError::ChangeDigestMismatch(_))));
    }

    #[test]
    fn mutation_of_change_digest_is_rejected() {
        let mut case = fixture();
        case.change_set.sha256 = "bad".into();
        assert!(matches!(evaluate_case(&case), Err(QualificationError::InvalidChangeDigest(_))));
    }
}
