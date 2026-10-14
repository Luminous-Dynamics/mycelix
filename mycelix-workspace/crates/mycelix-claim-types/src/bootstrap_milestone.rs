// Copyright (C) 2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Runtime-independent, fail-closed provenance contract for a space-bootstrap milestone.
//!
//! This module records claims and their evidence. It does not grant execution
//! authority, verify signatures, fetch evidence URIs, hash external artifacts, or
//! decide whether a physical capability is safe. Consumers must independently
//! verify every referenced digest and apply their authorization policy.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Schema version for BootstrapMilestone.
pub const BOOTSTRAP_MILESTONE_SCHEMA_VERSION: u32 = 1;

/// Industrial-closure stage claimed by a milestone.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClosureStage {
    /// Initial survival and observation capabilities.
    Seed,
    /// Local repair of existing equipment.
    Repair,
    /// Qualified production of useful feedstock.
    Feedstock,
    /// Production of selected structural products.
    Structural,
    /// Production and maintenance of machines.
    Machine,
    /// Closure of critical control and metrology dependencies.
    Control,
    /// Sustained factory operation.
    Factory,
    /// Support of a bounded local ecological loop.
    Ecological,
    /// Evidence-backed expansion beyond the initial industrial base.
    Expansion,
}

/// Evidence class describes what the referenced artifact is, not whether a
/// milestone is authorized or true.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceKind {
    /// Deterministic synthetic test input or fixture.
    SyntheticFixture,
    /// CI workflow execution receipt.
    WorkflowRun,
    /// Software test report or assertion transcript.
    TestReport,
    /// Model-produced output or prediction.
    ModelOutput,
    /// Simulation result, with its model assumptions retained.
    SimulationOutput,
    /// Remote or indirect resource estimate.
    RemoteEstimate,
    /// Observation collected at the target environment.
    InSituObservation,
    /// Calibrated mass measurement.
    MassMeasurement,
    /// Material composition or process assay.
    ResourceAssay,
    /// Reconciled inventory ledger evidence.
    InventoryReconciliation,
    /// Authenticated supply-chain or delivery receipt.
    SupplyChainReceipt,
    /// Measurement from an operating process.
    OperationalMeasurement,
    /// Result of an independent review.
    IndependentReview,
    /// Ground-test result under declared conditions.
    GroundTestReport,
    /// Result from a relevant-environment test.
    RelevantEnvironmentTestReport,
    /// Physical qualification evidence for a defined product and envelope.
    PhysicalQualificationReport,
    /// Inspection or product-acceptance result.
    InspectionResult,
    /// Calibration certificate and instrument traceability record.
    CalibrationCertificate,
    /// Requirement-to-verification mapping or matrix.
    RequirementsTraceabilityMatrix,
    /// Verification Control Document or equivalent evidence index.
    VerificationControlDocument,
    /// Hazard identification, risk assessment and control analysis.
    HazardAnalysis,
    /// Software assurance and software-safety assessment.
    SoftwareAssuranceReport,
    /// Independent software or system verification report.
    IndependentVerificationReport,
    /// Project-specific applicability/tailoring assessment for a standard.
    StandardsApplicabilityAssessment,
    /// Formal acceptance-test report for a stated configuration and envelope.
    AcceptanceTestReport,
    /// Policy or authorization decision reference (not an authorization grant).
    PolicyDecision,
}

/// Evidentiary class assigned to the milestone's overall claim.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FixtureClass {
    /// Synthetic software-contract tests only.
    SyntheticSoftwareContract,
    /// Output of a model or simulation; not a physical measurement.
    ModelDerived,
    /// Test performed in a ground/laboratory environment.
    GroundTest,
    /// Test in an environment relevant to the target operation.
    RelevantEnvironmentTest,
    /// Measurement from an operating system in a declared envelope.
    OperationalMeasured,
    /// Independently reviewed qualification for a defined envelope.
    PhysicalQualification,
}

/// Content-addressed reference to evidence stored outside this schema.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceReference {
    /// URI or repository-native locator for the artifact.
    pub uri: String,
    /// Lowercase SHA-256 digest of the referenced artifact bytes.
    pub sha256: String,
    /// Declared artifact type; not a truth or trust score.
    pub kind: EvidenceKind,
}

impl EvidenceReference {
    /// Validate the locator and digest syntax.
    pub fn validate(&self) -> Result<(), ValidationError> {
        validate_nonempty("evidence.uri", &self.uri)?;
        if self.uri.chars().any(char::is_whitespace) {
            return Err(ValidationError::new("evidence.uri must not contain whitespace"));
        }
        validate_sha256("evidence.sha256", &self.sha256)
    }
}

/// Classification of a dependency before and after the claimed transition.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DependencyDisposition {
    /// Capability or dependency is available from local production.
    LocallyClosed,
    /// Dependency is satisfied by a declared substitution.
    Substituted,
    /// Still relies on imported durable equipment or parts.
    ImportedDurable,
    /// Still relies on imported consumables.
    ImportedConsumable,
    /// Dependency is known but not yet satisfied.
    Blocked,
    /// Evidence is insufficient to classify the dependency.
    Unknown,
}

/// One dependency state transition. Identifiers are stable within the record.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DependencyDelta {
    /// Stable dependency identifier.
    pub dependency_id: String,
    /// Classification before the claimed milestone.
    pub prior: DependencyDisposition,
    /// Classification after the claimed milestone.
    pub resulting: DependencyDisposition,
}

/// Resource data are represented as distinct variants so a model estimate
/// cannot be mistaken for an accepted physical-inventory transaction.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "record_type", rename_all = "snake_case", deny_unknown_fields)]
pub enum ResourceStatement {
    /// Estimate or model result; never an inventory mutation.
    Estimate {
        /// Stable resource identifier.
        resource_id: String,
        /// Estimated quantity in milligrams.
        quantity_milligrams: u64,
        /// Source/model evidence.
        evidence: Vec<EvidenceReference>,
    },
    /// Observation made in the target environment; not yet accepted stock.
    Observation {
        /// Stable resource identifier.
        resource_id: String,
        /// Observed quantity in milligrams.
        quantity_milligrams: u64,
        /// Measurement evidence.
        evidence: Vec<EvidenceReference>,
    },
    /// Accepted physical-inventory change, tied to an identified batch.
    AcceptedInventoryDelta {
        /// Stable resource identifier.
        resource_id: String,
        /// Batch or delivery identifier.
        batch_id: String,
        /// Signed inventory change in milligrams: positive adds stock, negative
        /// removes stock. A change of zero is not an inventory transaction.
        delta_milligrams: i64,
        /// Must include a mass measurement, inventory reconciliation, or
        /// authenticated supply-chain receipt.
        evidence: Vec<EvidenceReference>,
    },
}

/// Outcome recorded for a fault/recovery scenario. NotRun is never equivalent
/// to passing or to "no failure observed".
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FailureRecoveryOutcome {
    /// Scenario was included in the plan but not executed.
    NotRun,
    /// Scenario executed and no failure was observed.
    NoFailureObserved,
    /// Unsafe or invalid operation was rejected without a state mutation.
    RejectedSafely,
    /// Failure was detected; recovery has not been demonstrated.
    Detected,
    /// Failure was recovered successfully within the declared envelope.
    Recovered,
    /// System continued in a constrained/degraded state.
    Degraded,
    /// System could not recover in the tested envelope.
    Unrecoverable,
}

/// Recorded outcome for one named failure scenario.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FailureRecoveryObservation {
    /// Frozen scenario identifier from the scenario manifest.
    pub scenario_id: String,
    /// Failure class, or "none" where no failure was injected/observed.
    pub failure_class: String,
    /// Explicit execution/recovery outcome.
    pub outcome: FailureRecoveryOutcome,
    /// Duration in milliseconds where measured and applicable.
    pub recovery_duration_milliseconds: Option<u64>,
    /// Supporting test, sensor, or recovery evidence.
    pub evidence: Vec<EvidenceReference>,
}

/// The policy context under which a claim was made. This is descriptive
/// provenance only; it is not an execution token, capability grant, or command.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuthorityBasis {
    /// Stable identifier for the governing policy.
    pub policy_id: String,
    /// SHA-256 commitment to the policy/version bytes.
    pub policy_sha256: String,
    /// Human-readable scope under which the claim was evaluated.
    pub scope: String,
    /// Optional reference to an authorization decision record.
    pub decision_evidence: Option<EvidenceReference>,
}

/// Whether the record is an original claim or an append-only correction/retraction.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MilestoneDisposition {
    /// A new milestone claim.
    Claim,
    /// A new record correcting a prior milestone; prior record remains intact.
    Correction,
    /// A new record retracting a prior milestone; prior record remains intact.
    Retraction,
}

/// Provenance record for an industrial-bootstrap milestone.
///
/// This structure is intentionally independent of Symtropy's simulation
/// internals and Holochain's storage/runtime APIs. A storage adapter must append
/// records immutably and resolve supersedes_milestone_id; structural
/// validation alone cannot enforce append-only storage or authenticate issuers.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BootstrapMilestone {
    /// Must equal BOOTSTRAP_MILESTONE_SCHEMA_VERSION.
    pub schema_version: u32,
    /// Stable identifier unique within the ledger/namespace.
    pub milestone_id: String,
    /// Original claim, correction, or retraction.
    pub disposition: MilestoneDisposition,
    /// ID of the previous record corrected or retracted, when applicable.
    pub supersedes_milestone_id: Option<String>,
    /// SHA-256 commitment to the prior bootstrap state.
    pub prior_state_sha256: String,
    /// SHA-256 commitment to the resulting bootstrap state.
    pub resulting_state_sha256: String,
    /// Capability asserted by this record; this remains a claim pending review.
    pub claimed_capability: String,
    /// Stage in the industrial-closure ladder.
    pub closure_stage: ClosureStage,
    /// Source repository or system identifier.
    pub source_repository: String,
    /// Lowercase Git commit SHA-1 (40 hex characters) or SHA-256 (64 hex characters).
    pub source_commit_sha: String,
    /// Digest of the exact input/scenario manifest used for this claim.
    pub scenario_manifest_sha256: String,
    /// Evidence class for the result; used to prevent synthetic/estimated results
    /// from silently being described as measured or physically qualified.
    pub fixture_class: FixtureClass,
    /// Immutable references supporting the claim and its declared class.
    pub evidence: Vec<EvidenceReference>,
    /// Dependency closures or blockers changed by the claimed transition.
    pub dependency_deltas: Vec<DependencyDelta>,
    /// Net imported-mass change in milligrams. Positive means more imported mass
    /// is required; negative means the requirement was reduced.
    pub imported_mass_delta_milligrams: i64,
    /// Net energy delta in millijoules against the declared baseline. Positive
    /// means additional energy consumed/required; negative means reduction or
    /// recovered energy relative to that baseline.
    pub energy_delta_millijoules: i64,
    /// Estimates, observations and accepted inventory changes remain typed apart.
    pub resource_statements: Vec<ResourceStatement>,
    /// Includes failed, degraded, recovered and NotRun scenarios explicitly.
    pub failure_recovery: Vec<FailureRecoveryObservation>,
    /// Describes the policy basis only; grants no actuation authority.
    pub authority_basis: AuthorityBasis,
}

impl BootstrapMilestone {
    /// Validate the record's internal invariants without fetching evidence,
    /// checking signatures, or granting authority.
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.schema_version != BOOTSTRAP_MILESTONE_SCHEMA_VERSION {
            return Err(ValidationError::new("unsupported bootstrap milestone schema version"));
        }
        validate_nonempty("milestone_id", &self.milestone_id)?;
        validate_sha256("prior_state_sha256", &self.prior_state_sha256)?;
        validate_sha256("resulting_state_sha256", &self.resulting_state_sha256)?;
        validate_nonempty("claimed_capability", &self.claimed_capability)?;
        validate_nonempty("source_repository", &self.source_repository)?;
        validate_git_commit("source_commit_sha", &self.source_commit_sha)?;
        validate_sha256("scenario_manifest_sha256", &self.scenario_manifest_sha256)?;

        match self.disposition {
            MilestoneDisposition::Claim if self.supersedes_milestone_id.is_some() => {
                return Err(ValidationError::new(
                    "a new claim must not supersede another milestone",
                ));
            }
            MilestoneDisposition::Correction | MilestoneDisposition::Retraction => {
                let supersedes = self.supersedes_milestone_id.as_deref().ok_or_else(|| {
                    ValidationError::new("correction/retraction must reference the prior milestone")
                })?;
                validate_nonempty("supersedes_milestone_id", supersedes)?;
                if supersedes == self.milestone_id {
                    return Err(ValidationError::new("a milestone cannot supersede itself"));
                }
            }
            MilestoneDisposition::Claim => {}
        }

        if self.evidence.is_empty() {
            return Err(ValidationError::new("at least one evidence reference is required"));
        }
        validate_evidence_list(&self.evidence)?;
        self.validate_fixture_evidence()?;

        let mut dependency_ids = BTreeSet::new();
        for delta in &self.dependency_deltas {
            validate_nonempty("dependency_id", &delta.dependency_id)?;
            if delta.prior == delta.resulting {
                return Err(ValidationError::new(
                    "dependency delta must change the prior disposition",
                ));
            }
            if !dependency_ids.insert(delta.dependency_id.as_str()) {
                return Err(ValidationError::new("duplicate dependency delta identifier"));
            }
        }

        let mut inventory_batches = BTreeSet::new();
        for statement in &self.resource_statements {
            match statement {
                ResourceStatement::Estimate {
                    resource_id,
                    evidence,
                    ..
                } => {
                    validate_nonempty("resource.resource_id", resource_id)?;
                    validate_evidence_list(evidence)?;
                    if !evidence.iter().any(|reference| {
                        matches!(
                            reference.kind,
                            EvidenceKind::RemoteEstimate
                                | EvidenceKind::ModelOutput
                                | EvidenceKind::SimulationOutput
                                | EvidenceKind::SyntheticFixture
                        )
                    }) {
                        return Err(ValidationError::new(
                            "resource estimate requires estimate/model/simulation/fixture evidence",
                        ));
                    }
                }
                ResourceStatement::Observation {
                    resource_id,
                    evidence,
                    ..
                } => {
                    validate_nonempty("resource.resource_id", resource_id)?;
                    validate_evidence_list(evidence)?;
                    if !evidence.iter().any(|reference| {
                        matches!(
                            reference.kind,
                            EvidenceKind::InSituObservation
                                | EvidenceKind::OperationalMeasurement
                                | EvidenceKind::ResourceAssay
                                | EvidenceKind::MassMeasurement
                        )
                    }) {
                        return Err(ValidationError::new(
                            "resource observation requires measurement evidence",
                        ));
                    }
                    if matches!(
                        self.fixture_class,
                        FixtureClass::SyntheticSoftwareContract | FixtureClass::ModelDerived
                    ) {
                        return Err(ValidationError::new(
                            "synthetic/model-derived claims cannot assert measured resource observations",
                        ));
                    }
                }
                ResourceStatement::AcceptedInventoryDelta {
                    resource_id,
                    batch_id,
                    delta_milligrams,
                    evidence,
                } => {
                    validate_nonempty("resource.resource_id", resource_id)?;
                    validate_nonempty("resource.batch_id", batch_id)?;
                    if *delta_milligrams == 0 {
                        return Err(ValidationError::new(
                            "accepted inventory delta must be non-zero",
                        ));
                    }
                    validate_evidence_list(evidence)?;
                    if !evidence.iter().any(|reference| {
                        matches!(
                            reference.kind,
                            EvidenceKind::MassMeasurement
                                | EvidenceKind::InventoryReconciliation
                                | EvidenceKind::SupplyChainReceipt
                        )
                    }) {
                        return Err(ValidationError::new(
                            "accepted inventory delta requires mass/reconciliation/receipt evidence",
                        ));
                    }
                    if matches!(
                        self.fixture_class,
                        FixtureClass::SyntheticSoftwareContract | FixtureClass::ModelDerived
                    ) {
                        return Err(ValidationError::new(
                            "synthetic/model-derived claims cannot mutate measured inventory",
                        ));
                    }
                    let key = (resource_id.as_str(), batch_id.as_str());
                    if !inventory_batches.insert(key) {
                        return Err(ValidationError::new(
                            "duplicate accepted inventory delta for the same resource batch",
                        ));
                    }
                }
            }
        }

        let mut scenario_ids = BTreeSet::new();
        for observation in &self.failure_recovery {
            validate_nonempty("failure.scenario_id", &observation.scenario_id)?;
            if !scenario_ids.insert(observation.scenario_id.as_str()) {
                return Err(ValidationError::new("duplicate failure/recovery scenario identifier"));
            }
            validate_nonempty("failure.failure_class", &observation.failure_class)?;
            validate_evidence_list_allow_empty(&observation.evidence)?;
            if observation.outcome == FailureRecoveryOutcome::Recovered
                && observation.recovery_duration_milliseconds.is_none()
            {
                return Err(ValidationError::new(
                    "a recovered failure requires a measured recovery duration",
                ));
            }
            if observation.outcome == FailureRecoveryOutcome::NotRun
                && observation.recovery_duration_milliseconds.is_some()
            {
                return Err(ValidationError::new(
                    "a NotRun scenario must not carry a recovery duration",
                ));
            }
        }

        validate_nonempty("authority_basis.policy_id", &self.authority_basis.policy_id)?;
        validate_sha256("authority_basis.policy_sha256", &self.authority_basis.policy_sha256)?;
        validate_nonempty("authority_basis.scope", &self.authority_basis.scope)?;
        if let Some(reference) = &self.authority_basis.decision_evidence {
            reference.validate()?;
            if reference.kind != EvidenceKind::PolicyDecision {
                return Err(ValidationError::new(
                    "authority decision evidence must be typed as a policy decision",
                ));
            }
        }

        Ok(())
    }

    fn validate_fixture_evidence(&self) -> Result<(), ValidationError> {
        let has = |kind| self.evidence.iter().any(|reference| reference.kind == kind);
        let valid = match self.fixture_class {
            FixtureClass::SyntheticSoftwareContract => {
                has(EvidenceKind::SyntheticFixture)
                    || has(EvidenceKind::WorkflowRun)
                    || has(EvidenceKind::TestReport)
            }
            FixtureClass::ModelDerived => {
                has(EvidenceKind::ModelOutput) || has(EvidenceKind::SimulationOutput)
            }
            FixtureClass::GroundTest => has(EvidenceKind::GroundTestReport),
            FixtureClass::RelevantEnvironmentTest => {
                has(EvidenceKind::RelevantEnvironmentTestReport)
            }
            FixtureClass::OperationalMeasured => has(EvidenceKind::OperationalMeasurement),
            FixtureClass::PhysicalQualification => self.evidence.iter().any(|qualification| {
                qualification.kind == EvidenceKind::PhysicalQualificationReport
                    && self.evidence.iter().any(|review| {
                        review.kind == EvidenceKind::IndependentReview
                            && (qualification.uri != review.uri
                                || qualification.sha256 != review.sha256)
                    })
            })
        };
        if !valid {
            return Err(ValidationError::new(
                "evidence kinds do not support the declared fixture class",
            ));
        }
        if self.fixture_class == FixtureClass::SyntheticSoftwareContract
            && has(EvidenceKind::PhysicalQualificationReport)
        {
            return Err(ValidationError::new(
                "synthetic software claims cannot be labelled as physical qualification",
            ));
        }
        Ok(())
    }
}

/// Structural validation error. Callers should not treat a valid schema as
/// proof that referenced artifacts, measurements, signatures, or authorities are valid.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ValidationError {
    message: String,
}

impl ValidationError {
    fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }

    /// Human-readable validation failure.
    pub fn message(&self) -> &str {
        &self.message
    }
}

impl std::fmt::Display for ValidationError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.message)
    }
}

impl std::error::Error for ValidationError {}

fn validate_nonempty(field: &str, value: &str) -> Result<(), ValidationError> {
    if value.trim().is_empty() {
        return Err(ValidationError::new(format!("{field} must not be empty")));
    }
    Ok(())
}

fn validate_sha256(field: &str, value: &str) -> Result<(), ValidationError> {
    if value.len() != 64
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(ValidationError::new(format!(
            "{field} must be a lowercase 64-character SHA-256 digest"
        )));
    }
    Ok(())
}

fn validate_git_commit(field: &str, value: &str) -> Result<(), ValidationError> {
    let valid_length = value.len() == 40 || value.len() == 64;
    let lowercase_hex = value
        .bytes()
        .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte));
    if !valid_length || !lowercase_hex {
        return Err(ValidationError::new(format!(
            "{field} must be a lowercase 40-character SHA-1 or 64-character SHA-256 commit ID"
        )));
    }
    Ok(())
}

fn validate_evidence_list(evidence: &[EvidenceReference]) -> Result<(), ValidationError> {
    if evidence.is_empty() {
        return Err(ValidationError::new(
            "at least one evidence reference is required for this statement",
        ));
    }
    validate_evidence_list_allow_empty(evidence)
}

fn validate_evidence_list_allow_empty(
    evidence: &[EvidenceReference],
) -> Result<(), ValidationError> {
    let mut identities = BTreeSet::new();
    for reference in evidence {
        reference.validate()?;
        if !identities.insert((reference.uri.as_str(), reference.sha256.as_str())) {
            return Err(ValidationError::new(
                "duplicate evidence bytes cannot be reclassified as separate evidence",
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn evidence(uri: &str, kind: EvidenceKind, digest_byte: char) -> EvidenceReference {
        EvidenceReference {
            uri: uri.to_string(),
            sha256: std::iter::repeat_n(digest_byte, 64).collect(),
            kind,
        }
    }

    fn valid_milestone() -> BootstrapMilestone {
        BootstrapMilestone {
            schema_version: BOOTSTRAP_MILESTONE_SCHEMA_VERSION,
            milestone_id: "milestone-0001".to_string(),
            disposition: MilestoneDisposition::Claim,
            supersedes_milestone_id: None,
            prior_state_sha256: "a".repeat(64),
            resulting_state_sha256: "b".repeat(64),
            claimed_capability: "synthetic_inventory_accounting_contract".to_string(),
            closure_stage: ClosureStage::Seed,
            source_repository: "https://example.invalid/source/symtropy".to_string(),
            source_commit_sha: "c".repeat(40),
            scenario_manifest_sha256: "d".repeat(64),
            fixture_class: FixtureClass::SyntheticSoftwareContract,
            evidence: vec![evidence(
                "https://example.invalid/evidence/reference-run.json",
                EvidenceKind::SyntheticFixture,
                'e',
            )],
            dependency_deltas: vec![DependencyDelta {
                dependency_id: "imported_metrology".to_string(),
                prior: DependencyDisposition::Unknown,
                resulting: DependencyDisposition::Blocked,
            }],
            imported_mass_delta_milligrams: -250_000,
            energy_delta_millijoules: 1_500_000,
            resource_statements: vec![ResourceStatement::Estimate {
                resource_id: "regolith-simulant".to_string(),
                quantity_milligrams: 100_000_000,
                evidence: vec![evidence(
                    "https://example.invalid/evidence/synthetic-feed.json",
                    EvidenceKind::SyntheticFixture,
                    'f',
                )],
            }],
            failure_recovery: vec![FailureRecoveryObservation {
                scenario_id: "LEDGER-DUPLICATE-EVENT".to_string(),
                failure_class: "duplicate_event_identity".to_string(),
                outcome: FailureRecoveryOutcome::RejectedSafely,
                recovery_duration_milliseconds: None,
                evidence: vec![evidence(
                    "https://example.invalid/evidence/fault-result.json",
                    EvidenceKind::TestReport,
                    '1',
                )],
            }],
            authority_basis: AuthorityBasis {
                policy_id: "bootstrap-planning-only".to_string(),
                policy_sha256: "2".repeat(64),
                scope: "claim-recording only; no execution authority".to_string(),
                decision_evidence: None,
            },
        }
    }

    #[test]
    fn accepts_a_well_formed_synthetic_claim_without_granting_authority() {
        let milestone = valid_milestone();
        assert!(milestone.validate().is_ok());
        assert!(milestone.authority_basis.scope.contains("no execution authority"));
    }

    #[test]
    fn rejects_wrong_schema_version_and_malformed_digests() {
        let mut milestone = valid_milestone();
        milestone.schema_version = BOOTSTRAP_MILESTONE_SCHEMA_VERSION + 1;
        assert!(milestone.validate().is_err());

        let mut milestone = valid_milestone();
        milestone.prior_state_sha256 = "ABC123".to_string();
        assert!(milestone.validate().is_err());

        let mut milestone = valid_milestone();
        milestone.source_commit_sha = "g".repeat(40);
        assert!(milestone.validate().is_err());
    }

    #[test]
    fn corrections_and_retractions_are_new_records_that_link_back() {
        let mut correction = valid_milestone();
        correction.disposition = MilestoneDisposition::Correction;
        assert!(correction.validate().is_err());

        correction.supersedes_milestone_id = Some("milestone-0000".to_string());
        assert!(correction.validate().is_ok());

        let mut self_superseding = correction.clone();
        self_superseding.supersedes_milestone_id = Some(self_superseding.milestone_id.clone());
        assert!(self_superseding.validate().is_err());
    }

    #[test]
    fn resource_estimates_are_not_inventory_mutations() {
        let milestone = valid_milestone();
        assert!(milestone.validate().is_ok());
        assert!(matches!(
            milestone.resource_statements.as_slice(),
            [ResourceStatement::Estimate { .. }]
        ));
    }

    #[test]
    fn synthetic_claim_cannot_relabel_a_mass_measurement_as_an_observation() {
        let mut milestone = valid_milestone();
        milestone.resource_statements = vec![ResourceStatement::Observation {
            resource_id: "regolith-simulant".to_string(),
            quantity_milligrams: 100_000_000,
            evidence: vec![evidence(
                "https://example.invalid/evidence/mass-measurement.json",
                EvidenceKind::MassMeasurement,
                '8',
            )],
        }];
        assert!(milestone.validate().is_err());
    }

    #[test]
    fn synthetic_fixture_cannot_promote_an_inventory_delta() {
        let mut milestone = valid_milestone();
        milestone.resource_statements.push(ResourceStatement::AcceptedInventoryDelta {
            resource_id: "metal-rich-fraction".to_string(),
            batch_id: "batch-001".to_string(),
            delta_milligrams: 50_000,
            evidence: vec![evidence(
                "https://example.invalid/evidence/mass-scale.json",
                EvidenceKind::MassMeasurement,
                '3',
            )],
        });
        assert!(milestone.validate().is_err());
    }

    #[test]
    fn measured_inventory_requires_measurement_evidence_and_nonzero_batch_delta() {
        let mut milestone = valid_milestone();
        milestone.fixture_class = FixtureClass::OperationalMeasured;
        milestone.evidence = vec![evidence(
            "https://example.invalid/evidence/operational-run.json",
            EvidenceKind::OperationalMeasurement,
            '4',
        )];
        milestone.resource_statements = vec![ResourceStatement::AcceptedInventoryDelta {
            resource_id: "accepted-feedstock".to_string(),
            batch_id: "batch-accepted-001".to_string(),
            delta_milligrams: 200_000,
            evidence: vec![evidence(
                "https://example.invalid/evidence/inventory-transaction.json",
                EvidenceKind::InventoryReconciliation,
                '5',
            )],
        }];
        assert!(milestone.validate().is_ok());

        if let ResourceStatement::AcceptedInventoryDelta { evidence, .. } =
            &mut milestone.resource_statements[0]
        {
            evidence[0].kind = EvidenceKind::ModelOutput;
        }
        assert!(milestone.validate().is_err());

        if let ResourceStatement::AcceptedInventoryDelta { evidence, .. } =
            &mut milestone.resource_statements[0]
        {
            evidence[0].kind = EvidenceKind::InventoryReconciliation;
        }
        if let ResourceStatement::AcceptedInventoryDelta {
            delta_milligrams, ..
        } = &mut milestone.resource_statements[0]
        {
            *delta_milligrams = 0;
        }
        assert!(milestone.validate().is_err());
    }

    #[test]
    fn failed_and_not_run_scenarios_are_not_collapsed_into_passes() {
        let mut milestone = valid_milestone();
        milestone.failure_recovery = vec![FailureRecoveryObservation {
            scenario_id: "COMMUNICATIONS-LOSS".to_string(),
            failure_class: "communications_loss".to_string(),
            outcome: FailureRecoveryOutcome::NotRun,
            recovery_duration_milliseconds: None,
            evidence: Vec::new(),
        }];
        assert!(milestone.validate().is_ok());

        milestone.failure_recovery[0].outcome = FailureRecoveryOutcome::Recovered;
        assert!(milestone.validate().is_err());
        milestone.failure_recovery[0].recovery_duration_milliseconds = Some(10);
        assert!(milestone.validate().is_ok());
    }

    #[test]
    fn duplicate_dependency_deltas_are_rejected() {
        let mut milestone = valid_milestone();
        milestone
            .dependency_deltas
            .push(milestone.dependency_deltas[0].clone());
        assert!(milestone.validate().is_err());
    }

    #[test]
    fn canonical_json_fixture_deserializes_and_validates() {
        let milestone: BootstrapMilestone = serde_json::from_str(include_str!(
            "../fixtures/bootstrap-milestone.synthetic.json"
        ))
        .expect("canonical shared fixture must deserialize");
        assert!(milestone.validate().is_ok());
        assert_eq!(
            milestone.fixture_class,
            FixtureClass::SyntheticSoftwareContract
        );
        assert_eq!(
            milestone.resource_statements.len(),
            1,
            "the sample resource is an estimate, not accepted inventory"
        );
    }

    #[test]
    fn unknown_fields_fail_closed_during_deserialization() {
        let mut value = serde_json::to_value(valid_milestone()).expect("serialize record");
        value["authority_basis"]["authority_granted"] = serde_json::Value::Bool(true);
        assert!(serde_json::from_value::<BootstrapMilestone>(value).is_err());
    }

    #[test]
    fn one_artifact_cannot_be_relabelled_as_independent_qualification_review() {
        let mut milestone = valid_milestone();
        let qualification = evidence(
            "https://example.invalid/evidence/same-artifact.json",
            EvidenceKind::PhysicalQualificationReport,
            '6',
        );
        let mut duplicate_review_label = qualification.clone();
        duplicate_review_label.kind = EvidenceKind::IndependentReview;
        milestone.fixture_class = FixtureClass::PhysicalQualification;
        milestone.evidence = vec![qualification, duplicate_review_label];
        assert!(milestone.validate().is_err());
    }

    #[test]
    fn duplicate_fault_scenario_ids_are_rejected() {
        let mut milestone = valid_milestone();
        milestone.failure_recovery.push(milestone.failure_recovery[0].clone());
        assert!(milestone.validate().is_err());
    }

    #[test]
    fn authority_decision_reference_must_be_typed_as_policy_decision() {
        let mut milestone = valid_milestone();
        milestone.authority_basis.decision_evidence = Some(evidence(
            "https://example.invalid/evidence/unrelated-test.json",
            EvidenceKind::TestReport,
            '9',
        ));
        assert!(milestone.validate().is_err());

        milestone.authority_basis.decision_evidence = Some(evidence(
            "https://example.invalid/evidence/policy-decision.json",
            EvidenceKind::PolicyDecision,
            'a',
        ));
        assert!(milestone.validate().is_ok());
    }

    #[test]
    fn calibration_and_standards_documents_do_not_replace_measurement_evidence() {
        let mut milestone = valid_milestone();
        milestone.fixture_class = FixtureClass::OperationalMeasured;
        milestone.evidence = vec![evidence(
            "https://example.invalid/evidence/calibration-certificate.json",
            EvidenceKind::CalibrationCertificate,
            '8',
        )];
        assert!(milestone.validate().is_err());

        milestone.evidence.push(evidence(
            "https://example.invalid/evidence/operational-measurement.json",
            EvidenceKind::OperationalMeasurement,
            '9',
        ));
        milestone.evidence.push(evidence(
            "https://example.invalid/evidence/verification-control.json",
            EvidenceKind::VerificationControlDocument,
            'a',
        ));
        assert!(milestone.validate().is_ok());
    }

    #[test]
    fn physical_qualification_requires_a_separate_independent_review() {
        let mut milestone = valid_milestone();
        milestone.fixture_class = FixtureClass::PhysicalQualification;
        milestone.evidence = vec![
            evidence(
                "https://example.invalid/evidence/qualification.json",
                EvidenceKind::PhysicalQualificationReport,
                '6',
            ),
            evidence(
                "https://example.invalid/evidence/independent-review.json",
                EvidenceKind::IndependentReview,
                '7',
            ),
        ];
        assert!(milestone.validate().is_ok());
        milestone.evidence.pop();
        assert!(milestone.validate().is_err());
    }
}
