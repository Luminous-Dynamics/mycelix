//! GCPN compositional assurance under interacting evidence-chain failures.
//!
//! This deterministic, synthetic evaluator audits claim-level evidence across
//! six civil-protection stages. It deliberately does not compute a single
//! compensatory score: a good forecast cannot compensate for stale evidence,
//! unknown population denominators, late decisions, or hindsight contamination.
//!
//! The canonical FNV-1a fingerprint is for reproducibility diagnostics only;
//! it is not a cryptographic attestation or a security boundary.

use std::collections::BTreeSet;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Stage {
    Forecast,
    Observation,
    CapLifecycle,
    PopulationReach,
    Decision,
    AfterEventReview,
}

impl Stage {
    pub const ALL: [Self; 6] = [
        Self::Forecast,
        Self::Observation,
        Self::CapLifecycle,
        Self::PopulationReach,
        Self::Decision,
        Self::AfterEventReview,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Forecast => "forecast",
            Self::Observation => "observation",
            Self::CapLifecycle => "cap_lifecycle",
            Self::PopulationReach => "population_reach",
            Self::Decision => "decision",
            Self::AfterEventReview => "after_event_review",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClaimDisposition {
    Supports,
    Refutes,
    Indeterminate,
    OutOfScope,
    Unavailable,
}

impl ClaimDisposition {
    pub fn label(self) -> &'static str {
        match self {
            Self::Supports => "supports",
            Self::Refutes => "refutes",
            Self::Indeterminate => "indeterminate",
            Self::OutOfScope => "out_of_scope",
            Self::Unavailable => "unavailable",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvidenceValidity {
    Current,
    Stale,
    Revoked,
    Invalid,
}

impl EvidenceValidity {
    pub fn label(self) -> &'static str {
        match self {
            Self::Current => "current",
            Self::Stale => "stale",
            Self::Revoked => "revoked",
            Self::Invalid => "invalid",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ViolationCode {
    ClaimEvidenceMissing,
    StaleEvidence,
    RevokedEvidence,
    InvalidEvidence,
    EvidenceAfterAsOf,
    SubjectDigestMismatch,
    SourceMismatch,
    VersionMismatch,
    InsufficientIndependentEvidence,
    CommonCauseLineage,
    ForecastIssuedAfterDecisionCutoff,
    InsufficientActionLead,
    CapReceiptMessageMismatch,
    CapReceiptVersionMismatch,
    PopulationDenominatorUnknown,
    PopulationReachExceedsEligible,
    PopulationAccountingMismatch,
    CandidateReferenceFrameMismatch,
    CandidateReferenceSourceMismatch,
    UnsupportedCausalClaim,
    PostprocessingOutcomeLeakage,
}

impl ViolationCode {
    pub fn label(self) -> &'static str {
        match self {
            Self::ClaimEvidenceMissing => "claim_evidence_missing",
            Self::StaleEvidence => "stale_evidence",
            Self::RevokedEvidence => "revoked_evidence",
            Self::InvalidEvidence => "invalid_evidence",
            Self::EvidenceAfterAsOf => "evidence_after_as_of",
            Self::SubjectDigestMismatch => "subject_digest_mismatch",
            Self::SourceMismatch => "source_mismatch",
            Self::VersionMismatch => "version_mismatch",
            Self::InsufficientIndependentEvidence => "insufficient_independent_evidence",
            Self::CommonCauseLineage => "common_cause_lineage",
            Self::ForecastIssuedAfterDecisionCutoff => "forecast_issued_after_decision_cutoff",
            Self::InsufficientActionLead => "insufficient_action_lead",
            Self::CapReceiptMessageMismatch => "cap_receipt_message_mismatch",
            Self::CapReceiptVersionMismatch => "cap_receipt_version_mismatch",
            Self::PopulationDenominatorUnknown => "population_denominator_unknown",
            Self::PopulationReachExceedsEligible => "population_reach_exceeds_eligible",
            Self::PopulationAccountingMismatch => "population_accounting_mismatch",
            Self::CandidateReferenceFrameMismatch => "candidate_reference_frame_mismatch",
            Self::CandidateReferenceSourceMismatch => "candidate_reference_source_mismatch",
            Self::UnsupportedCausalClaim => "unsupported_causal_claim",
            Self::PostprocessingOutcomeLeakage => "postprocessing_outcome_leakage",
        }
    }

    fn stage(self) -> Stage {
        match self {
            Self::StaleEvidence
            | Self::RevokedEvidence
            | Self::InvalidEvidence
            | Self::SubjectDigestMismatch
            | Self::SourceMismatch
            | Self::VersionMismatch
            | Self::InsufficientIndependentEvidence
            | Self::CommonCauseLineage => Stage::Observation,
            Self::ForecastIssuedAfterDecisionCutoff
            | Self::InsufficientActionLead
            | Self::PostprocessingOutcomeLeakage => Stage::Decision,
            Self::CapReceiptMessageMismatch | Self::CapReceiptVersionMismatch => {
                Stage::CapLifecycle
            }
            Self::PopulationDenominatorUnknown
            | Self::PopulationReachExceedsEligible
            | Self::PopulationAccountingMismatch => Stage::PopulationReach,
            Self::UnsupportedCausalClaim | Self::EvidenceAfterAsOf => Stage::AfterEventReview,
            Self::ClaimEvidenceMissing => Stage::Forecast,
            Self::CandidateReferenceFrameMismatch
            | Self::CandidateReferenceSourceMismatch => Stage::Forecast,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClaimRequirement {
    pub stage: Stage,
    pub claim_id: String,
    pub subject_digest: String,
    pub expected_source: Option<String>,
    pub expected_version: Option<String>,
    pub minimum_independent_supporting_lineages: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StageEvidence {
    pub evidence_id: String,
    pub stage: Stage,
    pub claim_id: String,
    pub subject_digest: String,
    pub source: String,
    pub version: String,
    pub event_time: u64,
    pub recorded_at: u64,
    pub validity: EvidenceValidity,
    pub disposition: ClaimDisposition,
    /// Shared causal/root lineage: calibration, network, retrieval, etc.
    /// Distinct evaluator names do not create independence when this is equal.
    pub lineage_root: String,
    pub evaluator_id: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ForecastTiming {
    pub issued_at: u64,
    pub decision_cutoff: u64,
    pub event_valid_at: u64,
    pub minimum_action_lead: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CapReceiptLink {
    pub current_message_id: String,
    pub current_version: String,
    pub receipt_message_id: String,
    pub receipt_version: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PopulationAccounting {
    pub eligible_denominator: Option<u64>,
    pub observed_cases: u64,
    pub typed_missing_cases: u64,
    pub unique_reach: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerificationAccounting {
    pub eligible_cases: u64,
    pub observed_cases: u64,
    pub typed_missing_cases: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PairedComparison {
    pub candidate_case_frame_digest: String,
    pub reference_case_frame_digest: String,
    pub candidate_source: String,
    pub reference_source: String,
    pub candidate_version: String,
    pub reference_version: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CausalClaim {
    pub claim_made: bool,
    pub identification_method_id: Option<String>,
    pub design_id: Option<String>,
    pub supporting_evidence_ids: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PostProcessingFacts {
    pub intrinsic_skill_claim: bool,
    pub calibration_uses_evaluation_outcomes: bool,
    pub calibration_trained_through: Option<u64>,
    pub evaluation_period_start: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ScenarioFacts {
    pub forecast_timing: Option<ForecastTiming>,
    pub cap_receipt: Option<CapReceiptLink>,
    pub population: Option<PopulationAccounting>,
    pub verification: Option<VerificationAccounting>,
    pub paired_comparison: Option<PairedComparison>,
    pub causal_claim: Option<CausalClaim>,
    pub post_processing: Option<PostProcessingFacts>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Scenario {
    pub scenario_id: String,
    pub specification_digest: String,
    pub as_of_cutoff: u64,
    pub requirements: Vec<ClaimRequirement>,
    pub evidence: Vec<StageEvidence>,
    pub facts: ScenarioFacts,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClaimResult {
    pub stage: Stage,
    pub claim_id: String,
    pub disposition: ClaimDisposition,
    pub evidence_count: usize,
    pub independent_supporting_lineages: usize,
    pub violations: Vec<ViolationCode>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuditReport {
    pub scenario_id: String,
    pub specification_digest: String,
    pub as_of_cutoff: u64,
    pub claims: Vec<ClaimResult>,
    pub violations: Vec<ViolationCode>,
    /// FNV-1a 64-bit, non-cryptographic reproducibility fingerprint.
    pub reproducibility_fingerprint: String,
}

impl Scenario {
    pub fn validate(&self) -> Result<(), String> {
        if !valid_identifier(&self.scenario_id) {
            return Err("scenario_id must be a non-empty canonical identifier".into());
        }
        if !valid_identifier(&self.specification_digest) {
            return Err("specification_digest must be a non-empty canonical identifier".into());
        }
        if self.requirements.is_empty() {
            return Err("scenario must declare at least one claim requirement".into());
        }

        let mut requirement_keys = BTreeSet::new();
        for requirement in &self.requirements {
            if !valid_identifier(&requirement.claim_id)
                || !valid_identifier(&requirement.subject_digest)
            {
                return Err("claim and subject identifiers must be canonical".into());
            }
            if requirement.minimum_independent_supporting_lineages == 0 {
                return Err("minimum independent lineages must be at least one".into());
            }
            if let Some(source) = &requirement.expected_source {
                if !valid_identifier(source) {
                    return Err("expected source must be a canonical identifier".into());
                }
            }
            if let Some(version) = &requirement.expected_version {
                if !valid_identifier(version) {
                    return Err("expected version must be a canonical identifier".into());
                }
            }
            if !requirement_keys.insert((requirement.stage, requirement.claim_id.as_str())) {
                return Err("duplicate stage/claim requirement".into());
            }
        }

        let mut evidence_ids = BTreeSet::new();
        for evidence in &self.evidence {
            for (name, value) in [
                ("evidence_id", evidence.evidence_id.as_str()),
                ("claim_id", evidence.claim_id.as_str()),
                ("subject_digest", evidence.subject_digest.as_str()),
                ("source", evidence.source.as_str()),
                ("version", evidence.version.as_str()),
                ("lineage_root", evidence.lineage_root.as_str()),
                ("evaluator_id", evidence.evaluator_id.as_str()),
            ] {
                if !valid_identifier(value) {
                    return Err(format!("{name} must be a canonical identifier"));
                }
            }
            if !evidence_ids.insert(evidence.evidence_id.as_str()) {
                return Err(format!("duplicate evidence id: {}", evidence.evidence_id));
            }
        }

        Ok(())
    }

    pub fn audit(&self) -> Result<AuditReport, String> {
        self.validate()?;

        let mut claims = Vec::with_capacity(self.requirements.len());
        let mut all_violations = BTreeSet::new();

        for requirement in &self.requirements {
            let matching: Vec<&StageEvidence> = self
                .evidence
                .iter()
                .filter(|evidence| {
                    evidence.stage == requirement.stage
                        && evidence.claim_id == requirement.claim_id
                })
                .collect();

            let mut violations = BTreeSet::new();
            let mut supporting_lineages = BTreeSet::new();
            let mut supporting_records = 0usize;
            let mut refuting_records = 0usize;
            let mut out_of_scope_records = 0usize;
            let mut unavailable_records = 0usize;
            let mut current_records = 0usize;

            if matching.is_empty() {
                violations.insert(ViolationCode::ClaimEvidenceMissing);
            }

            for evidence in &matching {
                if evidence.recorded_at > self.as_of_cutoff {
                    violations.insert(ViolationCode::EvidenceAfterAsOf);
                    continue;
                }

                match evidence.validity {
                    EvidenceValidity::Current => current_records += 1,
                    EvidenceValidity::Stale => {
                        violations.insert(ViolationCode::StaleEvidence);
                        continue;
                    }
                    EvidenceValidity::Revoked => {
                        violations.insert(ViolationCode::RevokedEvidence);
                        continue;
                    }
                    EvidenceValidity::Invalid => {
                        violations.insert(ViolationCode::InvalidEvidence);
                        continue;
                    }
                }

                let mut aligned = true;
                if evidence.subject_digest != requirement.subject_digest {
                    violations.insert(ViolationCode::SubjectDigestMismatch);
                    aligned = false;
                }
                if requirement
                    .expected_source
                    .as_ref()
                    .is_some_and(|source| source != &evidence.source)
                {
                    violations.insert(ViolationCode::SourceMismatch);
                    aligned = false;
                }
                if requirement
                    .expected_version
                    .as_ref()
                    .is_some_and(|version| version != &evidence.version)
                {
                    violations.insert(ViolationCode::VersionMismatch);
                    aligned = false;
                }
                if !aligned {
                    continue;
                }

                match evidence.disposition {
                    ClaimDisposition::Supports => {
                        supporting_records += 1;
                        supporting_lineages.insert(evidence.lineage_root.as_str());
                    }
                    ClaimDisposition::Refutes => refuting_records += 1,
                    ClaimDisposition::Indeterminate => {}
                    ClaimDisposition::OutOfScope => out_of_scope_records += 1,
                    ClaimDisposition::Unavailable => unavailable_records += 1,
                }
            }

            if supporting_records > 1
                && supporting_lineages.len()
                    < requirement.minimum_independent_supporting_lineages
            {
                violations.insert(ViolationCode::CommonCauseLineage);
            }

            if supporting_lineages.len()
                < requirement.minimum_independent_supporting_lineages
                && !matching.is_empty()
            {
                violations.insert(ViolationCode::InsufficientIndependentEvidence);
            }

            let disposition = if matching.is_empty() {
                ClaimDisposition::Unavailable
            } else if supporting_records > 0 && refuting_records > 0 {
                ClaimDisposition::Indeterminate
            } else if refuting_records > 0 {
                ClaimDisposition::Refutes
            } else if !violations.is_empty() {
                ClaimDisposition::Indeterminate
            } else if supporting_lineages.len()
                >= requirement.minimum_independent_supporting_lineages
            {
                ClaimDisposition::Supports
            } else if current_records > 0 && out_of_scope_records == current_records {
                ClaimDisposition::OutOfScope
            } else if current_records > 0 && unavailable_records == current_records {
                ClaimDisposition::Unavailable
            } else {
                ClaimDisposition::Indeterminate
            };

            for violation in &violations {
                all_violations.insert(*violation);
            }

            claims.push(ClaimResult {
                stage: requirement.stage,
                claim_id: requirement.claim_id.clone(),
                disposition,
                evidence_count: matching.len(),
                independent_supporting_lineages: supporting_lineages.len(),
                violations: violations.into_iter().collect(),
            });
        }

        let fact_violations = self.audit_facts();
        for violation in fact_violations {
            all_violations.insert(violation);
        }

        // Any scenario-level violation also makes the corresponding stage
        // claim non-supporting. This is intentionally fail-closed.
        for claim in &mut claims {
            let stage_violations: Vec<ViolationCode> = all_violations
                .iter()
                .copied()
                .filter(|violation| violation.stage() == claim.stage)
                .collect();
            for violation in stage_violations {
                if !claim.violations.contains(&violation) {
                    claim.violations.push(violation);
                }
            }
            claim.violations.sort();
            if !claim.violations.is_empty() && claim.disposition == ClaimDisposition::Supports {
                claim.disposition = ClaimDisposition::Indeterminate;
            }
        }

        claims.sort_by(|left, right| {
            (left.stage, &left.claim_id).cmp(&(right.stage, &right.claim_id))
        });
        let violations: Vec<ViolationCode> = all_violations.into_iter().collect();
        let mut report = AuditReport {
            scenario_id: self.scenario_id.clone(),
            specification_digest: self.specification_digest.clone(),
            as_of_cutoff: self.as_of_cutoff,
            claims,
            violations,
            reproducibility_fingerprint: String::new(),
        };
        report.reproducibility_fingerprint = fingerprint(&report.canonical_bytes());
        Ok(report)
    }

    fn audit_facts(&self) -> BTreeSet<ViolationCode> {
        let mut violations = BTreeSet::new();

        if let Some(timing) = &self.facts.forecast_timing {
            if timing.issued_at > timing.decision_cutoff {
                violations.insert(ViolationCode::ForecastIssuedAfterDecisionCutoff);
            }
            if timing.event_valid_at.saturating_sub(timing.decision_cutoff)
                < timing.minimum_action_lead
            {
                violations.insert(ViolationCode::InsufficientActionLead);
            }
        }

        if let Some(receipt) = &self.facts.cap_receipt {
            if receipt.current_message_id != receipt.receipt_message_id {
                violations.insert(ViolationCode::CapReceiptMessageMismatch);
            }
            if receipt.current_version != receipt.receipt_version {
                violations.insert(ViolationCode::CapReceiptVersionMismatch);
            }
        }

        if let Some(population) = &self.facts.population {
            match population.eligible_denominator {
                None => {
                    violations.insert(ViolationCode::PopulationDenominatorUnknown);
                }
                Some(eligible) => {
                    if population.unique_reach > eligible {
                        violations.insert(ViolationCode::PopulationReachExceedsEligible);
                    }
                    if population.observed_cases
                        .checked_add(population.typed_missing_cases)
                        != Some(eligible)
                    {
                        violations.insert(ViolationCode::PopulationAccountingMismatch);
                    }
                }
            }
        }

        if let Some(verification) = &self.facts.verification {
            if verification.observed_cases
                .checked_add(verification.typed_missing_cases)
                != Some(verification.eligible_cases)
            {
                violations.insert(ViolationCode::PopulationAccountingMismatch);
            }
        }

        if let Some(pairing) = &self.facts.paired_comparison {
            if pairing.candidate_case_frame_digest != pairing.reference_case_frame_digest {
                violations.insert(ViolationCode::CandidateReferenceFrameMismatch);
            }
            if pairing.candidate_source != pairing.reference_source
                || pairing.candidate_version != pairing.reference_version
            {
                violations.insert(ViolationCode::CandidateReferenceSourceMismatch);
            }
        }

        if let Some(causal) = &self.facts.causal_claim {
            if causal.claim_made
                && (causal
                    .identification_method_id
                    .as_ref()
                    .is_none_or(|value| value.trim().is_empty())
                    || causal
                        .design_id
                        .as_ref()
                        .is_none_or(|value| value.trim().is_empty())
                    || causal.supporting_evidence_ids.is_empty())
            {
                violations.insert(ViolationCode::UnsupportedCausalClaim);
            }
        }

        if let Some(post_processing) = &self.facts.post_processing {
            if post_processing.intrinsic_skill_claim
                && (post_processing.calibration_uses_evaluation_outcomes
                    || matches!(
                        (
                            post_processing.calibration_trained_through,
                            post_processing.evaluation_period_start
                        ),
                        (Some(trained_through), Some(evaluation_start))
                            if trained_through >= evaluation_start
                    ))
            {
                violations.insert(ViolationCode::PostprocessingOutcomeLeakage);
            }
        }

        violations
    }
}

impl AuditReport {
    /// Canonical newline-delimited representation. All collections are sorted.
    /// Identifiers are validated to exclude separators and control characters.
    pub fn canonical_bytes(&self) -> Vec<u8> {
        let mut lines = vec![
            format!("scenario\t{}", self.scenario_id),
            format!("specification\t{}", self.specification_digest),
            format!("as_of\t{}", self.as_of_cutoff),
        ];
        for claim in &self.claims {
            let violations = claim
                .violations
                .iter()
                .map(|value| value.label())
                .collect::<Vec<_>>()
                .join(",");
            lines.push(format!(
                "claim\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                claim.stage.label(),
                claim.claim_id,
                claim.disposition.label(),
                claim.evidence_count,
                claim.independent_supporting_lineages,
                violations,
                // Fixed marker makes line format versionable.
                "v1"
            ));
        }
        for violation in &self.violations {
            lines.push(format!("violation\t{}", violation.label()));
        }
        lines.join("\n").into_bytes()
    }
}

fn valid_identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 256
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"-_.:".contains(&byte))
}

fn fingerprint(bytes: &[u8]) -> String {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("fnv1a64:{hash:016x}")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn evidence(
        id: &str,
        stage: Stage,
        claim: &str,
        lineage: &str,
        disposition: ClaimDisposition,
    ) -> StageEvidence {
        StageEvidence {
            evidence_id: id.into(),
            stage,
            claim_id: claim.into(),
            subject_digest: "subject-1".into(),
            source: "source-a".into(),
            version: "v1".into(),
            event_time: 10,
            recorded_at: 20,
            validity: EvidenceValidity::Current,
            disposition,
            lineage_root: lineage.into(),
            evaluator_id: format!("evaluator-{id}"),
        }
    }

    fn requirement(stage: Stage, claim: &str, minimum: usize) -> ClaimRequirement {
        ClaimRequirement {
            stage,
            claim_id: claim.into(),
            subject_digest: "subject-1".into(),
            expected_source: Some("source-a".into()),
            expected_version: Some("v1".into()),
            minimum_independent_supporting_lineages: minimum,
        }
    }

    fn scenario() -> Scenario {
        Scenario {
            scenario_id: "fixture-happy-path".into(),
            specification_digest: "spec-v1".into(),
            as_of_cutoff: 100,
            requirements: vec![
                requirement(Stage::Forecast, "forecast-valid", 1),
                requirement(Stage::Observation, "reference-valid", 2),
                requirement(Stage::CapLifecycle, "receipt-valid", 1),
                requirement(Stage::PopulationReach, "reach-valid", 1),
                requirement(Stage::Decision, "decision-valid", 1),
                requirement(Stage::AfterEventReview, "review-valid", 1),
            ],
            evidence: vec![
                evidence("f-1", Stage::Forecast, "forecast-valid", "forecast-root", ClaimDisposition::Supports),
                evidence("o-1", Stage::Observation, "reference-valid", "sensor-root-a", ClaimDisposition::Supports),
                evidence("o-2", Stage::Observation, "reference-valid", "sensor-root-b", ClaimDisposition::Supports),
                evidence("c-1", Stage::CapLifecycle, "receipt-valid", "cap-root", ClaimDisposition::Supports),
                evidence("p-1", Stage::PopulationReach, "reach-valid", "population-root", ClaimDisposition::Supports),
                evidence("d-1", Stage::Decision, "decision-valid", "decision-root", ClaimDisposition::Supports),
                evidence("a-1", Stage::AfterEventReview, "review-valid", "review-root", ClaimDisposition::Supports),
            ],
            facts: ScenarioFacts {
                forecast_timing: Some(ForecastTiming {
                    issued_at: 10,
                    decision_cutoff: 20,
                    event_valid_at: 100,
                    minimum_action_lead: 30,
                }),
                cap_receipt: Some(CapReceiptLink {
                    current_message_id: "cap-42".into(),
                    current_version: "2".into(),
                    receipt_message_id: "cap-42".into(),
                    receipt_version: "2".into(),
                }),
                population: Some(PopulationAccounting {
                    eligible_denominator: Some(1000),
                    observed_cases: 900,
                    typed_missing_cases: 100,
                    unique_reach: 700,
                }),
                verification: Some(VerificationAccounting {
                    eligible_cases: 100,
                    observed_cases: 90,
                    typed_missing_cases: 10,
                }),
                paired_comparison: Some(PairedComparison {
                    candidate_case_frame_digest: "frame-1".into(),
                    reference_case_frame_digest: "frame-1".into(),
                    candidate_source: "source-a".into(),
                    reference_source: "source-a".into(),
                    candidate_version: "v1".into(),
                    reference_version: "v1".into(),
                }),
                causal_claim: Some(CausalClaim {
                    claim_made: false,
                    identification_method_id: None,
                    design_id: None,
                    supporting_evidence_ids: vec![],
                }),
                post_processing: Some(PostProcessingFacts {
                    intrinsic_skill_claim: true,
                    calibration_uses_evaluation_outcomes: false,
                    calibration_trained_through: Some(5),
                    evaluation_period_start: Some(20),
                }),
            },
        }
    }

    #[test]
    fn clean_composed_scenario_supports_each_required_claim() {
        let report = scenario().audit().unwrap();
        assert_eq!(report.claims.len(), 6);
        assert!(report.claims.iter().all(|claim| claim.disposition == ClaimDisposition::Supports));
        assert!(report.violations.is_empty());
    }

    #[test]
    fn common_cause_lineage_does_not_count_as_independent_witnesses() {
        let mut input = scenario();
        for evidence in input.evidence.iter_mut().filter(|item| item.stage == Stage::Observation) {
            evidence.lineage_root = "shared-calibration-root".into();
        }
        let report = input.audit().unwrap();
        let observation = report.claims.iter().find(|claim| claim.stage == Stage::Observation).unwrap();
        assert_eq!(observation.independent_supporting_lineages, 1);
        assert_eq!(observation.disposition, ClaimDisposition::Indeterminate);
        assert!(observation.violations.contains(&ViolationCode::CommonCauseLineage));
    }

    #[test]
    fn different_evaluator_names_do_not_create_independence() {
        let mut input = scenario();
        input.evidence[1].evaluator_id = "different-evaluator-1".into();
        input.evidence[2].evaluator_id = "different-evaluator-2".into();
        input.evidence[1].lineage_root = "same-network-root".into();
        input.evidence[2].lineage_root = "same-network-root".into();
        let report = input.audit().unwrap();
        let observation = report.claims.iter().find(|claim| claim.stage == Stage::Observation).unwrap();
        assert_eq!(observation.independent_supporting_lineages, 1);
        assert!(observation.violations.contains(&ViolationCode::CommonCauseLineage));
    }

    #[test]
    fn stale_evidence_cannot_support_a_claim() {
        let mut input = scenario();
        input.evidence[0].validity = EvidenceValidity::Stale;
        let report = input.audit().unwrap();
        let forecast = report.claims.iter().find(|claim| claim.stage == Stage::Forecast).unwrap();
        assert_eq!(forecast.disposition, ClaimDisposition::Indeterminate);
        assert!(forecast.violations.contains(&ViolationCode::StaleEvidence));
    }

    #[test]
    fn revoked_and_invalid_evidence_are_distinct() {
        let mut input = scenario();
        input.evidence[0].validity = EvidenceValidity::Revoked;
        let report = input.audit().unwrap();
        assert!(report.violations.contains(&ViolationCode::RevokedEvidence));

        input.evidence[0].validity = EvidenceValidity::Invalid;
        let report = input.audit().unwrap();
        assert!(report.violations.contains(&ViolationCode::InvalidEvidence));
    }

    #[test]
    fn evidence_after_cutoff_is_not_visible_in_historical_replay() {
        let mut input = scenario();
        input.evidence[6].recorded_at = 101;
        let report = input.audit().unwrap();
        let review = report.claims.iter().find(|claim| claim.stage == Stage::AfterEventReview).unwrap();
        assert_eq!(review.disposition, ClaimDisposition::Indeterminate);
        assert!(review.violations.contains(&ViolationCode::EvidenceAfterAsOf));
    }

    #[test]
    fn late_forecast_and_insufficient_lead_are_separate_violations() {
        let mut input = scenario();
        input.facts.forecast_timing = Some(ForecastTiming {
            issued_at: 30,
            decision_cutoff: 20,
            event_valid_at: 35,
            minimum_action_lead: 30,
        });
        let report = input.audit().unwrap();
        assert!(report.violations.contains(&ViolationCode::ForecastIssuedAfterDecisionCutoff));
        assert!(report.violations.contains(&ViolationCode::InsufficientActionLead));
    }

    #[test]
    fn cap_receipt_must_match_exact_message_and_version() {
        let mut input = scenario();
        input.facts.cap_receipt.as_mut().unwrap().receipt_message_id = "cap-41".into();
        input.facts.cap_receipt.as_mut().unwrap().receipt_version = "1".into();
        let report = input.audit().unwrap();
        assert!(report.violations.contains(&ViolationCode::CapReceiptMessageMismatch));
        assert!(report.violations.contains(&ViolationCode::CapReceiptVersionMismatch));
    }

    #[test]
    fn unknown_population_denominator_is_not_replaced_by_channel_totals() {
        let mut input = scenario();
        input.facts.population.as_mut().unwrap().eligible_denominator = None;
        let report = input.audit().unwrap();
        assert!(report.violations.contains(&ViolationCode::PopulationDenominatorUnknown));
    }

    #[test]
    fn population_overreach_and_unaccounted_cases_are_detected() {
        let mut input = scenario();
        input.facts.population.as_mut().unwrap().unique_reach = 1001;
        input.facts.population.as_mut().unwrap().typed_missing_cases = 10;
        let report = input.audit().unwrap();
        assert!(report.violations.contains(&ViolationCode::PopulationReachExceedsEligible));
        assert!(report.violations.contains(&ViolationCode::PopulationAccountingMismatch));
    }

    #[test]
    fn verification_frame_must_account_for_every_eligible_case() {
        let mut input = scenario();
        input.facts.verification.as_mut().unwrap().observed_cases = 80;
        let report = input.audit().unwrap();
        assert!(report.violations.contains(&ViolationCode::PopulationAccountingMismatch));
    }

    #[test]
    fn candidate_and_reference_must_share_frame_and_verification_lineage() {
        let mut input = scenario();
        input.facts.paired_comparison.as_mut().unwrap().reference_case_frame_digest = "frame-2".into();
        input.facts.paired_comparison.as_mut().unwrap().reference_source = "source-b".into();
        input.facts.paired_comparison.as_mut().unwrap().reference_version = "v2".into();
        let report = input.audit().unwrap();
        assert!(report.violations.contains(&ViolationCode::CandidateReferenceFrameMismatch));
        assert!(report.violations.contains(&ViolationCode::CandidateReferenceSourceMismatch));
    }

    #[test]
    fn unsupported_causal_claim_is_not_promoted_from_chronology() {
        let mut input = scenario();
        input.facts.causal_claim = Some(CausalClaim {
            claim_made: true,
            identification_method_id: None,
            design_id: Some("before-after".into()),
            supporting_evidence_ids: vec!["event-order".into()],
        });
        let report = input.audit().unwrap();
        assert!(report.violations.contains(&ViolationCode::UnsupportedCausalClaim));
    }

    #[test]
    fn calibration_using_confirmatory_outcomes_is_not_intrinsic_skill() {
        let mut input = scenario();
        input.facts.post_processing.as_mut().unwrap().calibration_uses_evaluation_outcomes = true;
        let report = input.audit().unwrap();
        assert!(report.violations.contains(&ViolationCode::PostprocessingOutcomeLeakage));
    }

    #[test]
    fn calibration_cutoff_must_precede_evaluation_period() {
        let mut input = scenario();
        input.facts.post_processing.as_mut().unwrap().calibration_trained_through = Some(20);
        let report = input.audit().unwrap();
        assert!(report.violations.contains(&ViolationCode::PostprocessingOutcomeLeakage));
    }

    #[test]
    fn subject_source_and_version_mismatch_prevent_support() {
        let mut input = scenario();
        input.evidence[0].subject_digest = "wrong-subject".into();
        input.evidence[0].source = "wrong-source".into();
        input.evidence[0].version = "v9".into();
        let report = input.audit().unwrap();
        let forecast = report.claims.iter().find(|claim| claim.stage == Stage::Forecast).unwrap();
        assert_eq!(forecast.disposition, ClaimDisposition::Indeterminate);
        assert!(forecast.violations.contains(&ViolationCode::SubjectDigestMismatch));
        assert!(forecast.violations.contains(&ViolationCode::SourceMismatch));
        assert!(forecast.violations.contains(&ViolationCode::VersionMismatch));
    }

    #[test]
    fn valid_refuting_evidence_refutes_the_claim() {
        let mut input = scenario();
        input.evidence[0].disposition = ClaimDisposition::Refutes;
        let report = input.audit().unwrap();
        let forecast = report.claims.iter().find(|claim| claim.stage == Stage::Forecast).unwrap();
        assert_eq!(forecast.disposition, ClaimDisposition::Refutes);
    }

    #[test]
    fn conflicting_current_support_and_refutation_is_indeterminate() {
        let mut input = scenario();
        let mut conflict = input.evidence[0].clone();
        conflict.evidence_id = "f-2".into();
        conflict.lineage_root = "independent-forecast-root".into();
        conflict.disposition = ClaimDisposition::Refutes;
        input.evidence.push(conflict);
        let report = input.audit().unwrap();
        let forecast = report.claims.iter().find(|claim| claim.stage == Stage::Forecast).unwrap();
        assert_eq!(forecast.disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn absent_claim_evidence_is_unavailable_not_a_pass() {
        let mut input = scenario();
        input.evidence.retain(|item| item.stage != Stage::Forecast);
        let report = input.audit().unwrap();
        let forecast = report.claims.iter().find(|claim| claim.stage == Stage::Forecast).unwrap();
        assert_eq!(forecast.disposition, ClaimDisposition::Indeterminate);
        assert!(forecast.violations.contains(&ViolationCode::ClaimEvidenceMissing));
    }

    #[test]
    fn duplicate_evidence_ids_reject_the_scenario() {
        let mut input = scenario();
        input.evidence[1].evidence_id = input.evidence[0].evidence_id.clone();
        assert!(input.validate().unwrap_err().contains("duplicate evidence id"));
    }

    #[test]
    fn record_order_does_not_change_canonical_result() {
        let mut first = scenario();
        let mut second = first.clone();
        second.evidence.reverse();
        second.requirements.reverse();
        let first_report = first.audit().unwrap();
        let second_report = second.audit().unwrap();
        assert_eq!(first_report, second_report);
        assert_eq!(first_report.canonical_bytes(), second_report.canonical_bytes());
    }

    #[test]
    fn irrelevant_out_of_scope_evidence_does_not_change_supported_claims() {
        let mut input = scenario();
        let mut irrelevant = evidence(
            "irrelevant-1",
            Stage::Observation,
            "different-claim",
            "irrelevant-root",
            ClaimDisposition::OutOfScope,
        );
        irrelevant.subject_digest = "other-subject".into();
        input.evidence.push(irrelevant);
        let report = input.audit().unwrap();
        assert!(report.claims.iter().all(|claim| claim.disposition == ClaimDisposition::Supports));
    }

    #[test]
    fn same_input_replay_has_same_fingerprint() {
        let input = scenario();
        let first = input.audit().unwrap();
        let second = input.audit().unwrap();
        assert_eq!(first.reproducibility_fingerprint, second.reproducibility_fingerprint);
    }

    #[test]
    fn canonical_fingerprint_changes_when_a_claim_disposition_changes() {
        let mut input = scenario();
        let first = input.audit().unwrap();
        input.evidence[0].disposition = ClaimDisposition::Refutes;
        let second = input.audit().unwrap();
        assert_ne!(first.reproducibility_fingerprint, second.reproducibility_fingerprint);
    }

    #[test]
    fn invalid_identifiers_and_duplicate_claim_requirements_are_rejected() {
        let mut input = scenario();
        input.scenario_id = "not a canonical id".into();
        assert!(input.validate().is_err());

        let mut input = scenario();
        input.requirements.push(input.requirements[0].clone());
        assert!(input.validate().unwrap_err().contains("duplicate stage/claim"));
    }

    #[test]
    fn all_six_stages_are_required_in_the_happy_path_fixture() {
        let report = scenario().audit().unwrap();
        let stages: BTreeSet<Stage> = report.claims.iter().map(|claim| claim.stage).collect();
        assert_eq!(stages.len(), Stage::ALL.len());
    }
}
