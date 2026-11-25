//! Longitudinal team capability-outcome contract validation.
//!
//! This validates whether a synthetic or study observation panel is eligible
//! for preregistered analysis. It is not a causal estimator, ethics approval,
//! human-subject recruitment system, or proof that an AI-off receipt is genuine.

use std::collections::{BTreeMap, BTreeSet};

use super::{is_sha256, QualificationReport, QualificationStatus};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum CapabilityStage {
    IndependentBaseline,
    AssistedCollaboration,
    ImmediateAiOff,
    DelayedAiOff,
    FarTransfer,
    Resilience,
}

impl CapabilityStage {
    const ALL: [Self; 6] = [
        Self::IndependentBaseline,
        Self::AssistedCollaboration,
        Self::ImmediateAiOff,
        Self::DelayedAiOff,
        Self::FarTransfer,
        Self::Resilience,
    ];

    fn is_ai_off(self) -> bool {
        matches!(
            self,
            Self::ImmediateAiOff | Self::DelayedAiOff | Self::FarTransfer | Self::Resilience
        )
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AccommodationState {
    NotRequired,
    Provided,
    Withheld,
    Unknown,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CapabilityMeasurement {
    /// Normalized decision quality; higher is better.
    pub decision_quality: f64,
    /// Fraction of task-relevant, uniquely held information surfaced and retained.
    pub unique_information_preservation: f64,
    /// Fraction of material evidence claims independently verified and traceable.
    pub verified_evidence_coverage: f64,
    /// Agency measure; higher means more control over decisions and assistance.
    pub agency: f64,
    /// Participant/team burden measure; higher means greater burden.
    pub burden: f64,
    /// Calibration measure; higher means better calibrated confidence.
    pub calibration: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CapabilityOutcomeObservation {
    pub run_id: String,
    pub team_id: String,
    pub team_roster_digest: String,
    pub team_size: u32,
    pub stage: CapabilityStage,
    pub task_family_id: String,
    pub task_digest: String,
    pub evaluator_digest: String,
    pub ai_available: bool,
    pub ai_used: bool,
    pub ai_artifact_reachable: bool,
    pub accommodation_state: AccommodationState,
    /// Voluntary pause/stop is missing outcome data, never a zero score or failure.
    pub voluntary_stop: bool,
    pub measurement: Option<CapabilityMeasurement>,
    /// The externally qualified AI-off receipt status for withdrawal/transfer stages.
    pub ai_off_qualification: Option<QualificationStatus>,
    /// Digest/reference to the independently retained AI-off receipt.
    pub ai_off_receipt_digest: Option<String>,
    /// Whole days elapsed since the assisted collaboration phase ended.
    pub elapsed_days_since_collaboration: u32,
    /// Required for the resilience perturbation stage.
    pub perturbation_id: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CapabilityStudyContract {
    pub run_id: String,
    pub evaluator_digest: String,
    pub baseline_task_family_id: String,
    pub assisted_task_family_id: String,
    pub far_transfer_task_family_id: String,
    pub minimum_delay_days: u32,
    pub expected_team_ids: Vec<String>,
}

type TeamStageKey<'a> = (&'a str, CapabilityStage);
type OutcomePanel<'a> = BTreeMap<TeamStageKey<'a>, &'a CapabilityOutcomeObservation>;

fn valid_unit_interval(value: f64) -> bool {
    value.is_finite() && (0.0..=1.0).contains(&value)
}

fn valid_measurement(measurement: &CapabilityMeasurement) -> bool {
    [
        measurement.decision_quality,
        measurement.unique_information_preservation,
        measurement.verified_evidence_coverage,
        measurement.agency,
        measurement.burden,
        measurement.calibration,
    ]
    .into_iter()
    .all(valid_unit_interval)
}

fn indeterminate(report: &mut QualificationReport, reason: impl Into<String>) {
    report.indeterminate(reason);
}

fn fail(report: &mut QualificationReport, reason: impl Into<String>) {
    report.fail(reason);
}

/// Validates a complete repeated-measures team panel before statistical analysis.
///
/// PASS means the panel satisfies this structural contract. It does not mean the
/// intervention improved capability. Estimate causal effects separately using the
/// preregistered team-level estimand and cluster-aware inference.
pub fn qualify_capability_outcomes(
    contract: &CapabilityStudyContract,
    observations: &[CapabilityOutcomeObservation],
) -> QualificationReport {
    let mut report = QualificationReport {
        status: QualificationStatus::Pass,
        reasons: Vec::new(),
    };

    if contract.run_id.trim().is_empty()
        || contract.baseline_task_family_id.trim().is_empty()
        || contract.assisted_task_family_id.trim().is_empty()
        || contract.far_transfer_task_family_id.trim().is_empty()
    {
        fail(&mut report, "study and task-family identifiers must be non-empty");
    }
    if !is_sha256(&contract.evaluator_digest) {
        fail(&mut report, "study evaluator digest is malformed");
    }
    if contract.minimum_delay_days == 0 {
        fail(&mut report, "minimum delayed assessment interval must be positive");
    }
    if contract.far_transfer_task_family_id == contract.baseline_task_family_id
        || contract.far_transfer_task_family_id == contract.assisted_task_family_id
    {
        fail(
            &mut report,
            "far-transfer task family must differ from baseline and assisted task families",
        );
    }

    let mut expected_teams = BTreeSet::new();
    for team_id in &contract.expected_team_ids {
        if team_id.trim().is_empty() || !expected_teams.insert(team_id.as_str()) {
            fail(&mut report, "expected team identities must be non-empty and unique");
        }
    }
    if expected_teams.is_empty() {
        fail(&mut report, "expected team roster is empty");
    }
    if report.status == QualificationStatus::Fail {
        return report;
    }

    let mut panel: OutcomePanel<'_> = BTreeMap::new();
    let mut roster_digests: BTreeMap<&str, &str> = BTreeMap::new();
    let mut team_sizes: BTreeMap<&str, u32> = BTreeMap::new();

    for observation in observations {
        if observation.run_id != contract.run_id {
            fail(&mut report, "observation belongs to a different study run");
        }
        if !expected_teams.contains(observation.team_id.as_str()) {
            fail(&mut report, "observation contains an unexpected team identity");
        }
        if observation.team_id.trim().is_empty() || observation.team_size < 2 {
            fail(&mut report, "team identity must be non-empty and team size at least two");
        }
        if !is_sha256(&observation.team_roster_digest)
            || !is_sha256(&observation.task_digest)
            || !is_sha256(&observation.evaluator_digest)
        {
            fail(&mut report, "team, task, or evaluator digest is malformed");
        }
        if observation.evaluator_digest != contract.evaluator_digest {
            fail(&mut report, "observation evaluator differs from preregistered evaluator");
        }

        if let Some(previous) = roster_digests.insert(
            observation.team_id.as_str(),
            observation.team_roster_digest.as_str(),
        ) {
            if previous != observation.team_roster_digest {
                fail(&mut report, "team roster changed across repeated measures");
            }
        }
        if let Some(previous) =
            team_sizes.insert(observation.team_id.as_str(), observation.team_size)
        {
            if previous != observation.team_size {
                fail(&mut report, "team size changed across repeated measures");
            }
        }

        if observation.task_family_id.trim().is_empty() {
            fail(&mut report, "task family identifier must be non-empty");
        }
        match observation.stage {
            CapabilityStage::IndependentBaseline => {
                if observation.task_family_id != contract.baseline_task_family_id {
                    fail(&mut report, "baseline observation uses the wrong task family");
                }
            }
            CapabilityStage::AssistedCollaboration => {
                if observation.task_family_id != contract.assisted_task_family_id {
                    fail(&mut report, "assisted observation uses the wrong task family");
                }
                if observation.ai_used && !observation.ai_available {
                    fail(&mut report, "AI was used when assistance was unavailable");
                }
            }
            CapabilityStage::FarTransfer => {
                if observation.task_family_id != contract.far_transfer_task_family_id {
                    fail(&mut report, "far-transfer observation uses the wrong task family");
                }
            }
            CapabilityStage::ImmediateAiOff
            | CapabilityStage::DelayedAiOff
            | CapabilityStage::Resilience => {}
        }

        if observation.stage == CapabilityStage::IndependentBaseline
            && (observation.ai_available
                || observation.ai_used
                || observation.ai_artifact_reachable)
        {
            fail(&mut report, "independent baseline has candidate-accessible AI");
        }

        if observation.stage.is_ai_off() {
            if observation.ai_available
                || observation.ai_used
                || observation.ai_artifact_reachable
            {
                fail(
                    &mut report,
                    "AI-off stage has AI service, AI use, or candidate-reachable AI artifact",
                );
            }
            match observation.ai_off_qualification {
                Some(QualificationStatus::Pass) => {
                    match observation.ai_off_receipt_digest.as_deref() {
                        Some(digest) if is_sha256(digest) => {}
                        Some(_) => fail(&mut report, "AI-off receipt digest is malformed"),
                        None => fail(&mut report, "AI-off PASS has no receipt digest"),
                    }
                }
                Some(QualificationStatus::Fail) => {
                    fail(&mut report, "independent AI-off qualification failed")
                }
                Some(QualificationStatus::Indeterminate) | None => {
                    indeterminate(&mut report, "AI-off qualification is missing or indeterminate")
                }
            }
        } else if observation.ai_off_qualification.is_some()
            || observation.ai_off_receipt_digest.is_some()
        {
            fail(
                &mut report,
                "AI-off receipt attached to a non-withdrawal stage",
            );
        }

        match observation.accommodation_state {
            AccommodationState::NotRequired | AccommodationState::Provided => {}
            AccommodationState::Withheld => {
                fail(&mut report, "required accessibility accommodation was withheld")
            }
            AccommodationState::Unknown => {
                indeterminate(&mut report, "accessibility accommodation state is unknown")
            }
        }

        if matches!(
            observation.stage,
            CapabilityStage::DelayedAiOff | CapabilityStage::FarTransfer
        ) && observation.elapsed_days_since_collaboration < contract.minimum_delay_days
        {
            fail(
                &mut report,
                "delayed or far-transfer observation occurred before the preregistered interval",
            );
        }

        if observation.stage == CapabilityStage::Resilience
            && observation
                .perturbation_id
                .as_deref()
                .is_none_or(|value| value.trim().is_empty())
        {
            indeterminate(&mut report, "resilience perturbation is not identified");
        }

        if observation.voluntary_stop {
            if observation.measurement.is_some() {
                fail(
                    &mut report,
                    "voluntary stop must not be converted into a scored observation",
                );
            }
            indeterminate(
                &mut report,
                "team voluntarily paused or stopped; outcome remains missing, not a failure",
            );
        } else {
            match &observation.measurement {
                Some(measurement) if valid_measurement(measurement) => {}
                Some(_) => fail(
                    &mut report,
                    "normalized outcome measurements must be finite and within [0, 1]",
                ),
                None => indeterminate(&mut report, "required outcome measurement is missing"),
            }
        }

        if panel
            .insert(
                (observation.team_id.as_str(), observation.stage),
                observation,
            )
            .is_some()
        {
            fail(&mut report, "duplicate team-stage observation");
        }
    }

    for team_id in &contract.expected_team_ids {
        for stage in CapabilityStage::ALL {
            if !panel.contains_key(&(team_id.as_str(), stage)) {
                indeterminate(
                    &mut report,
                    format!("team {team_id} is missing required stage {stage:?}"),
                );
            }
        }
    }

    report
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(character: char) -> String {
        std::iter::repeat_n(character, 64).collect()
    }

    fn contract() -> CapabilityStudyContract {
        CapabilityStudyContract {
            run_id: "capability-run-001".into(),
            evaluator_digest: digest('a'),
            baseline_task_family_id: "baseline-diagnosis".into(),
            assisted_task_family_id: "assisted-diagnosis".into(),
            far_transfer_task_family_id: "unseen-engineering".into(),
            minimum_delay_days: 7,
            expected_team_ids: vec!["team-001".into()],
        }
    }

    fn measurement() -> CapabilityMeasurement {
        CapabilityMeasurement {
            decision_quality: 0.75,
            unique_information_preservation: 0.80,
            verified_evidence_coverage: 0.90,
            agency: 0.85,
            burden: 0.25,
            calibration: 0.80,
        }
    }

    fn panel(contract: &CapabilityStudyContract) -> Vec<CapabilityOutcomeObservation> {
        CapabilityStage::ALL
            .into_iter()
            .map(|stage| {
                let task_family_id = match stage {
                    CapabilityStage::IndependentBaseline => {
                        contract.baseline_task_family_id.clone()
                    }
                    CapabilityStage::AssistedCollaboration
                    | CapabilityStage::ImmediateAiOff
                    | CapabilityStage::DelayedAiOff => contract.assisted_task_family_id.clone(),
                    CapabilityStage::FarTransfer => contract.far_transfer_task_family_id.clone(),
                    CapabilityStage::Resilience => "resilience-diagnostics".into(),
                };
                let ai_off = stage.is_ai_off();
                CapabilityOutcomeObservation {
                    run_id: contract.run_id.clone(),
                    team_id: "team-001".into(),
                    team_roster_digest: digest('b'),
                    team_size: 4,
                    stage,
                    task_family_id,
                    task_digest: digest('c'),
                    evaluator_digest: contract.evaluator_digest.clone(),
                    ai_available: stage == CapabilityStage::AssistedCollaboration,
                    ai_used: stage == CapabilityStage::AssistedCollaboration,
                    ai_artifact_reachable: false,
                    accommodation_state: AccommodationState::NotRequired,
                    voluntary_stop: false,
                    measurement: Some(measurement()),
                    ai_off_qualification: ai_off.then_some(QualificationStatus::Pass),
                    ai_off_receipt_digest: ai_off.then(|| digest('d')),
                    elapsed_days_since_collaboration: if matches!(
                        stage,
                        CapabilityStage::DelayedAiOff | CapabilityStage::FarTransfer
                    ) {
                        7
                    } else {
                        0
                    },
                    perturbation_id: (stage == CapabilityStage::Resilience)
                        .then(|| "network-loss-30s".into()),
                }
            })
            .collect()
    }

    #[test]
    fn complete_qualified_panel_passes_structural_contract() {
        let contract = contract();
        let observations = panel(&contract);
        let report = qualify_capability_outcomes(&contract, &observations);
        assert_eq!(report.status, QualificationStatus::Pass);
        assert!(report.reasons.is_empty());
    }

    #[test]
    fn missing_required_stage_is_indeterminate() {
        let contract = contract();
        let mut observations = panel(&contract);
        observations.retain(|item| item.stage != CapabilityStage::DelayedAiOff);
        let report = qualify_capability_outcomes(&contract, &observations);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn candidate_reachable_ai_artifact_fails_ai_off_stage() {
        let contract = contract();
        let mut observations = panel(&contract);
        let item = observations
            .iter_mut()
            .find(|item| item.stage == CapabilityStage::ImmediateAiOff)
            .unwrap();
        item.ai_artifact_reachable = true;
        let report = qualify_capability_outcomes(&contract, &observations);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn ai_off_pass_without_receipt_digest_fails() {
        let contract = contract();
        let mut observations = panel(&contract);
        let item = observations
            .iter_mut()
            .find(|item| item.stage == CapabilityStage::ImmediateAiOff)
            .unwrap();
        item.ai_off_receipt_digest = None;
        let report = qualify_capability_outcomes(&contract, &observations);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn indeterminate_ai_off_receipt_never_passes() {
        let contract = contract();
        let mut observations = panel(&contract);
        let item = observations
            .iter_mut()
            .find(|item| item.stage == CapabilityStage::DelayedAiOff)
            .unwrap();
        item.ai_off_qualification = Some(QualificationStatus::Indeterminate);
        let report = qualify_capability_outcomes(&contract, &observations);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn withheld_accessibility_support_fails() {
        let contract = contract();
        let mut observations = panel(&contract);
        observations[0].accommodation_state = AccommodationState::Withheld;
        let report = qualify_capability_outcomes(&contract, &observations);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn unknown_accessibility_state_is_indeterminate() {
        let contract = contract();
        let mut observations = panel(&contract);
        observations[0].accommodation_state = AccommodationState::Unknown;
        let report = qualify_capability_outcomes(&contract, &observations);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn voluntary_stop_is_missing_not_zero_or_failure() {
        let contract = contract();
        let mut observations = panel(&contract);
        let item = observations
            .iter_mut()
            .find(|item| item.stage == CapabilityStage::FarTransfer)
            .unwrap();
        item.voluntary_stop = true;
        item.measurement = None;
        let report = qualify_capability_outcomes(&contract, &observations);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
        assert!(!report
            .reasons
            .iter()
            .any(|reason| reason.contains("voluntary stop") && reason.contains("FAIL")));
    }

    #[test]
    fn early_delayed_measurement_fails_preregistration() {
        let contract = contract();
        let mut observations = panel(&contract);
        let item = observations
            .iter_mut()
            .find(|item| item.stage == CapabilityStage::DelayedAiOff)
            .unwrap();
        item.elapsed_days_since_collaboration = 2;
        let report = qualify_capability_outcomes(&contract, &observations);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn changed_team_roster_fails_repeated_measures() {
        let contract = contract();
        let mut observations = panel(&contract);
        observations[4].team_roster_digest = digest('e');
        let report = qualify_capability_outcomes(&contract, &observations);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn invalid_normalized_measurement_fails() {
        let contract = contract();
        let mut observations = panel(&contract);
        observations[0].measurement.as_mut().unwrap().agency = 1.5;
        let report = qualify_capability_outcomes(&contract, &observations);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn duplicate_team_stage_fails() {
        let contract = contract();
        let mut observations = panel(&contract);
        observations.push(observations[0].clone());
        let report = qualify_capability_outcomes(&contract, &observations);
        assert_eq!(report.status, QualificationStatus::Fail);
    }
}
