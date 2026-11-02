//! Synthetic reference oracle for end-to-end GCPN evidence-chain fault interactions.
//!
//! This crate enumerates a deterministic scenario catalogue and defines expected
//! claim dispositions. It does not call the component packages or prove that they
//! satisfy the oracle. A future adapter must submit actual component outputs to
//! compare_observed before integration claims are permitted.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum FaultAtom {
    ForecastLate,
    ForecastStale,
    ObservationMissing,
    SharedSourceLineage,
    AuthorityMismatch,
    CapMessageIdentityMismatch,
    ReceiverDenominatorMissing,
    DeliveryReceiptIncomplete,
    AccessibilityUnverified,
    ActionReceiptMissing,
    OutcomeUnascertained,
    PostCutoffEvidence,
    HindsightLeakage,
    DuplicateCaseKey,
    ValidContradictoryEvidence,
    ClaimOutOfScope,
    SourceUnavailable,
}

impl FaultAtom {
    pub const ALL: [Self; 17] = [
        Self::ForecastLate,
        Self::ForecastStale,
        Self::ObservationMissing,
        Self::SharedSourceLineage,
        Self::AuthorityMismatch,
        Self::CapMessageIdentityMismatch,
        Self::ReceiverDenominatorMissing,
        Self::DeliveryReceiptIncomplete,
        Self::AccessibilityUnverified,
        Self::ActionReceiptMissing,
        Self::OutcomeUnascertained,
        Self::PostCutoffEvidence,
        Self::HindsightLeakage,
        Self::DuplicateCaseKey,
        Self::ValidContradictoryEvidence,
        Self::ClaimOutOfScope,
        Self::SourceUnavailable,
    ];

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::ForecastLate => "forecast_late",
            Self::ForecastStale => "forecast_stale",
            Self::ObservationMissing => "observation_missing",
            Self::SharedSourceLineage => "shared_source_lineage",
            Self::AuthorityMismatch => "authority_mismatch",
            Self::CapMessageIdentityMismatch => "cap_message_identity_mismatch",
            Self::ReceiverDenominatorMissing => "receiver_denominator_missing",
            Self::DeliveryReceiptIncomplete => "delivery_receipt_incomplete",
            Self::AccessibilityUnverified => "accessibility_unverified",
            Self::ActionReceiptMissing => "action_receipt_missing",
            Self::OutcomeUnascertained => "outcome_unascertained",
            Self::PostCutoffEvidence => "post_cutoff_evidence",
            Self::HindsightLeakage => "hindsight_leakage",
            Self::DuplicateCaseKey => "duplicate_case_key",
            Self::ValidContradictoryEvidence => "valid_contradictory_evidence",
            Self::ClaimOutOfScope => "claim_out_of_scope",
            Self::SourceUnavailable => "source_unavailable",
        }
    }

    pub const fn stage(self) -> &'static str {
        match self {
            Self::ForecastLate | Self::ForecastStale => "forecast",
            Self::ObservationMissing | Self::SharedSourceLineage => "reference",
            Self::AuthorityMismatch => "authority",
            Self::CapMessageIdentityMismatch => "warning_lifecycle",
            Self::ReceiverDenominatorMissing
            | Self::DeliveryReceiptIncomplete
            | Self::AccessibilityUnverified => "receiver_reach",
            Self::ActionReceiptMissing => "action",
            Self::OutcomeUnascertained => "outcome",
            Self::PostCutoffEvidence | Self::HindsightLeakage => "temporal_integrity",
            Self::DuplicateCaseKey => "case_frame",
            Self::ValidContradictoryEvidence | Self::ClaimOutOfScope | Self::SourceUnavailable => {
                "claim_control"
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum Disposition {
    Supports,
    Refutes,
    Indeterminate,
    OutOfScope,
    Unavailable,
}

impl Disposition {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Supports => "supports",
            Self::Refutes => "refutes",
            Self::Indeterminate => "indeterminate",
            Self::OutOfScope => "out_of_scope",
            Self::Unavailable => "unavailable",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ScenarioError {
    InvalidId(String),
    DuplicateFault(FaultAtom),
}

impl fmt::Display for ScenarioError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidId(id) => write!(
                formatter,
                "scenario ID must be 1-96 lowercase ASCII letters, digits, '-' or '_': {id:?}"
            ),
            Self::DuplicateFault(fault) => {
                write!(formatter, "duplicate fault atom: {}", fault.as_str())
            }
        }
    }
}

impl std::error::Error for ScenarioError {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Scenario {
    id: String,
    faults: Vec<FaultAtom>,
}

impl Scenario {
    pub fn new(
        id: impl Into<String>,
        faults: impl IntoIterator<Item = FaultAtom>,
    ) -> Result<Self, ScenarioError> {
        let id = id.into();
        let valid_id = !id.is_empty()
            && id.len() <= 96
            && id.bytes().all(|byte| {
                byte.is_ascii_lowercase()
                    || byte.is_ascii_digit()
                    || byte == b'-'
                    || byte == b'_'
            });

        if !valid_id {
            return Err(ScenarioError::InvalidId(id));
        }

        let mut unique_faults = BTreeSet::new();
        for fault in faults {
            if !unique_faults.insert(fault) {
                return Err(ScenarioError::DuplicateFault(fault));
            }
        }

        Ok(Self {
            id,
            faults: unique_faults.into_iter().collect(),
        })
    }

    pub fn id(&self) -> &str {
        &self.id
    }

    pub fn faults(&self) -> &[FaultAtom] {
        &self.faults
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Assessment {
    pub scenario_id: String,
    pub faults: Vec<FaultAtom>,
    pub expected: Disposition,
    pub independent_source_families: usize,
    pub support_claim_permitted: bool,
}

pub fn evaluate_reference(scenario: &Scenario) -> Assessment {
    let expected = match scenario.faults.as_slice() {
        [] => Disposition::Supports,
        [FaultAtom::ValidContradictoryEvidence] => Disposition::Refutes,
        [FaultAtom::ClaimOutOfScope] => Disposition::OutOfScope,
        [FaultAtom::SourceUnavailable] => Disposition::Unavailable,
        _ => Disposition::Indeterminate,
    };

    // The synthetic clean case starts with two independent source families.
    // Reusing a common source lineage collapses corroboration to one family.
    let independent_source_families = if scenario
        .faults
        .contains(&FaultAtom::SharedSourceLineage)
    {
        1
    } else {
        2
    };

    Assessment {
        scenario_id: scenario.id.clone(),
        faults: scenario.faults.clone(),
        expected,
        independent_source_families,
        support_claim_permitted: expected == Disposition::Supports,
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OracleComparison {
    Match,
    Overclaim,
    Underclaim,
    WrongDisposition,
}

pub fn compare_observed(expected: Disposition, observed: Disposition) -> OracleComparison {
    if expected == observed {
        OracleComparison::Match
    } else if observed == Disposition::Supports {
        OracleComparison::Overclaim
    } else if expected == Disposition::Supports {
        OracleComparison::Underclaim
    } else {
        OracleComparison::WrongDisposition
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BenchmarkReport {
    pub assessments: Vec<Assessment>,
    pub disposition_counts: BTreeMap<Disposition, usize>,
}

impl BenchmarkReport {
    pub fn from_scenarios(scenarios: &[Scenario]) -> Self {
        let assessments: Vec<_> = scenarios.iter().map(evaluate_reference).collect();
        let mut disposition_counts = BTreeMap::new();

        for assessment in &assessments {
            *disposition_counts.entry(assessment.expected).or_insert(0) += 1;
        }

        Self {
            assessments,
            disposition_counts,
        }
    }

    pub fn count(&self, disposition: Disposition) -> usize {
        self.disposition_counts
            .get(&disposition)
            .copied()
            .unwrap_or(0)
    }

    pub fn summary_line(&self) -> String {
        format!(
            "scenarios={} supports={} refutes={} indeterminate={} out_of_scope={} unavailable={}",
            self.assessments.len(),
            self.count(Disposition::Supports),
            self.count(Disposition::Refutes),
            self.count(Disposition::Indeterminate),
            self.count(Disposition::OutOfScope),
            self.count(Disposition::Unavailable),
        )
    }

    pub fn to_csv(&self) -> String {
        let mut output = String::from(
            "scenario_id,fault_count,fault_stages,faults,expected_disposition,independent_source_families,support_claim_permitted\n",
        );

        for assessment in &self.assessments {
            let stages = assessment
                .faults
                .iter()
                .map(|fault| fault.stage())
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect::<Vec<_>>()
                .join(";");

            let faults = assessment
                .faults
                .iter()
                .map(|fault| fault.as_str())
                .collect::<Vec<_>>()
                .join(";");

            output.push_str(&format!(
                "{},{},{},{},{},{},{}\n",
                assessment.scenario_id,
                assessment.faults.len(),
                stages,
                faults,
                assessment.expected.as_str(),
                assessment.independent_source_families,
                assessment.support_claim_permitted,
            ));
        }

        output
    }
}

fn targeted_triples() -> [[FaultAtom; 3]; 6] {
    [
        [
            FaultAtom::SharedSourceLineage,
            FaultAtom::PostCutoffEvidence,
            FaultAtom::OutcomeUnascertained,
        ],
        [
            FaultAtom::AuthorityMismatch,
            FaultAtom::CapMessageIdentityMismatch,
            FaultAtom::DeliveryReceiptIncomplete,
        ],
        [
            FaultAtom::ReceiverDenominatorMissing,
            FaultAtom::DeliveryReceiptIncomplete,
            FaultAtom::AccessibilityUnverified,
        ],
        [
            FaultAtom::ForecastLate,
            FaultAtom::ObservationMissing,
            FaultAtom::SourceUnavailable,
        ],
        [
            FaultAtom::HindsightLeakage,
            FaultAtom::PostCutoffEvidence,
            FaultAtom::OutcomeUnascertained,
        ],
        [
            FaultAtom::DuplicateCaseKey,
            FaultAtom::SharedSourceLineage,
            FaultAtom::ValidContradictoryEvidence,
        ],
    ]
}

/// Generate the preregistered catalogue: clean baseline, all 17 single faults,
/// all 136 pairwise combinations, and six targeted three-way interactions.
pub fn standard_catalogue() -> Result<Vec<Scenario>, ScenarioError> {
    let mut scenarios = Vec::with_capacity(160);
    scenarios.push(Scenario::new("baseline-clean", [])?);

    for fault in FaultAtom::ALL {
        scenarios.push(Scenario::new(
            format!("single-{}", fault.as_str()),
            [fault],
        )?);
    }

    for (left_index, left) in FaultAtom::ALL.iter().enumerate() {
        for right in FaultAtom::ALL.iter().skip(left_index + 1) {
            scenarios.push(Scenario::new(
                format!("pair-{}-{}", left.as_str(), right.as_str()),
                [*left, *right],
            )?);
        }
    }

    for (index, faults) in targeted_triples().into_iter().enumerate() {
        scenarios.push(Scenario::new(
            format!(
                "triple-{index}-{}-{}-{}",
                faults[0].as_str(),
                faults[1].as_str(),
                faults[2].as_str()
            ),
            faults,
        )?);
    }

    let unique_ids: BTreeSet<_> = scenarios.iter().map(|scenario| scenario.id()).collect();
    if unique_ids.len() != scenarios.len() {
        return Err(ScenarioError::InvalidId(
            "generated scenario IDs are not unique".to_owned(),
        ));
    }

    Ok(scenarios)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn catalogue_has_expected_single_pair_and_targeted_triple_coverage() {
        let scenarios = standard_catalogue().expect("catalogue should be valid");
        assert_eq!(scenarios.len(), 160);
        assert_eq!(
            scenarios
                .iter()
                .filter(|scenario| scenario.faults().is_empty())
                .count(),
            1
        );
        assert_eq!(
            scenarios
                .iter()
                .filter(|scenario| scenario.faults().len() == 1)
                .count(),
            17
        );
        assert_eq!(
            scenarios
                .iter()
                .filter(|scenario| scenario.faults().len() == 2)
                .count(),
            136
        );
        assert_eq!(
            scenarios
                .iter()
                .filter(|scenario| scenario.faults().len() == 3)
                .count(),
            6
        );
    }

    #[test]
    fn only_the_clean_baseline_can_support_the_claim() {
        let scenarios = standard_catalogue().expect("catalogue should be valid");
        let supporting: Vec<_> = scenarios
            .iter()
            .filter(|scenario| {
                evaluate_reference(scenario).expected == Disposition::Supports
            })
            .collect();

        assert_eq!(supporting.len(), 1);
        assert_eq!(supporting[0].id(), "baseline-clean");
    }

    #[test]
    fn valid_contradiction_is_the_only_refutation_control() {
        let scenario = Scenario::new(
            "valid-refutation",
            [FaultAtom::ValidContradictoryEvidence],
        )
        .expect("scenario should be valid");
        assert_eq!(evaluate_reference(&scenario).expected, Disposition::Refutes);
    }

    #[test]
    fn missing_outcome_is_indeterminate_not_zero_impact_or_refutation() {
        let scenario = Scenario::new(
            "outcome-unascertained",
            [FaultAtom::OutcomeUnascertained],
        )
        .expect("scenario should be valid");
        assert_eq!(
            evaluate_reference(&scenario).expected,
            Disposition::Indeterminate
        );
    }

    #[test]
    fn unavailable_and_out_of_scope_remain_distinct() {
        let unavailable =
            Scenario::new("source-unavailable", [FaultAtom::SourceUnavailable])
                .expect("scenario should be valid");
        let out_of_scope =
            Scenario::new("claim-out-of-scope", [FaultAtom::ClaimOutOfScope])
                .expect("scenario should be valid");

        assert_eq!(
            evaluate_reference(&unavailable).expected,
            Disposition::Unavailable
        );
        assert_eq!(
            evaluate_reference(&out_of_scope).expected,
            Disposition::OutOfScope
        );
    }

    #[test]
    fn control_signals_mixed_with_other_faults_fail_closed() {
        let scenario = Scenario::new(
            "mixed-control-and-missing",
            [
                FaultAtom::ClaimOutOfScope,
                FaultAtom::ObservationMissing,
            ],
        )
        .expect("scenario should be valid");

        assert_eq!(
            evaluate_reference(&scenario).expected,
            Disposition::Indeterminate
        );
    }

    #[test]
    fn common_source_lineage_collapses_independent_corroboration() {
        let scenario = Scenario::new(
            "shared-origin",
            [FaultAtom::SharedSourceLineage],
        )
        .expect("scenario should be valid");

        let assessment = evaluate_reference(&scenario);
        assert_eq!(assessment.independent_source_families, 1);
        assert!(!assessment.support_claim_permitted);
    }

    #[test]
    fn every_pairwise_fault_combination_fails_closed() {
        let scenarios = standard_catalogue().expect("catalogue should be valid");
        let pairwise: Vec<_> = scenarios
            .iter()
            .filter(|scenario| scenario.faults().len() == 2)
            .collect();

        assert_eq!(pairwise.len(), 136);
        assert!(pairwise.iter().all(|scenario| {
            evaluate_reference(scenario).expected != Disposition::Supports
        }));
    }

    #[test]
    fn duplicate_fault_atoms_are_rejected() {
        let result = Scenario::new(
            "duplicate-fault",
            [FaultAtom::ForecastLate, FaultAtom::ForecastLate],
        );

        assert_eq!(
            result,
            Err(ScenarioError::DuplicateFault(FaultAtom::ForecastLate))
        );
    }

    #[test]
    fn invalid_scenario_ids_are_rejected() {
        assert!(matches!(
            Scenario::new("Contains Uppercase", []),
            Err(ScenarioError::InvalidId(_))
        ));
        assert!(matches!(
            Scenario::new("", []),
            Err(ScenarioError::InvalidId(_))
        ));
    }

    #[test]
    fn observed_false_support_is_reported_as_overclaim() {
        assert_eq!(
            compare_observed(Disposition::Indeterminate, Disposition::Supports),
            OracleComparison::Overclaim
        );
    }

    #[test]
    fn observed_failure_to_support_a_clean_case_is_underclaim() {
        assert_eq!(
            compare_observed(Disposition::Supports, Disposition::Indeterminate),
            OracleComparison::Underclaim
        );
    }

    #[test]
    fn deterministic_catalogue_produces_identical_csv() {
        let first = BenchmarkReport::from_scenarios(
            &standard_catalogue().expect("catalogue should be valid"),
        )
        .to_csv();
        let second = BenchmarkReport::from_scenarios(
            &standard_catalogue().expect("catalogue should be valid"),
        )
        .to_csv();

        assert_eq!(first, second);
        assert!(first.starts_with(
            "scenario_id,fault_count,fault_stages,faults,expected_disposition,"
        ));
    }

    #[test]
    fn report_counts_every_generated_scenario_once() {
        let scenarios = standard_catalogue().expect("catalogue should be valid");
        let report = BenchmarkReport::from_scenarios(&scenarios);

        assert_eq!(report.assessments.len(), 160);
        assert_eq!(report.count(Disposition::Supports), 1);
        assert_eq!(
            report.disposition_counts.values().sum::<usize>(),
            scenarios.len()
        );
    }
}
