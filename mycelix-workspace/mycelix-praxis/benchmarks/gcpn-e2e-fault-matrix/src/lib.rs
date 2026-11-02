//! Integrated, synthetic fault-coverage matrix across the GCPN evidence stack.
//!
//! The six-stage and civil-evidence adapters remain independently versioned,
//! but this crate executes their actual public audit paths under one coverage
//! contract. No single composite warning-success score is created.

use std::collections::BTreeSet;

use gcpn_civil_evidence_adapter::{
    execute_faults as execute_civil_faults, Fault as CivilFault,
};
use gcpn_civil_evidence_contracts::{
    CivilEvidenceError, CivilEvidenceReport, CivilViolation,
};
use gcpn_compositional_assurance_adapter::{
    evaluate_pair, evaluate_single, supported_pairs, Fault as EngineFault,
};

const CIVIL_ADDITIONS: [(&str, CivilFault); 5] = [
    ("authority_mismatch", CivilFault::AuthorityRegistryUnknown),
    ("delivery_receipt_incomplete", CivilFault::DeviceDeliveryMissing),
    ("accessibility_unverified", CivilFault::AccessibilityEvidenceMissing),
    ("action_receipt_missing", CivilFault::ActionReceiptMissing),
    ("duplicate_case_key", CivilFault::DuplicateCaseKey),
];

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FaultCoverageRecord {
    pub original_fault_family: String,
    pub adapter_owner: &'static str,
    pub expected_outcome: &'static str,
    pub observed_outcome: String,
    pub detected: bool,
    pub fail_closed: bool,
    pub evidence_detail: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MatrixReport {
    pub original_fault_families: usize,
    pub single_fault_records: Vec<FaultCoverageRecord>,
    pub six_stage_pairwise_cases: usize,
    pub six_stage_pairwise_no_false_support: usize,
    pub civil_pairwise_cases: usize,
    pub civil_pairwise_detected: usize,
    pub civil_pairwise_fail_closed: usize,
    pub civil_pairwise_no_false_support: usize,
}

impl MatrixReport {
    pub fn all_single_faults_detected(&self) -> bool {
        self.single_fault_records.iter().all(|record| record.detected)
    }

    pub fn all_pairwise_faults_detected(&self) -> bool {
        self.six_stage_pairwise_cases == self.six_stage_pairwise_no_false_support
            && self.civil_pairwise_cases == self.civil_pairwise_detected
            && self.civil_pairwise_cases == self.civil_pairwise_no_false_support
    }

    pub fn to_csv(&self) -> String {
        let mut output = String::from(
            "record_type,fault_family,adapter_owner,expected_outcome,observed_outcome,detected,fail_closed,evidence_detail\n",
        );
        for record in &self.single_fault_records {
            output.push_str(&format!(
                "single,{},{},{},{},{},{},{}\n",
                record.original_fault_family,
                record.adapter_owner,
                record.expected_outcome,
                record.observed_outcome,
                record.detected,
                record.fail_closed,
                record.evidence_detail
            ));
        }
        output.push_str(&format!(
            "summary,all_original_fault_families,integrated,17,{}, {},false,engine_pairs={};engine_no_false_support={};civil_pairs={};civil_detected={};civil_fail_closed={};civil_no_false_support={}\n",
            self.original_fault_families,
            self.all_single_faults_detected(),
            self.six_stage_pairwise_cases,
            self.six_stage_pairwise_no_false_support,
            self.civil_pairwise_cases,
            self.civil_pairwise_detected,
            self.civil_pairwise_fail_closed,
            self.civil_pairwise_no_false_support
        ));
        output
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum MatrixError {
    EngineAdapter(String),
    CivilAdapter(String),
    UnsupportedCivilOutcome(&'static str),
    CoverageCardinalityMismatch {
        expected: usize,
        observed: usize,
    },
}

impl std::fmt::Display for MatrixError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EngineAdapter(message) => write!(formatter, "six-stage adapter failed: {message}"),
            Self::CivilAdapter(message) => write!(formatter, "civil adapter failed: {message}"),
            Self::UnsupportedCivilOutcome(fault) => {
                write!(formatter, "civil fault is not mapped to a check: {fault}")
            }
            Self::CoverageCardinalityMismatch { expected, observed } => write!(
                formatter,
                "fault coverage cardinality mismatch: expected {expected}, observed {observed}"
            ),
        }
    }
}

impl std::error::Error for MatrixError {}

pub fn run_single_fault_matrix() -> Result<Vec<FaultCoverageRecord>, MatrixError> {
    let mut records = Vec::with_capacity(17);

    for fault in EngineFault::SUPPORTED {
        let observed = evaluate_single(fault)
            .map_err(|error| MatrixError::EngineAdapter(error.to_string()))?;
        records.push(FaultCoverageRecord {
            original_fault_family: fault.label().into(),
            adapter_owner: "six_stage_compositional",
            expected_outcome: disposition_label(observed.expected),
            observed_outcome: disposition_label(observed.observed).into(),
            detected: observed.expected == observed.observed && observed.violation_detected,
            fail_closed: false,
            evidence_detail: format!(
                "claim={}:{};expected_violation={};violation_detected={};fingerprint={}",
                observed.stage.label(),
                observed.claim_id,
                observed
                    .expected_violation
                    .map(|violation| violation.label())
                    .unwrap_or("none"),
                observed.violation_detected,
                observed.report_fingerprint
            ),
        });
    }

    for (original_family, civil_fault) in CIVIL_ADDITIONS {
        let record = evaluate_civil_single(original_family, civil_fault)?;
        records.push(record);
    }

    records.sort_by(|left, right| {
        left.original_fault_family
            .cmp(&right.original_fault_family)
    });
    let unique: BTreeSet<_> = records
        .iter()
        .map(|record| record.original_fault_family.as_str())
        .collect();
    if records.len() != 17 || unique.len() != 17 {
        return Err(MatrixError::CoverageCardinalityMismatch {
            expected: 17,
            observed: unique.len(),
        });
    }
    Ok(records)
}

fn evaluate_civil_single(
    original_family: &str,
    fault: CivilFault,
) -> Result<FaultCoverageRecord, MatrixError> {
    match execute_civil_faults(&[fault]) {
        Ok(report) => {
            let (outcome, detected, detail) = civil_single_assessment(fault, &report)?;
            Ok(FaultCoverageRecord {
                original_fault_family: original_family.into(),
                adapter_owner: "civil_evidence_contracts",
                expected_outcome: "non_supporting",
                observed_outcome: outcome,
                detected,
                fail_closed: false,
                evidence_detail: detail,
            })
        }
        Err(error) => {
            let fail_closed = matches!(
                (fault, &error),
                (
                    CivilFault::DuplicateCaseKey,
                    gcpn_civil_evidence_adapter::AdapterError::CivilEvidence(
                        CivilEvidenceError::DuplicateCaseKey(_)
                    )
                )
            );
            if !fail_closed {
                return Err(MatrixError::CivilAdapter(error.to_string()));
            }
            Ok(FaultCoverageRecord {
                original_fault_family: original_family.into(),
                adapter_owner: "civil_evidence_contracts",
                expected_outcome: "fail_closed",
                observed_outcome: "fail_closed_duplicate_case_key".into(),
                detected: true,
                fail_closed: true,
                evidence_detail: "duplicate frame identity rejected before an outcome claim is emitted".into(),
            })
        }
    }
}

fn civil_single_assessment(
    fault: CivilFault,
    report: &CivilEvidenceReport,
) -> Result<(String, bool, String), MatrixError> {
    match fault {
        CivilFault::AuthorityRegistryUnknown => {
            let observed = disposition_label(report.authority.disposition);
            let detected = report.authority.disposition
                != gcpn_compositional_assurance_adapter::baseline_scenario()
                    .audit()
                    .map_err(|error| MatrixError::EngineAdapter(error.to_string()))?
                    .claims
                    .first()
                    .map(|_| gcpn_compositional_assurance::ClaimDisposition::Supports)
                    .unwrap_or(gcpn_compositional_assurance::ClaimDisposition::Supports)
                && report
                    .authority
                    .violations
                    .contains(&CivilViolation::AuthorityRegistryUnknown);
            Ok((
                observed.into(),
                detected,
                format!("authority_violations={:?}", report.authority.violations),
            ))
        }
        CivilFault::DeviceDeliveryMissing => {
            let delivery = report.delivery.first().ok_or(
                MatrixError::UnsupportedCivilOutcome("delivery assessment missing"),
            )?;
            let detected = delivery.disposition
                != gcpn_compositional_assurance::ClaimDisposition::Supports
                && delivery.counts.device_delivered == 0
                && delivery.counts.provider_accepted > 0;
            Ok((
                disposition_label(delivery.disposition).into(),
                detected,
                format!(
                    "provider_accepted={};device_delivered={};not_confirmed={}",
                    delivery.counts.provider_accepted,
                    delivery.counts.device_delivered,
                    delivery.counts.not_confirmed_device_delivered
                ),
            ))
        }
        CivilFault::AccessibilityEvidenceMissing => {
            let assessment = report.accessibility.first().ok_or(
                MatrixError::UnsupportedCivilOutcome("accessibility assessment missing"),
            )?;
            let comprehension = report.comprehension.first().ok_or(
                MatrixError::UnsupportedCivilOutcome("comprehension assessment missing"),
            )?;
            let detected = assessment.disposition
                == gcpn_compositional_assurance::ClaimDisposition::Unavailable
                && assessment.state.is_none()
                && comprehension.disposition
                    != gcpn_compositional_assurance::ClaimDisposition::Supports;
            Ok((
                disposition_label(assessment.disposition).into(),
                detected,
                format!(
                    "accessibility={};comprehension={}",
                    disposition_label(assessment.disposition),
                    disposition_label(comprehension.disposition)
                ),
            ))
        }
        CivilFault::ActionReceiptMissing => {
            let action = report.actions.first().ok_or(
                MatrixError::UnsupportedCivilOutcome("action assessment missing"),
            )?;
            let detected = action.disposition
                == gcpn_compositional_assurance::ClaimDisposition::Unavailable
                && action.latest_state.is_none();
            Ok((
                disposition_label(action.disposition).into(),
                detected,
                format!(
                    "latest_state={};receipt_count={}",
                    action.latest_state.map(|state| state.label()).unwrap_or("missing"),
                    action.receipt_count
                ),
            ))
        }
        CivilFault::DuplicateCaseKey => Err(MatrixError::UnsupportedCivilOutcome(
            "duplicate case-key must fail closed",
        )),
        _ => Err(MatrixError::UnsupportedCivilOutcome(
            "fault is not one of the five original missing families",
        )),
    }
}

pub fn run_six_stage_pairwise_matrix() -> Result<(usize, usize), MatrixError> {
    let pairs = supported_pairs();
    let mut detected = 0usize;
    for pair in &pairs {
        let observation = evaluate_pair(*pair)
            .map_err(|error| MatrixError::EngineAdapter(error.to_string()))?;
        if observation.no_false_support {
            detected += 1;
        }
    }
    Ok((pairs.len(), detected))
}

pub fn run_civil_pairwise_matrix() -> Result<(usize, usize, usize), MatrixError> {
    let faults = CivilFault::ALL;
    let mut cases = 0usize;
    let mut detected_cases = 0usize;
    let mut fail_closed_cases = 0usize;
    let mut no_false_support_cases = 0usize;

    for left_index in 0..faults.len() {
        for right in faults.iter().skip(left_index + 1) {
            let left = faults[left_index];
            cases += 1;
            match execute_civil_faults(&[left, *right]) {
                Err(error) => {
                    let duplicate_case = left == CivilFault::DuplicateCaseKey
                        || *right == CivilFault::DuplicateCaseKey;
                    let correctly_failed_closed = duplicate_case
                        && matches!(
                            error,
                            gcpn_civil_evidence_adapter::AdapterError::CivilEvidence(
                                CivilEvidenceError::DuplicateCaseKey(_)
                            )
                        );
                    if correctly_failed_closed {
                        detected_cases += 1;
                        fail_closed_cases += 1;
                        no_false_support_cases += 1;
                    } else {
                        return Err(MatrixError::CivilAdapter(error.to_string()));
                    }
                }
                Ok(report) => {
                    let left_detected = civil_fault_not_supporting(left, &report)?;
                    let right_detected = civil_fault_not_supporting(*right, &report)?;
                    if left_detected && right_detected {
                        detected_cases += 1;
                        no_false_support_cases += 1;
                    }
                }
            }
        }
    }

    Ok((cases, detected_cases, fail_closed_cases.min(cases)))
}

fn civil_fault_not_supporting(
    fault: CivilFault,
    report: &CivilEvidenceReport,
) -> Result<bool, MatrixError> {
    use gcpn_compositional_assurance::ClaimDisposition::Supports;
    let not_supports = |disposition| disposition != Supports;
    match fault {
        CivilFault::AuthorityRegistryUnknown => Ok(
            not_supports(report.authority.disposition)
                && report.authority.violations.contains(&CivilViolation::AuthorityRegistryUnknown),
        ),
        CivilFault::DeviceDeliveryMissing => report
            .delivery
            .first()
            .map(|assessment| {
                not_supports(assessment.disposition)
                    && assessment.counts.device_delivered == 0
            })
            .ok_or(MatrixError::UnsupportedCivilOutcome("delivery assessment missing")),
        CivilFault::AccessibilityEvidenceMissing => report
            .accessibility
            .first()
            .map(|assessment| {
                not_supports(assessment.disposition)
                    && report.comprehension.first().is_some_and(|comprehension| {
                        not_supports(comprehension.disposition)
                    })
            })
            .ok_or(MatrixError::UnsupportedCivilOutcome("accessibility assessment missing")),
        CivilFault::ActionReceiptMissing => report
            .actions
            .first()
            .map(|assessment| {
                assessment.disposition
                    == gcpn_compositional_assurance::ClaimDisposition::Unavailable
                    && assessment.latest_state.is_none()
            })
            .ok_or(MatrixError::UnsupportedCivilOutcome("action assessment missing")),
        CivilFault::DuplicateCaseKey => Err(MatrixError::UnsupportedCivilOutcome(
            "duplicate case-key must fail closed",
        )),
        CivilFault::WrongAccessibilityDeliveryBinding => report
            .accessibility
            .first()
            .map(|assessment| not_supports(assessment.disposition))
            .ok_or(MatrixError::UnsupportedCivilOutcome("accessibility assessment missing")),
        CivilFault::ComprehensionAccessibilityBindingMismatch => report
            .comprehension
            .first()
            .map(|assessment| not_supports(assessment.disposition))
            .ok_or(MatrixError::UnsupportedCivilOutcome("comprehension assessment missing")),
        CivilFault::CapLifecycleUnqualified => report
            .delivery
            .first()
            .map(|assessment| not_supports(assessment.disposition))
            .ok_or(MatrixError::UnsupportedCivilOutcome("delivery assessment missing")),
        CivilFault::DecisionUnqualified => report
            .actions
            .first()
            .map(|assessment| not_supports(assessment.disposition))
            .ok_or(MatrixError::UnsupportedCivilOutcome("action assessment missing")),
        CivilFault::PostCutoffDeviceReceipt => report
            .delivery
            .first()
            .map(|assessment| {
                not_supports(assessment.disposition)
                    && assessment.counts.device_delivered == 0
                    && assessment
                        .violations
                        .contains(&CivilViolation::DeliveryReceiptAfterCutoff)
            })
            .ok_or(MatrixError::UnsupportedCivilOutcome("delivery assessment missing")),
    }
}

pub fn run_full_matrix() -> Result<MatrixReport, MatrixError> {
    let single_fault_records = run_single_fault_matrix()?;
    let (six_stage_pairwise_cases, six_stage_pairwise_no_false_support) =
        run_six_stage_pairwise_matrix()?;
    let (
        civil_pairwise_cases,
        civil_pairwise_detected,
        civil_pairwise_fail_closed,
    ) = run_civil_pairwise_matrix()?;

    Ok(MatrixReport {
        original_fault_families: single_fault_records.len(),
        single_fault_records,
        six_stage_pairwise_cases,
        six_stage_pairwise_no_false_support,
        civil_pairwise_cases,
        civil_pairwise_detected,
        civil_pairwise_fail_closed,
        civil_pairwise_no_false_support: civil_pairwise_detected,
    })
}

fn disposition_label(
    disposition: gcpn_compositional_assurance::ClaimDisposition,
) -> &'static str {
    match disposition {
        gcpn_compositional_assurance::ClaimDisposition::Supports => "supports",
        gcpn_compositional_assurance::ClaimDisposition::Refutes => "refutes",
        gcpn_compositional_assurance::ClaimDisposition::Indeterminate => "indeterminate",
        gcpn_compositional_assurance::ClaimDisposition::OutOfScope => "out_of_scope",
        gcpn_compositional_assurance::ClaimDisposition::Unavailable => "unavailable",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn original_seventeen_fault_families_have_exactly_one_owner() {
        let records = run_single_fault_matrix().unwrap();
        assert_eq!(records.len(), 17);
        let names: BTreeSet<_> = records
            .iter()
            .map(|record| record.original_fault_family.as_str())
            .collect();
        assert_eq!(names.len(), 17);
        assert_eq!(
            records
                .iter()
                .filter(|record| record.adapter_owner == "six_stage_compositional")
                .count(),
            12
        );
        assert_eq!(
            records
                .iter()
                .filter(|record| record.adapter_owner == "civil_evidence_contracts")
                .count(),
            5
        );
    }

    #[test]
    fn every_original_single_fault_is_detected_or_fails_closed() {
        let records = run_single_fault_matrix().unwrap();
        assert!(records.iter().all(|record| record.detected));
        assert_eq!(
            records.iter().filter(|record| record.fail_closed).count(),
            1
        );
    }

    #[test]
    fn six_stage_all_sixty_six_supported_pairs_have_no_false_support() {
        let (cases, no_false_support) = run_six_stage_pairwise_matrix().unwrap();
        assert_eq!(cases, 66);
        assert_eq!(no_false_support, 66);
    }

    #[test]
    fn civil_contract_all_forty_five_pairs_are_detected_or_fail_closed() {
        let (cases, detected, fail_closed) = run_civil_pairwise_matrix().unwrap();
        assert_eq!(cases, 45);
        assert_eq!(detected, 45);
        assert_eq!(fail_closed, 9);
    }

    #[test]
    fn full_report_cardinality_and_pairwise_results_are_reconstructible() {
        let report = run_full_matrix().unwrap();
        assert_eq!(report.original_fault_families, 17);
        assert!(report.all_single_faults_detected());
        assert!(report.all_pairwise_faults_detected());
        assert_eq!(report.six_stage_pairwise_cases, 66);
        assert_eq!(report.civil_pairwise_cases, 45);
    }

    #[test]
    fn integrated_matrix_csv_is_deterministic() {
        let first = run_full_matrix().unwrap().to_csv();
        let second = run_full_matrix().unwrap().to_csv();
        assert_eq!(first, second);
        assert!(first.starts_with(
            "record_type,fault_family,adapter_owner,expected_outcome,observed_outcome,detected,fail_closed,"
        ));
    }

    #[test]
    fn legacy_and_civil_coverage_labels_do_not_overlap() {
        let report = run_single_fault_matrix().unwrap();
        let six_stage: BTreeSet<_> = report
            .iter()
            .filter(|record| record.adapter_owner == "six_stage_compositional")
            .map(|record| record.original_fault_family.as_str())
            .collect();
        let civil: BTreeSet<_> = report
            .iter()
            .filter(|record| record.adapter_owner == "civil_evidence_contracts")
            .map(|record| record.original_fault_family.as_str())
            .collect();
        assert!(six_stage.is_disjoint(&civil));
    }
}
