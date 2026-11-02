//! Executable adapter for the five previously unrepresented GCPN civil faults.
//!
//! This crate mutates real CivilEvidenceProfile inputs and calls
//! audit_civil_evidence(). It does not provide a second oracle that merely
//! mirrors expected labels; its tests assert the actual component result.

use gcpn_civil_evidence_contracts::{
    audit_civil_evidence, AccessibilityEvidence, AccessibilityRequirement, AccessibilityState,
    ActionReceipt, ActionRequirement, ActionState, AuthorityAssertion, AuthorityRequirement,
    BaseClaimBinding, CaseDisposition, CaseFrame, ChannelCohortTarget, CivilEvidenceError,
    CivilEvidenceProfile, ComprehensionEvidence, ComprehensionRequirement, DeliveryReceipt,
    DeliveryState, FrameCase, RegistryAuthorityStatus, SignatureStatus,
};
use gcpn_compositional_assurance::{
    AuditReport, ClaimDisposition, ClaimResult, Stage, ViolationCode,
};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum Fault {
    AuthorityRegistryUnknown,
    DeviceDeliveryMissing,
    AccessibilityEvidenceMissing,
    ActionReceiptMissing,
    DuplicateCaseKey,
    WrongAccessibilityDeliveryBinding,
    ComprehensionAccessibilityBindingMismatch,
    CapLifecycleUnqualified,
    DecisionUnqualified,
    PostCutoffDeviceReceipt,
}

impl Fault {
    pub const ALL: [Self; 10] = [
        Self::AuthorityRegistryUnknown,
        Self::DeviceDeliveryMissing,
        Self::AccessibilityEvidenceMissing,
        Self::ActionReceiptMissing,
        Self::DuplicateCaseKey,
        Self::WrongAccessibilityDeliveryBinding,
        Self::ComprehensionAccessibilityBindingMismatch,
        Self::CapLifecycleUnqualified,
        Self::DecisionUnqualified,
        Self::PostCutoffDeviceReceipt,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::AuthorityRegistryUnknown => "authority_registry_unknown",
            Self::DeviceDeliveryMissing => "device_delivery_missing",
            Self::AccessibilityEvidenceMissing => "accessibility_evidence_missing",
            Self::ActionReceiptMissing => "action_receipt_missing",
            Self::DuplicateCaseKey => "duplicate_case_key",
            Self::WrongAccessibilityDeliveryBinding => "wrong_accessibility_delivery_binding",
            Self::ComprehensionAccessibilityBindingMismatch => {
                "comprehension_accessibility_binding_mismatch"
            }
            Self::CapLifecycleUnqualified => "cap_lifecycle_unqualified",
            Self::DecisionUnqualified => "decision_unqualified",
            Self::PostCutoffDeviceReceipt => "post_cutoff_device_receipt",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AdapterError {
    DuplicateFault(Fault),
    CivilEvidence(CivilEvidenceError),
    MissingExpectedAssessment(&'static str),
}

impl std::fmt::Display for AdapterError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DuplicateFault(fault) => {
                write!(formatter, "duplicate fault atom: {}", fault.label())
            }
            Self::CivilEvidence(error) => write!(formatter, "civil-evidence audit failed: {error}"),
            Self::MissingExpectedAssessment(name) => {
                write!(formatter, "expected assessment is missing: {name}")
            }
        }
    }
}

impl std::error::Error for AdapterError {}

impl From<CivilEvidenceError> for AdapterError {
    fn from(value: CivilEvidenceError) -> Self {
        Self::CivilEvidence(value)
    }
}

pub fn clean_profile() -> CivilEvidenceProfile {
    CivilEvidenceProfile {
        scenario_id: "civil-adapter-baseline".into(),
        as_of_cutoff: 100,
        base_audit: base_audit(),
        required_base_claims: vec![
            BaseClaimBinding {
                stage: Stage::CapLifecycle,
                claim_id: "cap-message-valid".into(),
            },
            BaseClaimBinding {
                stage: Stage::Decision,
                claim_id: "decision-valid".into(),
            },
        ],
        authority_requirement: AuthorityRequirement {
            authority_id: "authority-1".into(),
            jurisdiction_id: "district-1".into(),
            hazard_class_id: "flood".into(),
            cap_originator: "warning-authority".into(),
            cap_message_id: "cap-1".into(),
            cap_version: "1".into(),
            decision_id: "decision-1".into(),
            event_time: 50,
        },
        authority_assertion: Some(AuthorityAssertion {
            assertion_id: "assertion-1".into(),
            authority_id: "authority-1".into(),
            jurisdiction_id: "district-1".into(),
            hazard_class_id: "flood".into(),
            cap_originator: "warning-authority".into(),
            cap_message_id: "cap-1".into(),
            cap_version: "1".into(),
            decision_id: "decision-1".into(),
            valid_from: 1,
            valid_until: 100,
            recorded_at: 20,
            signature_status: SignatureStatus::Verified,
            registry_status: RegistryAuthorityStatus::Recognized,
        }),
        delivery_message_id: "cap-1".into(),
        delivery_message_version: "1".into(),
        delivery_targets: vec![ChannelCohortTarget {
            cohort_id: "cohort-a".into(),
            channel_id: "cell-broadcast".into(),
            intended_receivers: 10,
        }],
        delivery_receipts: vec![
            DeliveryReceipt {
                receipt_id: "provider-1".into(),
                message_id: "cap-1".into(),
                message_version: "1".into(),
                cohort_id: "cohort-a".into(),
                channel_id: "cell-broadcast".into(),
                state: DeliveryState::ProviderAccepted,
                count: 10,
                duplicate_of: None,
                recorded_at: 30,
            },
            DeliveryReceipt {
                receipt_id: "device-1".into(),
                message_id: "cap-1".into(),
                message_version: "1".into(),
                cohort_id: "cohort-a".into(),
                channel_id: "cell-broadcast".into(),
                state: DeliveryState::DeviceDelivered,
                count: 10,
                duplicate_of: None,
                recorded_at: 35,
            },
        ],
        accessibility_requirements: vec![AccessibilityRequirement {
            cohort_id: "cohort-a".into(),
            language_id: "en".into(),
            format_id: "text".into(),
            assistive_technology_profile_id: "screen-reader".into(),
        }],
        accessibility_evidence: vec![AccessibilityEvidence {
            evidence_id: "accessibility-1".into(),
            delivery_receipt_id: "device-1".into(),
            cohort_id: "cohort-a".into(),
            language_id: "en".into(),
            format_id: "text".into(),
            assistive_technology_profile_id: "screen-reader".into(),
            test_protocol_id: "accessibility-protocol-1".into(),
            state: AccessibilityState::Accessible,
            tested_at: 40,
            recorded_at: 45,
        }],
        comprehension_requirements: vec![ComprehensionRequirement {
            cohort_id: "cohort-a".into(),
            language_id: "en".into(),
            test_protocol_id: "comprehension-protocol-1".into(),
            minimum_comprehension_rate: 0.8,
            minimum_intent_rate: 0.7,
        }],
        comprehension_evidence: vec![ComprehensionEvidence {
            evidence_id: "comprehension-1".into(),
            delivery_receipt_id: "device-1".into(),
            accessibility_evidence_id: "accessibility-1".into(),
            cohort_id: "cohort-a".into(),
            language_id: "en".into(),
            test_protocol_id: "comprehension-protocol-1".into(),
            tested_count: 10,
            understood_count: 9,
            intended_action_count: 8,
            tested_at: 50,
            recorded_at: 55,
        }],
        action_requirements: vec![ActionRequirement {
            action_id: "action-1".into(),
            decision_id: "decision-1".into(),
            authority_assertion_id: "assertion-1".into(),
            executor_agency_id: "response-agency".into(),
            action_type_id: "evacuation".into(),
        }],
        action_receipts: vec![
            action_receipt("action-approved", ActionState::Approved, 25),
            action_receipt("action-dispatched", ActionState::Dispatched, 30),
            action_receipt("action-started", ActionState::Started, 40),
            action_receipt("action-completed", ActionState::Completed, 70),
        ],
        case_frame: CaseFrame {
            frame_id: "case-frame-1".into(),
            inclusion_criteria_id: "eligible-cases-v1".into(),
            declared_eligible_cases: 2,
            cases: vec![
                FrameCase {
                    case_key: "case-1".into(),
                    disposition: CaseDisposition::Observed,
                    recorded_at: 80,
                },
                FrameCase {
                    case_key: "case-2".into(),
                    disposition: CaseDisposition::Observed,
                    recorded_at: 80,
                },
            ],
        },
    }
}

fn action_receipt(receipt_id: &str, state: ActionState, event_time: u64) -> ActionReceipt {
    ActionReceipt {
        receipt_id: receipt_id.into(),
        action_id: "action-1".into(),
        decision_id: "decision-1".into(),
        authority_assertion_id: "assertion-1".into(),
        executor_agency_id: "response-agency".into(),
        action_type_id: "evacuation".into(),
        state,
        event_time,
        recorded_at: event_time,
    }
}

fn base_audit() -> AuditReport {
    AuditReport {
        scenario_id: "compositional-base".into(),
        specification_digest: "gcpn-compositional-assurance-v0-1".into(),
        as_of_cutoff: 100,
        claims: vec![
            ClaimResult {
                stage: Stage::CapLifecycle,
                claim_id: "cap-message-valid".into(),
                disposition: ClaimDisposition::Supports,
                evidence_count: 1,
                independent_supporting_lineages: 1,
                violations: Vec::new(),
            },
            ClaimResult {
                stage: Stage::Decision,
                claim_id: "decision-valid".into(),
                disposition: ClaimDisposition::Supports,
                evidence_count: 1,
                independent_supporting_lineages: 1,
                violations: Vec::new(),
            },
        ],
        violations: Vec::new(),
        reproducibility_fingerprint: "synthetic-base-fingerprint".into(),
    }
}

pub fn apply_faults(faults: &[Fault]) -> Result<CivilEvidenceProfile, AdapterError> {
    let mut profile = clean_profile();
    let mut seen = BTreeSet::new();
    for fault in faults {
        if !seen.insert(*fault) {
            return Err(AdapterError::DuplicateFault(*fault));
        }
        match fault {
            Fault::AuthorityRegistryUnknown => {
                profile.authority_assertion.as_mut().expect("fixture assertion").registry_status =
                    RegistryAuthorityStatus::Unknown;
            }
            Fault::DeviceDeliveryMissing => {
                profile.delivery_receipts.retain(|receipt| {
                    receipt.state != DeliveryState::DeviceDelivered
                });
            }
            Fault::AccessibilityEvidenceMissing => {
                profile.accessibility_evidence.clear();
            }
            Fault::ActionReceiptMissing => {
                profile.action_receipts.clear();
            }
            Fault::DuplicateCaseKey => {
                profile.case_frame.cases.push(profile.case_frame.cases[0].clone());
            }
            Fault::WrongAccessibilityDeliveryBinding => {
                profile.accessibility_evidence[0].delivery_receipt_id = "wrong-device".into();
            }
            Fault::ComprehensionAccessibilityBindingMismatch => {
                profile.comprehension_evidence[0].accessibility_evidence_id =
                    "wrong-accessibility".into();
            }
            Fault::CapLifecycleUnqualified => {
                let claim = profile.base_audit.claims.iter_mut().find(|claim| {
                    claim.stage == Stage::CapLifecycle && claim.claim_id == "cap-message-valid"
                }).expect("CAP claim");
                claim.disposition = ClaimDisposition::Indeterminate;
            }
            Fault::DecisionUnqualified => {
                let claim = profile.base_audit.claims.iter_mut().find(|claim| {
                    claim.stage == Stage::Decision && claim.claim_id == "decision-valid"
                }).expect("decision claim");
                claim.disposition = ClaimDisposition::Indeterminate;
            }
            Fault::PostCutoffDeviceReceipt => {
                let receipt = profile.delivery_receipts.iter_mut().find(|receipt| {
                    receipt.state == DeliveryState::DeviceDelivered
                }).expect("device receipt");
                receipt.recorded_at = 101;
            }
        }
    }
    Ok(profile)
}

pub fn execute_faults(faults: &[Fault]) -> Result<gcpn_civil_evidence_contracts::CivilEvidenceReport, AdapterError> {
    let profile = apply_faults(faults)?;
    audit_civil_evidence(&profile).map_err(AdapterError::CivilEvidence)
}

pub fn base_claim_disposition(
    report: &gcpn_civil_evidence_contracts::CivilEvidenceReport,
) -> ClaimDisposition {
    report.base_audit.disposition
}

#[cfg(test)]
mod tests {
    use super::*;
    use gcpn_civil_evidence_contracts::CivilViolation;

    #[test]
    fn clean_baseline_supports_all_required_civil_evidence_claims() {
        let report = execute_faults(&[]).unwrap();
        assert_eq!(report.base_audit.disposition, ClaimDisposition::Supports);
        assert_eq!(report.authority.disposition, ClaimDisposition::Supports);
        assert_eq!(report.delivery[0].disposition, ClaimDisposition::Supports);
        assert_eq!(report.accessibility[0].disposition, ClaimDisposition::Supports);
        assert_eq!(report.comprehension[0].disposition, ClaimDisposition::Supports);
        assert_eq!(report.actions[0].disposition, ClaimDisposition::Supports);
        assert_eq!(report.case_frame.disposition, ClaimDisposition::Supports);
    }

    #[test]
    fn authority_registry_unknown_is_not_authority_support() {
        let report = execute_faults(&[Fault::AuthorityRegistryUnknown]).unwrap();
        assert_eq!(report.authority.disposition, ClaimDisposition::Indeterminate);
        assert_eq!(report.actions[0].disposition, ClaimDisposition::Indeterminate);
        assert!(report.authority.violations.contains(&CivilViolation::AuthorityRegistryUnknown));
    }

    #[test]
    fn provider_acceptance_without_device_receipt_is_not_delivery() {
        let report = execute_faults(&[Fault::DeviceDeliveryMissing]).unwrap();
        assert_eq!(report.delivery[0].counts.provider_accepted, 10);
        assert_eq!(report.delivery[0].counts.device_delivered, 0);
        assert_eq!(report.delivery[0].disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn missing_accessibility_test_is_not_accessible_presentation() {
        let report = execute_faults(&[Fault::AccessibilityEvidenceMissing]).unwrap();
        assert_eq!(report.accessibility[0].disposition, ClaimDisposition::Unavailable);
        assert_eq!(report.comprehension[0].disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn missing_action_receipt_is_unavailable_not_completed() {
        let report = execute_faults(&[Fault::ActionReceiptMissing]).unwrap();
        assert_eq!(report.actions[0].disposition, ClaimDisposition::Unavailable);
        assert_eq!(report.actions[0].latest_state, None);
    }

    #[test]
    fn duplicate_case_key_fails_closed() {
        let error = execute_faults(&[Fault::DuplicateCaseKey]).unwrap_err();
        assert!(matches!(
            error,
            AdapterError::CivilEvidence(CivilEvidenceError::DuplicateCaseKey(_))
        ));
    }

    #[test]
    fn cap_lifecycle_and_delivery_faults_interact_without_false_support() {
        let report = execute_faults(&[
            Fault::CapLifecycleUnqualified,
            Fault::DeviceDeliveryMissing,
        ]).unwrap();
        assert_eq!(report.delivery[0].disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn authority_and_action_faults_interact_without_false_support() {
        let report = execute_faults(&[
            Fault::AuthorityRegistryUnknown,
            Fault::ActionReceiptMissing,
        ]).unwrap();
        assert_eq!(report.authority.disposition, ClaimDisposition::Indeterminate);
        assert_eq!(report.actions[0].disposition, ClaimDisposition::Unavailable);
    }

    #[test]
    fn accessible_evidence_cannot_survive_missing_device_receipt() {
        let report = execute_faults(&[
            Fault::DeviceDeliveryMissing,
            Fault::WrongAccessibilityDeliveryBinding,
        ]).unwrap();
        assert_eq!(report.accessibility[0].disposition, ClaimDisposition::Indeterminate);
        assert!(report.accessibility[0]
            .violations
            .contains(&CivilViolation::AccessibilityDeliveryBindingMismatch));
    }

    #[test]
    fn comprehension_cannot_survive_wrong_accessibility_parent() {
        let report = execute_faults(&[
            Fault::ComprehensionAccessibilityBindingMismatch,
        ]).unwrap();
        assert_eq!(report.comprehension[0].disposition, ClaimDisposition::Indeterminate);
        assert!(report.comprehension[0]
            .violations
            .contains(&CivilViolation::ComprehensionAccessibilityBindingMismatch));
    }

    #[test]
    fn decision_fault_downgrades_completed_action() {
        let report = execute_faults(&[Fault::DecisionUnqualified]).unwrap();
        assert_eq!(report.actions[0].disposition, ClaimDisposition::Indeterminate);
        assert!(report.actions[0]
            .violations
            .contains(&CivilViolation::ActionDecisionNotSupporting));
    }

    #[test]
    fn post_cutoff_device_receipt_is_not_counted() {
        let report = execute_faults(&[Fault::PostCutoffDeviceReceipt]).unwrap();
        assert_eq!(report.delivery[0].counts.device_delivered, 0);
        assert_eq!(report.delivery[0].disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn duplicate_fault_atom_fails_before_audit() {
        assert!(matches!(
            apply_faults(&[Fault::AuthorityRegistryUnknown, Fault::AuthorityRegistryUnknown]),
            Err(AdapterError::DuplicateFault(Fault::AuthorityRegistryUnknown))
        ));
    }

    #[test]
    fn all_ten_fault_atoms_have_stable_identifiers() {
        assert_eq!(Fault::ALL.len(), 10);
        let labels: BTreeSet<_> = Fault::ALL.iter().map(|fault| fault.label()).collect();
        assert_eq!(labels.len(), 10);
    }

    #[test]
    fn report_does_not_collapse_claims_into_one_score() {
        let report = execute_faults(&[
            Fault::AuthorityRegistryUnknown,
            Fault::DeviceDeliveryMissing,
            Fault::ActionReceiptMissing,
        ]).unwrap();
        assert_eq!(report.authority.disposition, ClaimDisposition::Indeterminate);
        assert_eq!(report.delivery[0].disposition, ClaimDisposition::Indeterminate);
        assert_eq!(report.actions[0].disposition, ClaimDisposition::Unavailable);
        let csv = report.to_csv();
        assert!(csv.contains("authority"));
        assert!(csv.contains("delivery"));
        assert!(csv.contains("action"));
    }
}
