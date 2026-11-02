use gcpn_civil_evidence_contracts::{
    audit_civil_evidence, AccessibilityEvidence, AccessibilityRequirement, AccessibilityState,
    ActionReceipt, ActionRequirement, ActionState, AuthorityAssertion, AuthorityRequirement,
    BaseClaimBinding, CaseDisposition, CaseFrame, CivilEvidenceProfile, ChannelCohortTarget,
    ComprehensionEvidence, ComprehensionRequirement, DeliveryReceipt, DeliveryState,
    FrameCase, RegistryAuthorityStatus, SignatureStatus,
};
use gcpn_compositional_assurance::{
    AuditReport, CapReceiptLink, CausalClaim, ClaimDisposition, ClaimRequirement,
    EvidenceValidity, ForecastTiming, PairedComparison, PopulationAccounting,
    PostProcessingFacts, Scenario, ScenarioFacts, Stage, StageEvidence,
    VerificationAccounting,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let report = audit_civil_evidence(&profile())?;
    println!("# Synthetic civil-evidence chain; no live alert or recipient records.");
    println!("# Signature verification is not a legal-authority determination.");
    println!("# Provider acceptance, device delivery, accessibility, comprehension and action are separate.");
    print!("{}", report.to_csv());
    Ok(())
}

fn profile() -> CivilEvidenceProfile {
    CivilEvidenceProfile {
        scenario_id: "synthetic-civil-event-1".into(),
        as_of_cutoff: 100,
        base_audit: base_audit(),
        required_base_claims: vec![
            BaseClaimBinding {
                stage: Stage::Forecast,
                claim_id: "forecast-valid".into(),
            },
            BaseClaimBinding {
                stage: Stage::CapLifecycle,
                claim_id: "receipt-valid".into(),
            },
            BaseClaimBinding {
                stage: Stage::PopulationReach,
                claim_id: "reach-valid".into(),
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
            cap_originator: "national-alert-centre".into(),
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
            cap_originator: "national-alert-centre".into(),
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
            intended_receivers: 100,
        }],
        delivery_receipts: vec![
            DeliveryReceipt {
                receipt_id: "provider-receipt".into(),
                message_id: "cap-1".into(),
                message_version: "1".into(),
                cohort_id: "cohort-a".into(),
                channel_id: "cell-broadcast".into(),
                state: DeliveryState::ProviderAccepted,
                count: 100,
                duplicate_of: None,
                recorded_at: 30,
            },
            DeliveryReceipt {
                receipt_id: "device-receipt".into(),
                message_id: "cap-1".into(),
                message_version: "1".into(),
                cohort_id: "cohort-a".into(),
                channel_id: "cell-broadcast".into(),
                state: DeliveryState::DeviceDelivered,
                count: 95,
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
            evidence_id: "accessibility-test-1".into(),
            delivery_receipt_id: "device-receipt".into(),
            cohort_id: "cohort-a".into(),
            language_id: "en".into(),
            format_id: "text".into(),
            assistive_technology_profile_id: "screen-reader".into(),
            test_protocol_id: "wcag-test-v1".into(),
            state: AccessibilityState::Accessible,
            tested_at: 40,
            recorded_at: 45,
        }],
        comprehension_requirements: vec![ComprehensionRequirement {
            cohort_id: "cohort-a".into(),
            language_id: "en".into(),
            test_protocol_id: "human-test-v1".into(),
            minimum_comprehension_rate: 0.8,
            minimum_intent_rate: 0.7,
        }],
        comprehension_evidence: vec![ComprehensionEvidence {
            evidence_id: "comprehension-test-1".into(),
            delivery_receipt_id: "device-receipt".into(),
            accessibility_evidence_id: "accessibility-test-1".into(),
            cohort_id: "cohort-a".into(),
            language_id: "en".into(),
            test_protocol_id: "human-test-v1".into(),
            tested_count: 50,
            understood_count: 45,
            intended_action_count: 40,
            tested_at: 50,
            recorded_at: 55,
        }],
        action_requirements: vec![ActionRequirement {
            action_id: "evacuation-1".into(),
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
            frame_id: "outcome-frame-1".into(),
            inclusion_criteria_id: "eligible-households-v1".into(),
            declared_eligible_cases: 3,
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
                FrameCase {
                    case_key: "case-3".into(),
                    disposition: CaseDisposition::Missing,
                    recorded_at: 80,
                },
                FrameCase {
                    case_key: "case-4".into(),
                    disposition: CaseDisposition::Excluded,
                    recorded_at: 80,
                },
            ],
        },
    }
}

fn action_receipt(receipt_id: &str, state: ActionState, event_time: u64) -> ActionReceipt {
    ActionReceipt {
        receipt_id: receipt_id.into(),
        action_id: "evacuation-1".into(),
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
    let requirements = vec![
        requirement(Stage::Forecast, "forecast-valid"),
        requirement(Stage::CapLifecycle, "receipt-valid"),
        requirement(Stage::PopulationReach, "reach-valid"),
        requirement(Stage::Decision, "decision-valid"),
    ];
    let evidence = vec![
        record("forecast-e", Stage::Forecast, "forecast-valid", "forecast-root"),
        record("cap-e", Stage::CapLifecycle, "receipt-valid", "cap-root"),
        record("reach-e", Stage::PopulationReach, "reach-valid", "reach-root"),
        record("decision-e", Stage::Decision, "decision-valid", "decision-root"),
    ];
    Scenario {
        scenario_id: "base-scenario".into(),
        specification_digest: "gcpn-compositional-assurance-v0.1".into(),
        as_of_cutoff: 100,
        requirements,
        evidence,
        facts: ScenarioFacts {
            forecast_timing: Some(ForecastTiming {
                issued_at: 10,
                decision_cutoff: 20,
                event_valid_at: 100,
                minimum_action_lead: 10,
            }),
            cap_receipt: Some(CapReceiptLink {
                current_message_id: "cap-1".into(),
                current_version: "1".into(),
                receipt_message_id: "cap-1".into(),
                receipt_version: "1".into(),
            }),
            population: Some(PopulationAccounting {
                eligible_denominator: Some(100),
                observed_cases: 100,
                typed_missing_cases: 0,
                unique_reach: 80,
            }),
            verification: Some(VerificationAccounting {
                eligible_cases: 100,
                observed_cases: 100,
                typed_missing_cases: 0,
            }),
            paired_comparison: Some(PairedComparison {
                candidate_case_frame_digest: "frame".into(),
                reference_case_frame_digest: "frame".into(),
                candidate_source: "source".into(),
                reference_source: "source".into(),
                candidate_version: "v1".into(),
                reference_version: "v1".into(),
            }),
            causal_claim: Some(CausalClaim {
                claim_id: "review".into(),
                claim_made: false,
                identification_method_id: None,
                design_id: None,
                supporting_evidence_ids: Vec::new(),
            }),
            post_processing: Some(PostProcessingFacts {
                intrinsic_skill_claim: true,
                calibration_uses_evaluation_outcomes: false,
                calibration_trained_through: Some(5),
                evaluation_period_start: Some(20),
            }),
        },
    }
    .audit()
    .expect("synthetic base audit must pass")
}

fn requirement(stage: Stage, claim_id: &str) -> ClaimRequirement {
    ClaimRequirement {
        stage,
        claim_id: claim_id.into(),
        subject_digest: "subject-1".into(),
        expected_source: Some("source".into()),
        expected_version: Some("v1".into()),
        minimum_independent_supporting_lineages: 1,
    }
}

fn record(id: &str, stage: Stage, claim_id: &str, lineage: &str) -> StageEvidence {
    StageEvidence {
        evidence_id: id.into(),
        stage,
        claim_id: claim_id.into(),
        subject_digest: "subject-1".into(),
        source: "source".into(),
        version: "v1".into(),
        event_time: 10,
        recorded_at: 20,
        validity: EvidenceValidity::Current,
        disposition: ClaimDisposition::Supports,
        lineage_root: lineage.into(),
        evaluator_id: format!("evaluator-{id}"),
    }
}
