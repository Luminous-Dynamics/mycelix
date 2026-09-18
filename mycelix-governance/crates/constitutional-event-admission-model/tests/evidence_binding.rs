use constitutional_event_admission_model::{
    AdmissionError, AdmissionEvidence, DependencyQualification, EventAdmissionState,
    QualificationState, REQUIRED_QUALIFICATION_SUBJECTS, event_target_commitment,
};
use constitutional_event_provider::{DurableConstitutionalEvent, EventAuthorityBinding};
use serde_json::json;

fn event(action_commitment: &str) -> DurableConstitutionalEvent {
    DurableConstitutionalEvent::new(
        EventAuthorityBinding {
            operation_id: "operation-evidence-binding".into(),
            action_id: "action-evidence-binding".into(),
            proposal_id: "proposal-evidence-binding".into(),
            claim_binding_commitment: "claim-binding:evidence-binding".into(),
            action_commitment: action_commitment.into(),
            publisher_did: "did:mycelix:evidence-publisher".into(),
        },
        "EvidenceBoundEvent",
        &json!({"same": "payload"}),
        1_000,
    )
    .unwrap()
}

fn qualifications() -> Vec<DependencyQualification> {
    REQUIRED_QUALIFICATION_SUBJECTS
        .iter()
        .enumerate()
        .map(|(index, (dependency_id, semantic_head))| DependencyQualification {
            dependency_id: (*dependency_id).to_string(),
            semantic_head: (*semantic_head).to_string(),
            state: QualificationState::Qualified {
                receipt_id: format!("test-only-exact-receipt-{index}"),
            },
        })
        .collect()
}

fn evidence_for(event: &DurableConstitutionalEvent) -> AdmissionEvidence {
    AdmissionEvidence::new(
        event,
        event.authority.claim_binding_commitment.clone(),
        event_target_commitment(event),
        event.payload_commitment.clone(),
        qualifications(),
    )
    .unwrap()
}

#[test]
fn evidence_cannot_be_transplanted_to_changed_action_commitment() {
    let authorized = event("action-commitment:authorized");
    let evidence = evidence_for(&authorized);

    let changed = event("action-commitment:changed");
    assert_eq!(authorized.authority.action_id, changed.authority.action_id);
    assert_eq!(
        event_target_commitment(&authorized),
        event_target_commitment(&changed)
    );
    assert_eq!(authorized.payload_commitment, changed.payload_commitment);
    assert_ne!(authorized.event_commitment, changed.event_commitment);

    let mut state = EventAdmissionState::new();
    assert_eq!(
        state.admit_event(
            "did:mycelix:evidence-publisher",
            changed,
            &evidence,
        ),
        Err(AdmissionError::EvidenceCommitmentMismatch)
    );
}

#[test]
fn qualification_receipt_mutation_breaks_admission_evidence_commitment() {
    let candidate = event("action-commitment:authorized");
    let mut evidence = evidence_for(&candidate);
    evidence.qualifications[0].state = QualificationState::Qualified {
        receipt_id: "test-only-mutated-receipt".into(),
    };

    let mut state = EventAdmissionState::new();
    assert_eq!(
        state.admit_event(
            "did:mycelix:evidence-publisher",
            candidate,
            &evidence,
        ),
        Err(AdmissionError::EvidenceCommitmentMismatch)
    );
}
