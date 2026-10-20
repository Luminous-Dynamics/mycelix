use constitutional_event_provider::{DurableConstitutionalEvent, EventProviderError};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

pub const ADMISSION_MODEL_SCHEMA_VERSION: u16 = 1;
pub const PROVIDER_LANE: &str = "EmitEvent";
pub const COMMITMENT_PREFIX: &str = "blake3-256:";
pub const MAX_ID_LEN: usize = 256;
pub const MAX_EVIDENCE_ID_LEN: usize = 512;

const TARGET_DOMAIN: &[u8] = b"MYCELIX-CONSTITUTIONAL-EVENT-TARGET\0V1\0";
const ADMISSION_EVIDENCE_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-EVENT-ADMISSION-EVIDENCE\0V1\0";

/// Exact semantic subjects that this E1B model knows how to admit.
///
/// A repaired verifier may change without changing these semantic subjects, but
/// a semantic-head change requires a new admission-model revision rather than a
/// silent compatibility assumption.
pub const REQUIRED_QUALIFICATION_SUBJECTS: [(&str, &str); 5] = [
    (
        "MYC-CONST-003B4",
        "037f61c15ff367a1518d98f7acc8ec0fa962c2f6",
    ),
    (
        "MYC-CONST-003CR1",
        "4edb56bd3c36ddc6277e2de5e0584a65e8c99b3c",
    ),
    (
        "MYC-CONST-003D1C",
        "47d1d764323dbfaf991b5574cfde83abb7a3e4a4",
    ),
    (
        "MYC-CONST-003D1D-E0",
        "36a4fcffb6ca806570ebf439f9c36cf76401e7b2",
    ),
    (
        "MYC-CONST-003D1D-E1A",
        "249753017c3781f7a3259061aae16d7e0075e984",
    ),
];

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum AdmissionError {
    #[error("event contract invalid: {0}")]
    EventInvalid(String),
    #[error("{0}")]
    Violation(String),
    #[error("required qualification dependency missing: {0}")]
    QualificationMissing(String),
    #[error("qualification subject mismatch for {dependency_id}: expected {expected}, got {actual}")]
    QualificationSubjectMismatch {
        dependency_id: String,
        expected: String,
        actual: String,
    },
    #[error("qualification dependency is still pending: {0}")]
    QualificationPending(String),
    #[error("qualification dependency failed: {dependency_id} ({evidence_id})")]
    QualificationFailed {
        dependency_id: String,
        evidence_id: String,
    },
    #[error("authenticated author does not match durable event publisher")]
    AuthorMismatch,
    #[error("ClaimBinding reference does not match the durable event authority")]
    ClaimBindingReferenceMismatch,
    #[error("ClaimBinding target digest does not authorize this exact event target")]
    TargetDigestMismatch,
    #[error("ClaimBinding payload digest does not authorize this exact event payload")]
    PayloadDigestMismatch,
    #[error("admission evidence commitment mismatch")]
    EvidenceCommitmentMismatch,
    #[error("action is integrity halted: {0}")]
    IntegrityHalted(String),
    #[error("conflicting event under the same constitutional action identity")]
    IntegrityConflict,
    #[error("constitutional event records are immutable")]
    ImmutableEvent,
    #[error("canonical action-key index cannot be deleted")]
    CanonicalIndexImmutable,
    #[error("cannot index an action that has no admitted durable event")]
    MissingEventForIndex,
    #[error("action-key index target conflicts with the canonical durable event")]
    ConflictingIndex,
    #[error("state invariant violation: {0}")]
    StateInvariant(String),
}

type AdmissionResult<T> = Result<T, AdmissionError>;

fn violation(message: impl Into<String>) -> AdmissionError {
    AdmissionError::Violation(message.into())
}

fn map_event_error(error: EventProviderError) -> AdmissionError {
    AdmissionError::EventInvalid(error.to_string())
}

fn require_opaque(label: &str, value: &str, max_len: usize) -> AdmissionResult<()> {
    if value.trim().is_empty() || value.len() > max_len {
        return Err(violation(format!(
            "{label} must be non-empty and <= {max_len} bytes"
        )));
    }
    Ok(())
}

fn require_did(label: &str, value: &str) -> AdmissionResult<()> {
    require_opaque(label, value, MAX_ID_LEN)?;
    if !value.starts_with("did:") {
        return Err(violation(format!("{label} must be a DID")));
    }
    Ok(())
}

fn valid_hex_40(value: &str) -> bool {
    value.len() == 40
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn require_tagged_commitment(label: &str, value: &str) -> AdmissionResult<()> {
    let Some(hex) = value.strip_prefix(COMMITMENT_PREFIX) else {
        return Err(violation(format!(
            "{label} must use {COMMITMENT_PREFIX}<64-lowercase-hex>"
        )));
    };
    if hex.len() != 64
        || !hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(violation(format!(
            "{label} must use {COMMITMENT_PREFIX}<64-lowercase-hex>"
        )));
    }
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum QualificationState {
    Qualified { receipt_id: String },
    Pending,
    Failed { evidence_id: String },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DependencyQualification {
    pub dependency_id: String,
    pub semantic_head: String,
    pub state: QualificationState,
}

impl DependencyQualification {
    fn validate_shape(&self) -> AdmissionResult<()> {
        require_opaque("dependency_id", &self.dependency_id, MAX_ID_LEN)?;
        if !valid_hex_40(&self.semantic_head) {
            return Err(violation(format!(
                "semantic_head for {} must be 40 lowercase hexadecimal characters",
                self.dependency_id
            )));
        }
        match &self.state {
            QualificationState::Qualified { receipt_id } => {
                require_opaque("qualification receipt_id", receipt_id, MAX_EVIDENCE_ID_LEN)?;
            }
            QualificationState::Pending => {}
            QualificationState::Failed { evidence_id } => {
                require_opaque(
                    "qualification failure evidence_id",
                    evidence_id,
                    MAX_EVIDENCE_ID_LEN,
                )?;
            }
        }
        Ok(())
    }
}

/// Explicit evidence presented to the future integrity admission boundary.
///
/// This model never accepts a caller-provided `is_authorized: bool`. The claim
/// fields are inputs from independently authenticated/qualified evidence and are
/// verified against the candidate event; production callers are not offered a
/// convenience constructor that derives authorization from the event itself.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdmissionEvidence {
    pub schema_version: u16,
    pub claim_binding_reference: String,
    pub claim_target_digest: String,
    pub claim_payload_digest: String,
    pub qualifications: Vec<DependencyQualification>,
    pub evidence_commitment: String,
}

impl AdmissionEvidence {
    /// Construct externally supplied claim evidence and bind it to the candidate
    /// event. The three claim fields are intentionally caller-supplied; this
    /// function does not derive what the authorization ought to have said.
    pub fn new(
        event: &DurableConstitutionalEvent,
        claim_binding_reference: String,
        claim_target_digest: String,
        claim_payload_digest: String,
        qualifications: Vec<DependencyQualification>,
    ) -> AdmissionResult<Self> {
        event.validate().map_err(map_event_error)?;
        let mut evidence = Self {
            schema_version: ADMISSION_MODEL_SCHEMA_VERSION,
            claim_binding_reference,
            claim_target_digest,
            claim_payload_digest,
            qualifications,
            evidence_commitment: String::new(),
        };
        evidence.evidence_commitment = evidence.compute_commitment(event)?;
        Ok(evidence)
    }

    pub fn validate_against(
        &self,
        authenticated_author_did: &str,
        event: &DurableConstitutionalEvent,
    ) -> AdmissionResult<()> {
        event.validate().map_err(map_event_error)?;
        require_did("authenticated_author_did", authenticated_author_did)?;

        if self.schema_version != ADMISSION_MODEL_SCHEMA_VERSION {
            return Err(violation(format!(
                "unsupported admission evidence schema version {}",
                self.schema_version
            )));
        }
        require_opaque(
            "claim_binding_reference",
            &self.claim_binding_reference,
            MAX_EVIDENCE_ID_LEN,
        )?;
        require_tagged_commitment("claim_target_digest", &self.claim_target_digest)?;
        require_tagged_commitment("claim_payload_digest", &self.claim_payload_digest)?;
        require_tagged_commitment("evidence_commitment", &self.evidence_commitment)?;

        if authenticated_author_did != event.authority.publisher_did {
            return Err(AdmissionError::AuthorMismatch);
        }
        if self.claim_binding_reference != event.authority.claim_binding_commitment {
            return Err(AdmissionError::ClaimBindingReferenceMismatch);
        }
        if self.claim_target_digest != event_target_commitment(event) {
            return Err(AdmissionError::TargetDigestMismatch);
        }
        if self.claim_payload_digest != event.payload_commitment {
            return Err(AdmissionError::PayloadDigestMismatch);
        }

        self.validate_qualification_census()?;

        let expected = self.compute_commitment(event)?;
        if self.evidence_commitment != expected {
            return Err(AdmissionError::EvidenceCommitmentMismatch);
        }
        Ok(())
    }

    fn validate_qualification_census(&self) -> AdmissionResult<()> {
        let mut seen = BTreeSet::new();
        for qualification in &self.qualifications {
            qualification.validate_shape()?;
            if !seen.insert(qualification.dependency_id.as_str()) {
                return Err(violation(format!(
                    "duplicate qualification dependency {}",
                    qualification.dependency_id
                )));
            }
        }

        for (required_id, required_head) in REQUIRED_QUALIFICATION_SUBJECTS {
            let Some(qualification) = self
                .qualifications
                .iter()
                .find(|item| item.dependency_id == required_id)
            else {
                return Err(AdmissionError::QualificationMissing(required_id.to_string()));
            };

            if qualification.semantic_head != required_head {
                return Err(AdmissionError::QualificationSubjectMismatch {
                    dependency_id: required_id.to_string(),
                    expected: required_head.to_string(),
                    actual: qualification.semantic_head.clone(),
                });
            }

            match &qualification.state {
                QualificationState::Qualified { receipt_id } => {
                    require_opaque(
                        "qualification receipt_id",
                        receipt_id,
                        MAX_EVIDENCE_ID_LEN,
                    )?;
                }
                QualificationState::Pending => {
                    return Err(AdmissionError::QualificationPending(
                        required_id.to_string(),
                    ));
                }
                QualificationState::Failed { evidence_id } => {
                    return Err(AdmissionError::QualificationFailed {
                        dependency_id: required_id.to_string(),
                        evidence_id: evidence_id.clone(),
                    });
                }
            }
        }

        if seen.len() != REQUIRED_QUALIFICATION_SUBJECTS.len() {
            return Err(violation(
                "qualification evidence must contain exactly the required dependency census",
            ));
        }

        Ok(())
    }

    fn compute_commitment(&self, event: &DurableConstitutionalEvent) -> AdmissionResult<String> {
        event.validate().map_err(map_event_error)?;
        let mut qualifications = self.qualifications.clone();
        qualifications.sort_by(|left, right| left.dependency_id.cmp(&right.dependency_id));

        let mut hasher = blake3::Hasher::new();
        hasher.update(ADMISSION_EVIDENCE_DOMAIN);
        hasher.update(&self.schema_version.to_be_bytes());
        push_str(&mut hasher, &event.event_commitment);
        push_str(&mut hasher, &self.claim_binding_reference);
        push_str(&mut hasher, &self.claim_target_digest);
        push_str(&mut hasher, &self.claim_payload_digest);
        for qualification in &qualifications {
            push_str(&mut hasher, &qualification.dependency_id);
            push_str(&mut hasher, &qualification.semantic_head);
            match &qualification.state {
                QualificationState::Qualified { receipt_id } => {
                    push_str(&mut hasher, "Qualified");
                    push_str(&mut hasher, receipt_id);
                }
                QualificationState::Pending => {
                    push_str(&mut hasher, "Pending");
                }
                QualificationState::Failed { evidence_id } => {
                    push_str(&mut hasher, "Failed");
                    push_str(&mut hasher, evidence_id);
                }
            }
        }
        Ok(tagged_hash(hasher.finalize()))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum IntegrityFault {
    ConflictingEvent {
        action_id: String,
        preserved_event_commitment: String,
        conflicting_event_commitment: String,
    },
    ConflictingActionIndex {
        action_id: String,
        canonical_event_commitment: String,
        conflicting_target_commitment: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum AdmissionOutcome {
    Created {
        action_id: String,
        event_commitment: String,
    },
    ExistingSame {
        action_id: String,
        event_commitment: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum IndexWriteOutcome {
    ExistingSame {
        action_id: String,
        event_commitment: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ActionAdmissionStatus {
    Absent,
    Admitted { event_commitment: String },
    IntegrityHalted {
        preserved_event_commitment: Option<String>,
        fault: IntegrityFault,
    },
}

/// Pure reference state for the eventual DHT admission boundary.
///
/// This is deliberately not a Holochain entry store. It defines safety behavior
/// that a future integrity zome must preserve even under direct/malicious writes.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct EventAdmissionState {
    events_by_action: BTreeMap<String, DurableConstitutionalEvent>,
    action_index: BTreeMap<String, String>,
    halted_actions: BTreeMap<String, IntegrityFault>,
}

impl EventAdmissionState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn admit_event(
        &mut self,
        authenticated_author_did: &str,
        event: DurableConstitutionalEvent,
        evidence: &AdmissionEvidence,
    ) -> AdmissionResult<AdmissionOutcome> {
        let action_id = event.authority.action_id.clone();
        if self.halted_actions.contains_key(&action_id) {
            return Err(AdmissionError::IntegrityHalted(action_id));
        }

        evidence.validate_against(authenticated_author_did, &event)?;

        if let Some(existing) = self.events_by_action.get(&action_id) {
            let canonical_index = self.action_index.get(&action_id).ok_or_else(|| {
                AdmissionError::StateInvariant(format!(
                    "admitted action {action_id} has no canonical index"
                ))
            })?;
            if canonical_index != &existing.event_commitment {
                return Err(AdmissionError::StateInvariant(format!(
                    "canonical index for {action_id} does not match preserved event"
                )));
            }

            if existing.event_commitment == event.event_commitment {
                return Ok(AdmissionOutcome::ExistingSame {
                    action_id,
                    event_commitment: existing.event_commitment.clone(),
                });
            }

            let fault = IntegrityFault::ConflictingEvent {
                action_id: action_id.clone(),
                preserved_event_commitment: existing.event_commitment.clone(),
                conflicting_event_commitment: event.event_commitment.clone(),
            };
            self.halted_actions.insert(action_id, fault);
            self.validate_state()?;
            return Err(AdmissionError::IntegrityConflict);
        }

        if self.action_index.contains_key(&action_id) {
            return Err(AdmissionError::StateInvariant(format!(
                "action index exists without event for {action_id}"
            )));
        }

        let event_commitment = event.event_commitment.clone();
        self.events_by_action.insert(action_id.clone(), event);
        self.action_index
            .insert(action_id.clone(), event_commitment.clone());
        self.validate_state()?;

        Ok(AdmissionOutcome::Created {
            action_id,
            event_commitment,
        })
    }

    /// Models an adversarial/direct attempt to create another logical action-key
    /// index edge. The normal admission path creates the canonical index together
    /// with the event. A different target is an integrity fault, never an
    /// alternate valid lookup path.
    pub fn admit_action_index_write(
        &mut self,
        action_id: &str,
        target_event_commitment: &str,
    ) -> AdmissionResult<IndexWriteOutcome> {
        require_opaque("action_id", action_id, MAX_ID_LEN)?;
        require_tagged_commitment("target_event_commitment", target_event_commitment)?;

        if self.halted_actions.contains_key(action_id) {
            return Err(AdmissionError::IntegrityHalted(action_id.to_string()));
        }

        let Some(event) = self.events_by_action.get(action_id) else {
            return Err(AdmissionError::MissingEventForIndex);
        };
        let canonical = event.event_commitment.clone();
        let indexed = self.action_index.get(action_id).ok_or_else(|| {
            AdmissionError::StateInvariant(format!(
                "admitted action {action_id} has no canonical index"
            ))
        })?;
        if indexed != &canonical {
            return Err(AdmissionError::StateInvariant(format!(
                "stored action index for {action_id} does not match event"
            )));
        }

        if target_event_commitment == canonical {
            return Ok(IndexWriteOutcome::ExistingSame {
                action_id: action_id.to_string(),
                event_commitment: canonical,
            });
        }

        let fault = IntegrityFault::ConflictingActionIndex {
            action_id: action_id.to_string(),
            canonical_event_commitment: canonical,
            conflicting_target_commitment: target_event_commitment.to_string(),
        };
        self.halted_actions.insert(action_id.to_string(), fault);
        self.validate_state()?;
        Err(AdmissionError::ConflictingIndex)
    }

    pub fn attempt_update_event(
        &self,
        action_id: &str,
        _replacement: &DurableConstitutionalEvent,
    ) -> AdmissionResult<()> {
        require_opaque("action_id", action_id, MAX_ID_LEN)?;
        if !self.events_by_action.contains_key(action_id) {
            return Err(violation("cannot update an absent constitutional event"));
        }
        Err(AdmissionError::ImmutableEvent)
    }

    pub fn attempt_delete_action_index(&self, action_id: &str) -> AdmissionResult<()> {
        require_opaque("action_id", action_id, MAX_ID_LEN)?;
        if !self.action_index.contains_key(action_id) {
            return Err(violation("cannot delete an absent canonical action index"));
        }
        Err(AdmissionError::CanonicalIndexImmutable)
    }

    pub fn event(&self, action_id: &str) -> Option<&DurableConstitutionalEvent> {
        self.events_by_action.get(action_id)
    }

    pub fn canonical_index_target(&self, action_id: &str) -> Option<&str> {
        self.action_index.get(action_id).map(String::as_str)
    }

    pub fn status(&self, action_id: &str) -> ActionAdmissionStatus {
        if let Some(fault) = self.halted_actions.get(action_id) {
            return ActionAdmissionStatus::IntegrityHalted {
                preserved_event_commitment: self
                    .events_by_action
                    .get(action_id)
                    .map(|event| event.event_commitment.clone()),
                fault: fault.clone(),
            };
        }
        if let Some(event) = self.events_by_action.get(action_id) {
            return ActionAdmissionStatus::Admitted {
                event_commitment: event.event_commitment.clone(),
            };
        }
        ActionAdmissionStatus::Absent
    }

    pub fn validate_state(&self) -> AdmissionResult<()> {
        for (action_id, event) in &self.events_by_action {
            event.validate().map_err(map_event_error)?;
            if event.provider_key() != action_id {
                return Err(AdmissionError::StateInvariant(format!(
                    "event map key {action_id} differs from provider key {}",
                    event.provider_key()
                )));
            }
            let target = self.action_index.get(action_id).ok_or_else(|| {
                AdmissionError::StateInvariant(format!(
                    "event {action_id} is missing canonical action index"
                ))
            })?;
            if target != &event.event_commitment {
                return Err(AdmissionError::StateInvariant(format!(
                    "event {action_id} canonical index target mismatch"
                )));
            }
        }

        for (action_id, target) in &self.action_index {
            let event = self.events_by_action.get(action_id).ok_or_else(|| {
                AdmissionError::StateInvariant(format!(
                    "action index {action_id} points without an admitted event"
                ))
            })?;
            if target != &event.event_commitment {
                return Err(AdmissionError::StateInvariant(format!(
                    "action index {action_id} target differs from admitted event"
                )));
            }
        }

        for (action_id, fault) in &self.halted_actions {
            match fault {
                IntegrityFault::ConflictingEvent {
                    action_id: fault_action,
                    preserved_event_commitment,
                    ..
                } => {
                    if fault_action != action_id {
                        return Err(AdmissionError::StateInvariant(
                            "conflicting-event halt stored under wrong action key".into(),
                        ));
                    }
                    let event = self.events_by_action.get(action_id).ok_or_else(|| {
                        AdmissionError::StateInvariant(
                            "conflicting-event halt must preserve original event".into(),
                        )
                    })?;
                    if &event.event_commitment != preserved_event_commitment {
                        return Err(AdmissionError::StateInvariant(
                            "conflicting-event halt does not preserve canonical event".into(),
                        ));
                    }
                }
                IntegrityFault::ConflictingActionIndex {
                    action_id: fault_action,
                    canonical_event_commitment,
                    ..
                } => {
                    if fault_action != action_id {
                        return Err(AdmissionError::StateInvariant(
                            "conflicting-index halt stored under wrong action key".into(),
                        ));
                    }
                    let event = self.events_by_action.get(action_id).ok_or_else(|| {
                        AdmissionError::StateInvariant(
                            "conflicting-index halt must preserve original event".into(),
                        )
                    })?;
                    if &event.event_commitment != canonical_event_commitment {
                        return Err(AdmissionError::StateInvariant(
                            "conflicting-index halt canonical event mismatch".into(),
                        ));
                    }
                }
            }
        }

        Ok(())
    }
}

/// Deterministic E1A target commitment for the exact logical event target.
pub fn event_target_commitment(event: &DurableConstitutionalEvent) -> String {
    let mut hasher = blake3::Hasher::new();
    hasher.update(TARGET_DOMAIN);
    push_str(&mut hasher, PROVIDER_LANE);
    push_str(&mut hasher, &event.authority.operation_id);
    push_str(&mut hasher, &event.authority.action_id);
    push_str(&mut hasher, &event.authority.proposal_id);
    push_str(&mut hasher, &event.event_name);
    tagged_hash(hasher.finalize())
}

fn tagged_hash(hash: blake3::Hash) -> String {
    format!("{COMMITMENT_PREFIX}{}", hash.to_hex())
}

fn push_str(hasher: &mut blake3::Hasher, value: &str) {
    let bytes = value.as_bytes();
    hasher.update(&(bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
}

#[cfg(test)]
mod tests {
    use super::*;
    use constitutional_event_provider::{
        EventAuthorityBinding, ProjectionOutcome, SignalProjectionAttempt,
        constitutional_event_complete,
    };
    use serde_json::json;

    fn event_at(timestamp: u64) -> DurableConstitutionalEvent {
        DurableConstitutionalEvent::new(
            EventAuthorityBinding {
                operation_id: "operation-001".into(),
                action_id: "action-002".into(),
                proposal_id: "proposal-003".into(),
                claim_binding_commitment: "claim-binding:exact-b4-binding".into(),
                action_commitment: "action-commitment:def456".into(),
                publisher_did: "did:mycelix:publisher".into(),
            },
            "ProposalExecuted",
            &json!({"proposal_id": "proposal-003", "outcome": "accepted"}),
            timestamp,
        )
        .unwrap()
    }

    fn qualified_dependencies() -> Vec<DependencyQualification> {
        REQUIRED_QUALIFICATION_SUBJECTS
            .iter()
            .enumerate()
            .map(|(index, (dependency_id, semantic_head))| DependencyQualification {
                dependency_id: (*dependency_id).to_string(),
                semantic_head: (*semantic_head).to_string(),
                state: QualificationState::Qualified {
                    receipt_id: format!("test-only-retained-receipt-{index}"),
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
            qualified_dependencies(),
        )
        .unwrap()
    }

    fn admit_baseline() -> (
        EventAdmissionState,
        DurableConstitutionalEvent,
        AdmissionEvidence,
    ) {
        let event = event_at(1_000);
        let evidence = evidence_for(&event);
        let mut state = EventAdmissionState::new();
        assert!(matches!(
            state
                .admit_event("did:mycelix:publisher", event.clone(), &evidence)
                .unwrap(),
            AdmissionOutcome::Created { .. }
        ));
        (state, event, evidence)
    }

    #[test]
    fn first_qualified_exact_event_commits_once_with_canonical_index() {
        let (state, event, _) = admit_baseline();
        assert_eq!(
            state.canonical_index_target(event.provider_key()),
            Some(event.event_commitment.as_str())
        );
        assert!(matches!(
            state.status(event.provider_key()),
            ActionAdmissionStatus::Admitted { .. }
        ));
        state.validate_state().unwrap();
    }

    #[test]
    fn pending_qualification_rejects_admission_fail_closed() {
        let event = event_at(1_000);
        let mut dependencies = qualified_dependencies();
        dependencies[0].state = QualificationState::Pending;
        let evidence = AdmissionEvidence::new(
            &event,
            event.authority.claim_binding_commitment.clone(),
            event_target_commitment(&event),
            event.payload_commitment.clone(),
            dependencies,
        )
        .unwrap();
        let mut state = EventAdmissionState::new();
        assert!(matches!(
            state.admit_event("did:mycelix:publisher", event, &evidence),
            Err(AdmissionError::QualificationPending(_))
        ));
    }

    #[test]
    fn failed_qualification_rejects_admission_fail_closed() {
        let event = event_at(1_000);
        let mut dependencies = qualified_dependencies();
        dependencies[1].state = QualificationState::Failed {
            evidence_id: "retained-red-receipt".into(),
        };
        let evidence = AdmissionEvidence::new(
            &event,
            event.authority.claim_binding_commitment.clone(),
            event_target_commitment(&event),
            event.payload_commitment.clone(),
            dependencies,
        )
        .unwrap();
        let mut state = EventAdmissionState::new();
        assert!(matches!(
            state.admit_event("did:mycelix:publisher", event, &evidence),
            Err(AdmissionError::QualificationFailed { .. })
        ));
    }

    #[test]
    fn missing_qualification_dependency_is_rejected() {
        let event = event_at(1_000);
        let mut dependencies = qualified_dependencies();
        dependencies.pop();
        let evidence = AdmissionEvidence::new(
            &event,
            event.authority.claim_binding_commitment.clone(),
            event_target_commitment(&event),
            event.payload_commitment.clone(),
            dependencies,
        )
        .unwrap();
        let mut state = EventAdmissionState::new();
        assert!(matches!(
            state.admit_event("did:mycelix:publisher", event, &evidence),
            Err(AdmissionError::QualificationMissing(_))
        ));
    }

    #[test]
    fn qualification_for_wrong_semantic_subject_is_rejected() {
        let event = event_at(1_000);
        let mut dependencies = qualified_dependencies();
        dependencies[2].semantic_head = "0000000000000000000000000000000000000000".into();
        let evidence = AdmissionEvidence::new(
            &event,
            event.authority.claim_binding_commitment.clone(),
            event_target_commitment(&event),
            event.payload_commitment.clone(),
            dependencies,
        )
        .unwrap();
        let mut state = EventAdmissionState::new();
        assert!(matches!(
            state.admit_event("did:mycelix:publisher", event, &evidence),
            Err(AdmissionError::QualificationSubjectMismatch { .. })
        ));
    }

    #[test]
    fn direct_write_with_spoofed_publisher_is_rejected_without_halting_canonical_action() {
        let event = event_at(1_000);
        let evidence = evidence_for(&event);
        let mut state = EventAdmissionState::new();
        assert_eq!(
            state.admit_event("did:mycelix:attacker", event.clone(), &evidence),
            Err(AdmissionError::AuthorMismatch)
        );
        assert_eq!(
            state.status(event.provider_key()),
            ActionAdmissionStatus::Absent
        );
    }

    #[test]
    fn wrong_claim_binding_reference_is_rejected() {
        let event = event_at(1_000);
        let evidence = AdmissionEvidence::new(
            &event,
            "claim-binding:other".into(),
            event_target_commitment(&event),
            event.payload_commitment.clone(),
            qualified_dependencies(),
        )
        .unwrap();
        let mut state = EventAdmissionState::new();
        assert_eq!(
            state.admit_event("did:mycelix:publisher", event, &evidence),
            Err(AdmissionError::ClaimBindingReferenceMismatch)
        );
    }

    #[test]
    fn wrong_target_digest_is_rejected() {
        let event = event_at(1_000);
        let evidence = AdmissionEvidence::new(
            &event,
            event.authority.claim_binding_commitment.clone(),
            format!("{COMMITMENT_PREFIX}{}", "0".repeat(64)),
            event.payload_commitment.clone(),
            qualified_dependencies(),
        )
        .unwrap();
        let mut state = EventAdmissionState::new();
        assert_eq!(
            state.admit_event("did:mycelix:publisher", event, &evidence),
            Err(AdmissionError::TargetDigestMismatch)
        );
    }

    #[test]
    fn wrong_payload_digest_is_rejected() {
        let event = event_at(1_000);
        let evidence = AdmissionEvidence::new(
            &event,
            event.authority.claim_binding_commitment.clone(),
            event_target_commitment(&event),
            format!("{COMMITMENT_PREFIX}{}", "1".repeat(64)),
            qualified_dependencies(),
        )
        .unwrap();
        let mut state = EventAdmissionState::new();
        assert_eq!(
            state.admit_event("did:mycelix:publisher", event, &evidence),
            Err(AdmissionError::PayloadDigestMismatch)
        );
    }

    #[test]
    fn evidence_commitment_tampering_is_rejected() {
        let event = event_at(1_000);
        let mut evidence = evidence_for(&event);
        evidence.evidence_commitment = format!("{COMMITMENT_PREFIX}{}", "2".repeat(64));
        let mut state = EventAdmissionState::new();
        assert_eq!(
            state.admit_event("did:mycelix:publisher", event, &evidence),
            Err(AdmissionError::EvidenceCommitmentMismatch)
        );
    }

    #[test]
    fn exact_duplicate_is_idempotent_and_does_not_create_second_event() {
        let (mut state, event, evidence) = admit_baseline();
        let retry = event_at(9_000);
        let retry_evidence = evidence_for(&retry);
        assert_eq!(event.event_commitment, retry.event_commitment);
        assert!(matches!(
            state
                .admit_event("did:mycelix:publisher", retry, &retry_evidence)
                .unwrap(),
            AdmissionOutcome::ExistingSame { .. }
        ));
        assert_eq!(state.events_by_action.len(), 1);
        assert_eq!(state.action_index.len(), 1);
        assert_eq!(
            evidence.claim_target_digest,
            retry_evidence.claim_target_digest
        );
    }

    #[test]
    fn conflicting_payload_under_same_action_preserves_original_and_halts() {
        let (mut state, original, _) = admit_baseline();
        let conflict = DurableConstitutionalEvent::new(
            original.authority.clone(),
            original.event_name.clone(),
            &json!({"proposal_id": "proposal-003", "outcome": "DIFFERENT"}),
            2_000,
        )
        .unwrap();
        let evidence = evidence_for(&conflict);

        assert_eq!(
            state.admit_event("did:mycelix:publisher", conflict, &evidence),
            Err(AdmissionError::IntegrityConflict)
        );
        assert_eq!(
            state.event(original.provider_key()).unwrap().event_commitment,
            original.event_commitment
        );
        assert!(matches!(
            state.status(original.provider_key()),
            ActionAdmissionStatus::IntegrityHalted { .. }
        ));
    }

    #[test]
    fn independently_authenticated_conflicting_publisher_halts_same_action() {
        let (mut state, original, _) = admit_baseline();
        let mut authority = original.authority.clone();
        authority.publisher_did = "did:mycelix:other-author".into();
        let conflict = DurableConstitutionalEvent::new(
            authority,
            original.event_name.clone(),
            &json!({"proposal_id": "proposal-003", "outcome": "accepted"}),
            2_000,
        )
        .unwrap();
        let evidence = evidence_for(&conflict);
        assert_eq!(
            state.admit_event("did:mycelix:other-author", conflict, &evidence),
            Err(AdmissionError::IntegrityConflict)
        );
        assert!(matches!(
            state.status(original.provider_key()),
            ActionAdmissionStatus::IntegrityHalted { .. }
        ));
    }

    #[test]
    fn post_conflict_replay_is_blocked_until_future_recovery_protocol() {
        let (mut state, original, evidence) = admit_baseline();
        let conflict = DurableConstitutionalEvent::new(
            original.authority.clone(),
            original.event_name.clone(),
            &json!({"conflict": true}),
            2_000,
        )
        .unwrap();
        let conflict_evidence = evidence_for(&conflict);
        assert_eq!(
            state.admit_event("did:mycelix:publisher", conflict, &conflict_evidence),
            Err(AdmissionError::IntegrityConflict)
        );
        assert!(matches!(
            state.admit_event("did:mycelix:publisher", original, &evidence),
            Err(AdmissionError::IntegrityHalted(_))
        ));
    }

    #[test]
    fn constitutional_event_update_is_always_rejected() {
        let (state, event, _) = admit_baseline();
        let replacement = event_at(9_000);
        assert_eq!(
            state.attempt_update_event(event.provider_key(), &replacement),
            Err(AdmissionError::ImmutableEvent)
        );
    }

    #[test]
    fn canonical_action_index_delete_is_always_rejected() {
        let (state, event, _) = admit_baseline();
        assert_eq!(
            state.attempt_delete_action_index(event.provider_key()),
            Err(AdmissionError::CanonicalIndexImmutable)
        );
    }

    #[test]
    fn duplicate_same_action_index_is_idempotent() {
        let (mut state, event, _) = admit_baseline();
        assert!(matches!(
            state
                .admit_action_index_write(event.provider_key(), &event.event_commitment)
                .unwrap(),
            IndexWriteOutcome::ExistingSame { .. }
        ));
    }

    #[test]
    fn alternate_action_index_target_is_rejected_and_halts_action() {
        let (mut state, event, _) = admit_baseline();
        let alternate = format!("{COMMITMENT_PREFIX}{}", "f".repeat(64));
        assert_eq!(
            state.admit_action_index_write(event.provider_key(), &alternate),
            Err(AdmissionError::ConflictingIndex)
        );
        assert!(matches!(
            state.status(event.provider_key()),
            ActionAdmissionStatus::IntegrityHalted { .. }
        ));
        assert_eq!(
            state.canonical_index_target(event.provider_key()),
            Some(event.event_commitment.as_str())
        );
    }

    #[test]
    fn action_index_cannot_exist_before_durable_event() {
        let mut state = EventAdmissionState::new();
        let alternate = format!("{COMMITMENT_PREFIX}{}", "a".repeat(64));
        assert_eq!(
            state.admit_action_index_write("action-absent", &alternate),
            Err(AdmissionError::MissingEventForIndex)
        );
    }

    #[test]
    fn signal_projection_failure_does_not_change_admission_truth() {
        let (state, event, _) = admit_baseline();
        let projection = SignalProjectionAttempt {
            projection_attempt_id: "projection-1".into(),
            operation_id: event.authority.operation_id.clone(),
            action_id: event.authority.action_id.clone(),
            durable_event_commitment: event.event_commitment.clone(),
            attempt_ordinal: 1,
            attempted_at_unix_ms: 2_000,
            outcome: ProjectionOutcome::Failed {
                error_class: "subscriber-offline".into(),
            },
        };
        assert!(constitutional_event_complete(&event, &[projection]).unwrap());
        assert!(matches!(
            state.status(event.provider_key()),
            ActionAdmissionStatus::Admitted { .. }
        ));
    }
}
