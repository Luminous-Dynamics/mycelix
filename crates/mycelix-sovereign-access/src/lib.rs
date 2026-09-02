//! Sovereign Intelligence Fabric: accountable person-linked access protocol.
//!
//! This crate defines portable, serialization-friendly policy types and local
//! validation invariants. It intentionally does **not** implement signatures,
//! credential verification, ZK proofs, persistence, or notification delivery.
//! Those are supplied by Mycelix, Xenia, Symthaea, or another conforming host.

use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "sif-access-v0.1";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SubjectRef {
    /// DID, pairwise pseudonym, or another stable opaque subject identifier.
    pub id: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActorRef {
    pub id: String,
    pub organization: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Purpose {
    /// Stable machine-readable purpose code, e.g. `vehicle.theft.recovery`.
    pub code: String,
    /// Human-readable explanation shown to the data subject.
    pub description: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityRef {
    /// Reference to the credential, warrant, consent grant, policy delegation,
    /// or other authority whose validity is checked outside this crate.
    pub id: String,
    pub authority_type: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DisclosureClass {
    /// Only a predicate result such as yes/no or bounded membership is returned.
    PredicateOnly,
    /// A declared minimal set of fields may be disclosed.
    MinimalFields { fields: Vec<String> },
    /// A specific evidentiary artifact may be disclosed by reference.
    EvidenceArtifact,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LookupScope {
    /// A query intentionally targets an identifiable or linkable individual.
    PersonLinked { subject: SubjectRef },
    /// Privacy-preserving aggregate computation that must not resolve subjects.
    AggregateOnly { cohort_policy: String },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct QueryCapability {
    pub protocol_version: String,
    pub capability_id: String,
    pub requester: ActorRef,
    pub scope: LookupScope,
    pub purpose: Purpose,
    pub authority: AuthorityRef,
    pub issued_at_ms: u64,
    pub expires_at_ms: u64,
    pub max_disclosure: DisclosureClass,
    /// Maximum number of successful person-linked evaluations permitted by
    /// this capability. Aggregate-only capabilities must use zero here.
    pub person_query_budget: u32,
    /// Hash/reference to the governing policy bundle.
    pub policy_ref: String,
    /// Signature/proof reference checked by the integrating security layer.
    pub authorization_proof_ref: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DecisionKind {
    Allowed,
    Denied,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AccessDecision {
    pub kind: DecisionKind,
    pub reason_code: String,
    pub policy_ref: String,
    /// Attestation/reference proving which policy computation produced this
    /// decision. The proof itself lives outside this crate.
    pub decision_proof_ref: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DelayedNotificationAuthorization {
    pub authorization_id: String,
    /// Machine-readable reason; free-form hidden narratives should not be the
    /// only basis for suppressing immediate notice.
    pub justification_code: String,
    /// Reference to the independent legal/policy authority for delay.
    pub authority_ref: AuthorityRef,
    /// One or more approvers. Integrators may require threshold signatures.
    pub approved_by: Vec<ActorRef>,
    pub created_at_ms: u64,
    pub release_at_ms: u64,
    /// Hard outer limit. Notification cannot remain concealed after this time.
    pub mandatory_release_at_ms: u64,
    pub authorization_proof_ref: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NotificationPolicy {
    Immediate,
    Delayed(DelayedNotificationAuthorization),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DisclosureResult {
    None,
    Predicate { result_ref: String },
    Fields { field_names: Vec<String>, artifact_ref: String },
    Evidence { artifact_ref: String },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AccessReceipt {
    pub protocol_version: String,
    pub receipt_id: String,
    pub capability_id: String,
    pub subject: SubjectRef,
    pub requester: ActorRef,
    pub purpose: Purpose,
    pub authority: AuthorityRef,
    pub evaluated_at_ms: u64,
    pub decision: AccessDecision,
    pub disclosure: DisclosureResult,
    pub notification: NotificationPolicy,
    /// Cryptographic session provenance supplied by Xenia or equivalent.
    pub session_provenance_ref: String,
    /// Optional previous receipt commitment for append-only/hash-chain storage.
    pub previous_receipt_ref: Option<String>,
    /// Signature/attestation covering the complete canonical receipt.
    pub receipt_proof_ref: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeliveryState {
    Pending,
    Delivered { delivered_at_ms: u64 },
    Failed { last_attempt_at_ms: u64, reason_code: String },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CitizenNotification {
    pub protocol_version: String,
    pub notification_id: String,
    pub receipt_id: String,
    pub subject: SubjectRef,
    /// Earliest instant the notification may become visible. For immediate
    /// notices this should equal the receipt evaluation time.
    pub available_at_ms: u64,
    pub state: DeliveryState,
    pub human_summary: String,
    /// Stable route/reference for inspection, correction, or contest.
    pub contest_ref: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ValidationError {
    WrongProtocolVersion,
    Empty(&'static str),
    InvalidTimeRange(&'static str),
    PersonLookupRequiresBudget,
    AggregateLookupMustNotHavePersonBudget,
    AggregateLookupCannotRequestEvidenceArtifact,
    MinimalFieldsMustBeDeclared,
    DelayRequiresApprover,
    DelayReleaseAfterMandatoryRelease,
    DelayCreatedAfterRelease,
    DelayMissingProof,
    DeniedLookupDisclosedData,
    DisclosureExceedsCapability,
    ReceiptSubjectMismatch,
    ReceiptCapabilityMismatch,
    ReceiptPolicyMismatch,
    ReceiptMissingProof,
    ImmediateNotificationAvailabilityMismatch,
    DelayedNotificationAvailabilityMismatch,
    NotificationSubjectMismatch,
    EmptyContestRoute,
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for ValidationError {}

fn require(value: &str, field: &'static str) -> Result<(), ValidationError> {
    if value.trim().is_empty() {
        Err(ValidationError::Empty(field))
    } else {
        Ok(())
    }
}

impl QueryCapability {
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(ValidationError::WrongProtocolVersion);
        }
        require(&self.capability_id, "capability_id")?;
        require(&self.requester.id, "requester.id")?;
        require(&self.purpose.code, "purpose.code")?;
        require(&self.purpose.description, "purpose.description")?;
        require(&self.authority.id, "authority.id")?;
        require(&self.authority.authority_type, "authority.authority_type")?;
        require(&self.policy_ref, "policy_ref")?;
        require(&self.authorization_proof_ref, "authorization_proof_ref")?;

        if self.expires_at_ms <= self.issued_at_ms {
            return Err(ValidationError::InvalidTimeRange("capability lifetime"));
        }

        match &self.scope {
            LookupScope::PersonLinked { subject } => {
                require(&subject.id, "scope.subject.id")?;
                if self.person_query_budget == 0 {
                    return Err(ValidationError::PersonLookupRequiresBudget);
                }
            }
            LookupScope::AggregateOnly { cohort_policy } => {
                require(cohort_policy, "scope.cohort_policy")?;
                if self.person_query_budget != 0 {
                    return Err(ValidationError::AggregateLookupMustNotHavePersonBudget);
                }
                if matches!(self.max_disclosure, DisclosureClass::EvidenceArtifact) {
                    return Err(ValidationError::AggregateLookupCannotRequestEvidenceArtifact);
                }
            }
        }

        if let DisclosureClass::MinimalFields { fields } = &self.max_disclosure {
            if fields.is_empty() || fields.iter().any(|f| f.trim().is_empty()) {
                return Err(ValidationError::MinimalFieldsMustBeDeclared);
            }
        }

        Ok(())
    }

    pub fn subject(&self) -> Option<&SubjectRef> {
        match &self.scope {
            LookupScope::PersonLinked { subject } => Some(subject),
            LookupScope::AggregateOnly { .. } => None,
        }
    }
}

impl DelayedNotificationAuthorization {
    pub fn validate(&self) -> Result<(), ValidationError> {
        require(&self.authorization_id, "delay.authorization_id")?;
        require(&self.justification_code, "delay.justification_code")?;
        require(&self.authority_ref.id, "delay.authority_ref.id")?;
        require(&self.authority_ref.authority_type, "delay.authority_ref.authority_type")?;
        if self.approved_by.is_empty() || self.approved_by.iter().any(|a| a.id.trim().is_empty()) {
            return Err(ValidationError::DelayRequiresApprover);
        }
        if self.created_at_ms >= self.release_at_ms {
            return Err(ValidationError::DelayCreatedAfterRelease);
        }
        if self.release_at_ms > self.mandatory_release_at_ms {
            return Err(ValidationError::DelayReleaseAfterMandatoryRelease);
        }
        if self.authorization_proof_ref.trim().is_empty() {
            return Err(ValidationError::DelayMissingProof);
        }
        Ok(())
    }
}

impl AccessReceipt {
    pub fn validate_against(&self, capability: &QueryCapability) -> Result<(), ValidationError> {
        capability.validate()?;
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(ValidationError::WrongProtocolVersion);
        }
        require(&self.receipt_id, "receipt_id")?;
        if self.capability_id != capability.capability_id {
            return Err(ValidationError::ReceiptCapabilityMismatch);
        }
        let subject = capability.subject().ok_or(ValidationError::ReceiptSubjectMismatch)?;
        if &self.subject != subject {
            return Err(ValidationError::ReceiptSubjectMismatch);
        }
        if self.decision.policy_ref != capability.policy_ref {
            return Err(ValidationError::ReceiptPolicyMismatch);
        }
        require(&self.decision.reason_code, "decision.reason_code")?;
        require(&self.decision.decision_proof_ref, "decision.decision_proof_ref")?;
        require(&self.session_provenance_ref, "session_provenance_ref")?;
        if self.receipt_proof_ref.trim().is_empty() {
            return Err(ValidationError::ReceiptMissingProof);
        }

        if self.decision.kind == DecisionKind::Denied && self.disclosure != DisclosureResult::None {
            return Err(ValidationError::DeniedLookupDisclosedData);
        }

        if self.decision.kind == DecisionKind::Allowed && !disclosure_within(&self.disclosure, &capability.max_disclosure) {
            return Err(ValidationError::DisclosureExceedsCapability);
        }

        if let NotificationPolicy::Delayed(delay) = &self.notification {
            delay.validate()?;
            if delay.created_at_ms > self.evaluated_at_ms {
                return Err(ValidationError::InvalidTimeRange("delay created after lookup"));
            }
        }
        Ok(())
    }

    pub fn notification_available_at_ms(&self) -> u64 {
        match &self.notification {
            NotificationPolicy::Immediate => self.evaluated_at_ms,
            NotificationPolicy::Delayed(delay) => delay.release_at_ms,
        }
    }
}

fn disclosure_within(result: &DisclosureResult, allowed: &DisclosureClass) -> bool {
    match (result, allowed) {
        (DisclosureResult::None, _) => true,
        (DisclosureResult::Predicate { .. }, _) => true,
        (DisclosureResult::Evidence { .. }, DisclosureClass::EvidenceArtifact) => true,
        (DisclosureResult::Fields { field_names, .. }, DisclosureClass::MinimalFields { fields }) => {
            !field_names.is_empty() && field_names.iter().all(|name| fields.contains(name))
        }
        (DisclosureResult::Fields { .. }, DisclosureClass::EvidenceArtifact) => true,
        _ => false,
    }
}

impl CitizenNotification {
    pub fn validate_against(&self, receipt: &AccessReceipt) -> Result<(), ValidationError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(ValidationError::WrongProtocolVersion);
        }
        require(&self.notification_id, "notification_id")?;
        if self.receipt_id != receipt.receipt_id {
            return Err(ValidationError::ReceiptCapabilityMismatch);
        }
        if self.subject != receipt.subject {
            return Err(ValidationError::NotificationSubjectMismatch);
        }
        let expected = receipt.notification_available_at_ms();
        match receipt.notification {
            NotificationPolicy::Immediate if self.available_at_ms != expected => {
                return Err(ValidationError::ImmediateNotificationAvailabilityMismatch)
            }
            NotificationPolicy::Delayed(_) if self.available_at_ms != expected => {
                return Err(ValidationError::DelayedNotificationAvailabilityMismatch)
            }
            _ => {}
        }
        require(&self.human_summary, "human_summary")?;
        if self.contest_ref.trim().is_empty() {
            return Err(ValidationError::EmptyContestRoute);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn capability() -> QueryCapability {
        QueryCapability {
            protocol_version: PROTOCOL_VERSION.into(),
            capability_id: "cap-1".into(),
            requester: ActorRef { id: "did:example:officer".into(), organization: Some("Example Agency".into()) },
            scope: LookupScope::PersonLinked { subject: SubjectRef { id: "did:example:citizen".into() } },
            purpose: Purpose { code: "vehicle.theft.recovery".into(), description: "Locate a reported stolen vehicle".into() },
            authority: AuthorityRef { id: "auth-1".into(), authority_type: "case-delegation".into() },
            issued_at_ms: 1_000,
            expires_at_ms: 2_000,
            max_disclosure: DisclosureClass::PredicateOnly,
            person_query_budget: 1,
            policy_ref: "policy:v1".into(),
            authorization_proof_ref: "proof:cap".into(),
        }
    }

    fn receipt(notification: NotificationPolicy) -> AccessReceipt {
        AccessReceipt {
            protocol_version: PROTOCOL_VERSION.into(),
            receipt_id: "receipt-1".into(),
            capability_id: "cap-1".into(),
            subject: SubjectRef { id: "did:example:citizen".into() },
            requester: ActorRef { id: "did:example:officer".into(), organization: Some("Example Agency".into()) },
            purpose: Purpose { code: "vehicle.theft.recovery".into(), description: "Locate a reported stolen vehicle".into() },
            authority: AuthorityRef { id: "auth-1".into(), authority_type: "case-delegation".into() },
            evaluated_at_ms: 1_200,
            decision: AccessDecision { kind: DecisionKind::Allowed, reason_code: "authorized".into(), policy_ref: "policy:v1".into(), decision_proof_ref: "proof:decision".into() },
            disclosure: DisclosureResult::Predicate { result_ref: "proof:match".into() },
            notification,
            session_provenance_ref: "xenia:session:1".into(),
            previous_receipt_ref: None,
            receipt_proof_ref: "proof:receipt".into(),
        }
    }

    #[test]
    fn person_linked_lookup_requires_nonzero_budget() {
        let mut cap = capability();
        cap.person_query_budget = 0;
        assert_eq!(cap.validate(), Err(ValidationError::PersonLookupRequiresBudget));
    }

    #[test]
    fn aggregate_capability_cannot_hide_person_budget() {
        let mut cap = capability();
        cap.scope = LookupScope::AggregateOnly { cohort_policy: "dp:traffic-volume:v1".into() };
        assert_eq!(cap.validate(), Err(ValidationError::AggregateLookupMustNotHavePersonBudget));
    }

    #[test]
    fn denied_lookup_must_disclose_nothing_but_still_has_receipt() {
        let cap = capability();
        let mut r = receipt(NotificationPolicy::Immediate);
        r.decision.kind = DecisionKind::Denied;
        r.disclosure = DisclosureResult::Predicate { result_ref: "should-not-exist".into() };
        assert_eq!(r.validate_against(&cap), Err(ValidationError::DeniedLookupDisclosedData));
        r.disclosure = DisclosureResult::None;
        assert!(r.validate_against(&cap).is_ok());
    }

    #[test]
    fn immediate_notice_is_bound_to_lookup_time() {
        let cap = capability();
        let r = receipt(NotificationPolicy::Immediate);
        assert!(r.validate_against(&cap).is_ok());
        let n = CitizenNotification {
            protocol_version: PROTOCOL_VERSION.into(),
            notification_id: "notice-1".into(),
            receipt_id: r.receipt_id.clone(),
            subject: r.subject.clone(),
            available_at_ms: r.evaluated_at_ms,
            state: DeliveryState::Pending,
            human_summary: "Your record was queried for a stolen-vehicle investigation.".into(),
            contest_ref: "mycelix://contest/receipt-1".into(),
        };
        assert!(n.validate_against(&r).is_ok());
    }

    #[test]
    fn delayed_notice_requires_independent_authorization_and_hard_release() {
        let cap = capability();
        let delay = DelayedNotificationAuthorization {
            authorization_id: "delay-1".into(),
            justification_code: "active-investigation-risk".into(),
            authority_ref: AuthorityRef { id: "court-order-7".into(), authority_type: "judicial-order".into() },
            approved_by: vec![ActorRef { id: "did:example:judge".into(), organization: Some("Example Court".into()) }],
            created_at_ms: 1_100,
            release_at_ms: 1_800,
            mandatory_release_at_ms: 2_500,
            authorization_proof_ref: "proof:delay".into(),
        };
        let r = receipt(NotificationPolicy::Delayed(delay));
        assert!(r.validate_against(&cap).is_ok());
        assert_eq!(r.notification_available_at_ms(), 1_800);
    }

    #[test]
    fn delayed_notice_cannot_outlive_mandatory_release() {
        let mut delay = DelayedNotificationAuthorization {
            authorization_id: "delay-1".into(),
            justification_code: "active-investigation-risk".into(),
            authority_ref: AuthorityRef { id: "court-order-7".into(), authority_type: "judicial-order".into() },
            approved_by: vec![ActorRef { id: "did:example:judge".into(), organization: None }],
            created_at_ms: 1_100,
            release_at_ms: 3_000,
            mandatory_release_at_ms: 2_500,
            authorization_proof_ref: "proof:delay".into(),
        };
        assert_eq!(delay.validate(), Err(ValidationError::DelayReleaseAfterMandatoryRelease));
        delay.release_at_ms = 2_000;
        assert!(delay.validate().is_ok());
    }

    #[test]
    fn field_disclosure_cannot_exceed_declared_capability() {
        let mut cap = capability();
        cap.max_disclosure = DisclosureClass::MinimalFields { fields: vec!["plate".into()] };
        let mut r = receipt(NotificationPolicy::Immediate);
        r.disclosure = DisclosureResult::Fields { field_names: vec!["plate".into(), "home_address".into()], artifact_ref: "artifact:1".into() };
        assert_eq!(r.validate_against(&cap), Err(ValidationError::DisclosureExceedsCapability));
    }

    #[test]
    fn wire_shape_round_trips() {
        let cap = capability();
        let json = serde_json::to_string(&cap).unwrap();
        let decoded: QueryCapability = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, cap);
    }
}
