// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Structured, append-only-friendly security evidence vocabulary.
//!
//! These types describe what happened; they do not make an authorization true.
//! A SecurityEvent may therefore record Allow, Deny, or Indeterminate outcomes.
//! Provenance records origin and transformation context without asserting that
//! the underlying claim is true.

use serde::{de::Error as _, Deserialize, Deserializer, Serialize};

use crate::security_kernel::{AuthorizationDecision, AuthorizationRequest, EnforcementRequest};

pub const MAX_PROVENANCE_IDENTIFIER_BYTES: usize = 512;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProvenanceRef {
    pub artifact_id: String,
    pub relation: ProvenanceRelation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProvenanceRelation {
    DerivedFrom,
    TransformedFrom,
    Summarizes,
    References,
    Supersedes,
}

impl ProvenanceRef {
    pub fn new(
        artifact_id: impl Into<String>,
        relation: ProvenanceRelation,
    ) -> Result<Self, &'static str> {
        let artifact_id = artifact_id.into();
        if artifact_id.is_empty() {
            return Err("provenance artifact identifier cannot be empty");
        }
        if artifact_id.len() > MAX_PROVENANCE_IDENTIFIER_BYTES {
            return Err("provenance artifact identifier exceeds size limit");
        }
        Ok(Self {
            artifact_id,
            relation,
        })
    }
}

/// A structured record of an authorization-relevant security event.
///
/// This is evidence for reconstruction and audit. It is intentionally not an
/// authority primitive and must not be used as a substitute for a fresh
/// authorization decision.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct SecurityEvent {
    event_id: String,
    actor_id: String,
    capability_ref: String,
    request: AuthorizationRequest,
    decision: AuthorizationDecision,
    policy_version: u64,
    timestamp_us: u64,
    /// Kernel-derived commitment of the exact capability used for a successful
    /// enforcement request. Directly constructed events may leave this absent.
    #[serde(default)]
    capability_binding: Option<[u8; 32]>,
    /// Opaque commitment for the exact authority generation/freshness state
    /// that qualified a successful enforcement request.
    #[serde(default)]
    authority_binding: Option<[u8; 32]>,
    provenance: Vec<ProvenanceRef>,
    recovery_correlation: Option<String>,
}

#[derive(Debug, Deserialize)]
struct SecurityEventWire {
    event_id: String,
    actor_id: String,
    capability_ref: String,
    request: AuthorizationRequest,
    decision: AuthorizationDecision,
    policy_version: u64,
    timestamp_us: u64,
    #[serde(default)]
    capability_binding: Option<[u8; 32]>,
    #[serde(default)]
    authority_binding: Option<[u8; 32]>,
    provenance: Vec<ProvenanceRef>,
    recovery_correlation: Option<String>,
}

impl<'de> serde::Deserialize<'de> for SecurityEvent {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let wire = SecurityEventWire::deserialize(deserializer)?;
        let mut event = Self::new_internal(
            wire.event_id,
            wire.actor_id,
            wire.capability_ref,
            wire.request,
            wire.decision.clone(),
            wire.policy_version,
            wire.timestamp_us,
        )
        .map_err(D::Error::custom)?;

        if matches!(&wire.decision, AuthorizationDecision::Allow) {
            if wire.capability_binding.is_none() || wire.authority_binding.is_none() {
                return Err(D::Error::custom(
                    "allow security events require kernel capability and authority bindings",
                ));
            }
            if wire.capability_binding == Some([0; 32]) || wire.authority_binding == Some([0; 32]) {
                return Err(D::Error::custom(
                    "allow security event bindings must be non-zero",
                ));
            }
            if event.actor_id != event.request.subject() {
                return Err(D::Error::custom(
                    "allow security events require actor to match request subject",
                ));
            }
        } else if wire.capability_binding.is_some() || wire.authority_binding.is_some() {
            return Err(D::Error::custom(
                "non-Allow security events cannot carry enforcement bindings",
            ));
        }

        event.capability_binding = wire.capability_binding;
        event.authority_binding = wire.authority_binding;
        event.provenance = wire.provenance;
        event.recovery_correlation = wire.recovery_correlation;
        Ok(event)
    }
}

impl SecurityEvent {
    pub fn event_id(&self) -> &str {
        &self.event_id
    }

    pub fn actor_id(&self) -> &str {
        &self.actor_id
    }

    pub fn capability_ref(&self) -> &str {
        &self.capability_ref
    }

    pub fn request(&self) -> &AuthorizationRequest {
        &self.request
    }

    pub fn decision(&self) -> &AuthorizationDecision {
        &self.decision
    }

    pub fn policy_version(&self) -> u64 {
        self.policy_version
    }

    pub fn timestamp_us(&self) -> u64 {
        self.timestamp_us
    }

    pub fn capability_binding(&self) -> Option<[u8; 32]> {
        self.capability_binding
    }

    pub fn authority_binding(&self) -> Option<[u8; 32]> {
        self.authority_binding
    }

    pub fn provenance(&self) -> &[ProvenanceRef] {
        &self.provenance
    }

    pub fn recovery_correlation(&self) -> Option<&str> {
        self.recovery_correlation.as_deref()
    }

    fn new_internal(
        event_id: impl Into<String>,
        actor_id: impl Into<String>,
        capability_ref: impl Into<String>,
        request: AuthorizationRequest,
        decision: AuthorizationDecision,
        policy_version: u64,
        timestamp_us: u64,
    ) -> Result<Self, &'static str> {
        let event_id = event_id.into();
        let actor_id = actor_id.into();
        let capability_ref = capability_ref.into();

        if event_id.is_empty() || actor_id.is_empty() || capability_ref.is_empty() {
            return Err("security event identifiers cannot be empty");
        }
        if event_id.len() > MAX_PROVENANCE_IDENTIFIER_BYTES
            || actor_id.len() > MAX_PROVENANCE_IDENTIFIER_BYTES
            || capability_ref.len() > MAX_PROVENANCE_IDENTIFIER_BYTES
        {
            return Err("security event identifier exceeds size limit");
        }
        if policy_version != request.policy_version() {
            return Err("security event policy version does not match request");
        }

        Ok(Self {
            event_id,
            actor_id,
            capability_ref,
            request,
            decision,
            policy_version,
            timestamp_us,
            capability_binding: None,
            authority_binding: None,
            provenance: Vec::new(),
            recovery_correlation: None,
        })
    }

    pub fn new(
        event_id: impl Into<String>,
        actor_id: impl Into<String>,
        capability_ref: impl Into<String>,
        request: AuthorizationRequest,
        decision: AuthorizationDecision,
        policy_version: u64,
        timestamp_us: u64,
    ) -> Result<Self, &'static str> {
        if matches!(&decision, AuthorizationDecision::Allow) {
            return Err("allow security events must originate from enforcement");
        }
        Self::new_internal(
            event_id,
            actor_id,
            capability_ref,
            request,
            decision,
            policy_version,
            timestamp_us,
        )
    }

    pub fn with_provenance(mut self, provenance: Vec<ProvenanceRef>) -> Self {
        self.provenance = provenance;
        self
    }

    pub fn with_recovery_correlation(
        mut self,
        correlation: impl Into<String>,
    ) -> Result<Self, &'static str> {
        let correlation = correlation.into();
        if correlation.is_empty() {
            return Err("recovery correlation cannot be empty");
        }
        if correlation.len() > MAX_PROVENANCE_IDENTIFIER_BYTES {
            return Err("recovery correlation exceeds size limit");
        }
        self.recovery_correlation = Some(correlation);
        Ok(self)
    }

    /// Record the exact request carried by an enforcement permit.
    ///
    /// This does not re-authorize the operation; enforcement should already
    /// have received an EnforcementRequest derived from the permit. The event
    /// timestamp must equal the revalidation timestamp carried by that request.
    pub fn from_enforcement_request(
        event_id: impl Into<String>,
        actor_id: impl Into<String>,
        capability_ref: impl Into<String>,
        enforcement: &EnforcementRequest,
        policy_version: u64,
        timestamp_us: u64,
    ) -> Result<Self, &'static str> {
        let actor_id = actor_id.into();
        if actor_id != enforcement.request().subject() {
            return Err("security event actor does not match enforcement subject");
        }
        if policy_version != enforcement.request().policy_version() {
            return Err("security event policy version does not match enforcement request");
        }
        if timestamp_us != enforcement.revalidated_at_us() {
            return Err("security event timestamp does not match enforcement revalidation time");
        }
        let mut event = Self::new_internal(
            event_id,
            actor_id,
            capability_ref,
            enforcement.request().clone(),
            AuthorizationDecision::Allow,
            policy_version,
            timestamp_us,
        )?;
        event.capability_binding = Some(enforcement.capability_binding());
        event.authority_binding = Some(enforcement.authority_binding());
        Ok(event)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::security_kernel::CapabilityAction;


        crate::security_kernel::Capability::new(
            "did:mycelix:alice",
            "did:mycelix:issuer",
            "resource:ledger",
            vec![CapabilityAction::Read],
            100,
            200,
            7,
        )
        .unwrap()
    }


    #[test]
    fn provenance_ref_rejects_empty_and_oversized_ids() {
        assert!(ProvenanceRef::new("", ProvenanceRelation::References).is_err());
        assert!(
            ProvenanceRef::new(
                "x".repeat(MAX_PROVENANCE_IDENTIFIER_BYTES + 1),
                ProvenanceRelation::References
            )
            .is_err()
        );
    }

    #[test]
    fn security_event_preserves_allow_decision_and_lineage() {
        let enforcement = crate::security_kernel::test_enforcement_request();
        let source =
            ProvenanceRef::new("evidence:source-1", ProvenanceRelation::DerivedFrom).unwrap();

        let event = SecurityEvent::from_enforcement_request(
            "event:1",
            "did:mycelix:alice",
            "capability:1",
            &enforcement,
            7,
            150,
        )
        .unwrap()
        .with_provenance(vec![source])
        .with_recovery_correlation("recovery:1")
        .unwrap();

        assert_eq!(event.decision(), &AuthorizationDecision::Allow);
        assert_eq!(event.request(), enforcement.request());
        assert_eq!(event.provenance().len(), 1);
        assert_eq!(event.recovery_correlation(), Some("recovery:1"));
        assert_eq!(
            event.capability_binding(),
            Some(enforcement.capability_binding())
        );
        assert_eq!(
            event.authority_binding(),
            Some(enforcement.authority_binding())
        );
    }

    #[test]
    fn enforcement_event_rejects_actor_mismatch() {
        let request = crate::security_kernel::AuthorizationRequest::new(
            "did:mycelix:alice",
            "resource:ledger",
            CapabilityAction::Read,
            7,
        )
        .unwrap();
        let permit = authorize_permit(&verified(), &request, 150).unwrap();
        let enforcement = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability(&capability(), true, true, true),
            150,
        )
        .unwrap();

        assert_eq!(
            SecurityEvent::from_enforcement_request(
                "event:actor-mismatch",
                "did:mycelix:bob",
                "capability:read",
                &enforcement,
                7,
                150,
            )
            .unwrap_err(),
            "security event actor does not match enforcement subject"
        );
    }

    #[test]
    fn legacy_security_event_without_binding_deserializes_as_deny() {
        let json = r#"{
            "event_id":"event:legacy",
            "actor_id":"did:mycelix:alice",
            "capability_ref":"capability:legacy",
            "request":{
                "subject":"did:mycelix:alice",
                "resource":"resource:ledger",
                "action":"Read",
                "policy_version":7
            },
            "decision":"Deny",
            "policy_version":7,
            "timestamp_us":151,
            "provenance":[],
            "recovery_correlation":null
        }"#;
        let event: SecurityEvent = serde_json::from_str(json).unwrap();
        assert_eq!(
            event.decision(),
            &AuthorizationDecision::Deny(
                crate::security_kernel::AuthorizationDenial::ActionNotGranted
            )
        );
        assert_eq!(event.capability_binding(), None);
    }

    #[test]
    fn deserialization_rejects_unbound_allow() {
        let json = r#"{
            "event_id":"event:forged-allow",
            "actor_id":"did:mycelix:alice",
            "capability_ref":"capability:forged",
            "request":{
                "subject":"did:mycelix:alice",
                "resource":"resource:ledger",
                "action":"Read",
                "policy_version":7
            },
            "decision":"Allow",
            "policy_version":7,
            "timestamp_us":151,
            "provenance":[],
            "recovery_correlation":null
        }"#;

        assert!(serde_json::from_str::<SecurityEvent>(json).is_err());
    }

    #[test]
    fn deserialization_rejects_non_allow_bindings() {
        let json = r#"{
            "event_id":"event:forged-deny-binding",
            "actor_id":"did:mycelix:alice",
            "capability_ref":"capability:forged",
            "request":{
                "subject":"did:mycelix:alice",
                "resource":"resource:ledger",
                "action":"Read",
                "policy_version":7
            },
            "decision":{
                "Deny":"ActionNotGranted"
            },
            "policy_version":7,
            "timestamp_us":151,
            "capability_binding":[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
            "authority_binding":[2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2],
            "provenance":[],
            "recovery_correlation":null
        }"#;

        assert!(serde_json::from_str::<SecurityEvent>(json).is_err());
    }

    #[test]
    fn deserialization_rejects_zero_allow_bindings() {
        let json = r#"{
            "event_id":"event:zero-allow-binding",
            "actor_id":"did:mycelix:alice",
            "capability_ref":"capability:zero",
            "request":{
                "subject":"did:mycelix:alice",
                "resource":"resource:ledger",
                "action":"Read",
                "policy_version":7
            },
            "decision":"Allow",
            "policy_version":7,
            "timestamp_us":151,
            "capability_binding":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
            "authority_binding":[3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3],
            "provenance":[],
            "recovery_correlation":null
        }"#;

        assert!(serde_json::from_str::<SecurityEvent>(json).is_err());
    }

    #[test]
    fn serialized_enforcement_event_round_trips() {
        let request = crate::security_kernel::AuthorizationRequest::new(
            "did:mycelix:alice",
            "resource:ledger",
            CapabilityAction::Read,
            7,
        )
        .unwrap();
        let permit = authorize_permit(&verified(), &request, 150).unwrap();
        let enforcement = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability(&capability(), true, true, true),
            150,
        )
        .unwrap();
        let event = SecurityEvent::from_enforcement_request(
            "event:round-trip",
            "did:mycelix:alice",
            "capability:1",
            &enforcement,
            7,
            150,
        )
        .unwrap();

        let encoded = serde_json::to_string(&event).unwrap();
        let decoded: SecurityEvent = serde_json::from_str(&encoded).unwrap();

        assert_eq!(decoded, event);
        assert_eq!(decoded.decision(), &AuthorizationDecision::Allow);
        assert!(decoded.capability_binding().is_some());
        assert!(decoded.authority_binding().is_some());
    }

    #[test]
    fn enforcement_event_rejects_policy_version_mismatch() {
        let request = crate::security_kernel::AuthorizationRequest::new(
            "did:mycelix:alice",
            "resource:ledger",
            CapabilityAction::Read,
            7,
        )
        .unwrap();
        let permit = authorize_permit(&verified(), &request, 150).unwrap();
        let enforcement = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability(&capability(), true, true, true),
            150,
        )
        .unwrap();
        assert_eq!(
            SecurityEvent::from_enforcement_request(
                "event:mismatch",
                "did:mycelix:alice",
                "capability:1",
                &enforcement,
                8,
                151,
            )
            .unwrap_err(),
            "security event policy version does not match enforcement request"
        );
    }

    #[test]
    fn security_event_constructor_rejects_allow() {
        let request = crate::security_kernel::AuthorizationRequest::new(
            "did:mycelix:alice",
            "resource:ledger",
            CapabilityAction::Read,
            7,
        )
        .unwrap();

        assert_eq!(
            SecurityEvent::new(
                "event:allow",
                "did:mycelix:alice",
                "capability:1",
                request,
                AuthorizationDecision::Allow,
                7,
                150,
            )
            .unwrap_err(),
            "allow security events must originate from enforcement"
        );
    }

    #[test]
    fn security_event_constructor_rejects_policy_version_mismatch() {
        let request = crate::security_kernel::AuthorizationRequest::new(
            "did:mycelix:alice",
            "resource:ledger",
            CapabilityAction::Read,
            7,
        )
        .unwrap();

        assert_eq!(
            SecurityEvent::new(
                "event:mismatch",
                "did:mycelix:alice",
                "capability:1",
                request,
                AuthorizationDecision::Deny(
                    crate::security_kernel::AuthorizationDenial::ActionNotGranted,
                ),
                8,
                150,
            )
            .unwrap_err(),
            "security event policy version does not match request"
        );
    }
}
