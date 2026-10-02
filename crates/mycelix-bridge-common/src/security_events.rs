// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Structured, append-only-friendly security evidence vocabulary.
//!
//! These types describe what happened; they do not make an authorization true.
//! A SecurityEvent may therefore record Allow, Deny, or Indeterminate outcomes.
//! Provenance records origin and transformation context without asserting that
//! the underlying claim is true.

use serde::{Deserialize, Serialize};

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
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SecurityEvent {
    pub event_id: String,
    pub actor_id: String,
    pub capability_ref: String,
    pub request: AuthorizationRequest,
    pub decision: AuthorizationDecision,
    pub policy_version: u64,
    pub timestamp_us: u64,
    /// Kernel-derived commitment of the exact capability used for a successful
    /// enforcement request. Directly constructed events may leave this absent.
    #[serde(default)]
    pub capability_binding: Option<[u8; 32]>,
    pub provenance: Vec<ProvenanceRef>,
    pub recovery_correlation: Option<String>,
}

impl SecurityEvent {
    pub fn new(
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

        Ok(Self {
            event_id,
            actor_id,
            capability_ref,
            request,
            decision,
            policy_version,
            timestamp_us,
            capability_binding: None,
            provenance: Vec::new(),
            recovery_correlation: None,
        })
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
    /// have received an EnforcementRequest derived from the permit.
    pub fn from_enforcement_request(
        event_id: impl Into<String>,
        actor_id: impl Into<String>,
        capability_ref: impl Into<String>,
        enforcement: &EnforcementRequest,
        policy_version: u64,
        timestamp_us: u64,
    ) -> Result<Self, &'static str> {
        let mut event = Self::new(
            event_id,
            actor_id,
            capability_ref,
            enforcement.request().clone(),
            AuthorizationDecision::Allow,
            policy_version,
            timestamp_us,
        )?;
        event.capability_binding = Some(enforcement.capability_binding());
        Ok(event)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::security_kernel::{
        Capability, CapabilityAction, VerificationEvidence, VerifiedCapability, authorize_permit,
    };

    fn capability() -> Capability {
        Capability::new(
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

    fn verified() -> VerifiedCapability {
        let capability = capability();
        crate::security_kernel::verify_capability(
            capability.clone(),
            VerificationEvidence::new_for_capability(&capability, true, true, true),
            150,
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
        let source =
            ProvenanceRef::new("evidence:source-1", ProvenanceRelation::DerivedFrom).unwrap();

        let event = SecurityEvent::from_enforcement_request(
            "event:1",
            "did:mycelix:alice",
            "capability:1",
            &enforcement,
            7,
            151,
        )
        .unwrap()
        .with_provenance(vec![source])
        .with_recovery_correlation("recovery:1")
        .unwrap();

        assert_eq!(event.decision, AuthorizationDecision::Allow);
        assert_eq!(event.request, *enforcement.request());
        assert_eq!(event.provenance.len(), 1);
        assert_eq!(event.recovery_correlation.as_deref(), Some("recovery:1"));
    }

    #[test]
    fn security_event_can_record_denial() {
        let request = crate::security_kernel::AuthorizationRequest::new(
            "did:mycelix:alice",
            "resource:ledger",
            CapabilityAction::Admin,
            7,
        )
        .unwrap();
        let event = SecurityEvent::new(
            "event:deny",
            "did:mycelix:alice",
            "capability:1",
            request,
            AuthorizationDecision::Deny(
                crate::security_kernel::AuthorizationDenial::ActionNotGranted,
            ),
            7,
            151,
        )
        .unwrap();
        assert!(matches!(event.decision, AuthorizationDecision::Deny(_)));
    }
}
